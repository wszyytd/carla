"""Server boundary for a single-target, stationary-camera preview."""

from __future__ import annotations

import math
import subprocess
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from ..client import versions_compatible
from ..metrics import (
    build_projection_matrix,
    count_dominant_vehicle_instance_pixels,
    project_bounding_box,
)
from ..runtime import OwnedActors, apply_camera_capture_settings
from ..sensors import spawn_paired_camera_rig
from .artifacts import digest, finish_artifacts, save_images, write_json
from .preview import blocked_by, matches_view, pose_values, poses_close, preview_views


def connect(carla, config):
    client = carla.Client(config.host, config.port)
    client.set_timeout(config.timeout_seconds)
    versions = [str(client.get_client_version()), str(client.get_server_version())]
    if not versions_compatible(*versions):
        raise RuntimeError(f"CARLA major/minor version mismatch: {versions}")
    return client, versions


def inventory(carla, config):
    client, versions = connect(carla, config)
    world = client.get_world()
    return _inventory(world, config, versions)


def _inventory(world, config, versions):
    mapping, actors = world.get_map(), world.get_actors()
    vehicles = []
    for bp in sorted(world.get_blueprint_library().filter("vehicle.*"), key=lambda b: b.id):
        wheels = (
            bp.get_attribute("number_of_wheels").as_int()
            if bp.has_attribute("number_of_wheels")
            else 0
        )
        if wheels == 4:
            vehicles.append({"id": bp.id, "number_of_wheels": wheels})
    return {
        "schema_version": 1,
        "client": asdict(config),
        "versions": versions,
        "map": str(mapping.name),
        "vehicles": vehicles,
        "spawn_points": [
            {"index": i, "pose": list(pose_values(p))}
            for i, p in enumerate(mapping.get_spawn_points())
        ],
        "actor_counts": {
            "all": len(actors),
            "vehicles": len(actors.filter("vehicle.*")),
            "walkers": len(actors.filter("walker.pedestrian.*")),
        },
    }


def transform(carla, values):
    x, y, z, pitch, yaw, roll = values
    return carla.Transform(
        carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


def geometry(carla, world):
    """Conservative AABBs for point exclusion, not a flight collision guarantee."""
    labels = (
        "Buildings",
        "Walls",
        "Fences",
        "Poles",
        "Vegetation",
        "Static",
        "Bridge",
        "GuardRail",
        "TrafficLight",
        "TrafficSigns",
    )
    boxes, counts = [], {}
    for name in labels:
        tag = getattr(carla.CityObjectLabel, name, None)
        if tag is None:
            counts[name] = None
            continue
        native = world.get_level_bbs(tag)
        counts[name] = len(native)
        for box in native:
            vertices = np.asarray(
                [[p.x, p.y, p.z] for p in box.get_world_vertices(carla.Transform())]
            )
            if vertices.shape != (8, 3) or not np.isfinite(vertices).all():
                raise RuntimeError(f"invalid static bounding box for {name}")
            boxes.append((vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()))
    if counts["Buildings"] is None:
        raise RuntimeError("CARLA API lacks Buildings geometry query")
    return boxes, counts


def await_view(world, rig, desired, *, min_frame, config):
    deadline = time.monotonic() + config.timeout_seconds
    for _ in range(config.max_frame_ticks):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        frame = int(world.tick(min(config.client.timeout_seconds, remaining)))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            pairs = rig.wait_for_ready(max_frame=frame, timeout_seconds=min(0.2, remaining))
        except TimeoutError:
            continue
        for pair in pairs:
            if matches_view(pair, desired, min_frame=min_frame):
                return pair
    raise TimeoutError("no synchronized camera pair matching the requested view")


def _weather(world):
    current = world.get_weather()
    names = (
        "cloudiness",
        "precipitation",
        "precipitation_deposits",
        "wind_intensity",
        "sun_azimuth_angle",
        "sun_altitude_angle",
        "fog_density",
        "fog_distance",
        "fog_falloff",
        "wetness",
        "scattering_intensity",
        "mie_scattering_scale",
        "rayleigh_scattering_scale",
        "dust_storm",
    )
    return {name: float(getattr(current, name)) for name in names if hasattr(current, name)}


def _git_revision():
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).parents[3],
            capture_output=True,
            text=True,
            check=True,
            timeout=3,
        )
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def _capture_class(
    carla, world, config, root, index, blueprint_id, boxes, nodes, metadata, stream, cleanup, report
):
    import json

    actors = OwnedActors()
    try:
        blueprint = world.get_blueprint_library().find(blueprint_id)
        target = actors.add(world.try_spawn_actor(blueprint, transform(carla, config.target_pose)))
        if target is None:
            raise RuntimeError(f"cannot spawn {blueprint_id}; choose a clear spawn point")
        target.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        previous = target.get_transform()
        for _ in range(config.settle_ticks):
            previous = target.get_transform()
            world.tick(config.client.timeout_seconds)
        settled = target.get_transform()
        speed = target.get_velocity()
        if math.sqrt(speed.x**2 + speed.y**2 + speed.z**2) > 0.1 or not poses_close(
            previous, settled, position_m=0.02, angle_deg=0.5
        ):
            raise RuntimeError("target did not settle; increase settle_ticks or change spawn point")
        if math.dist(config.target_pose[:2], pose_values(settled)[:2]) > 1:
            raise RuntimeError("target drifted horizontally by more than 1 m during settling")
        target.set_simulate_physics(False)
        world.tick(config.client.timeout_seconds)
        frozen = target.get_transform()
        views = preview_views(
            config.center,
            ground_z=config.ground_z,
            heights_m=config.heights_m,
            horizontal_offset_m=config.horizontal_offset_m,
        )
        first = views[0]
        rig = spawn_paired_camera_rig(
            world,
            config.camera,
            transform(carla, (first.x, first.y, first.z, first.pitch, first.yaw, first.roll)),
            actors,
        )
        metadata["targets"].append(
            {
                "class_index": index,
                "blueprint_id": blueprint_id,
                "pose": list(pose_values(frozen)),
                "semantic_tags": list(target.semantic_tags),
                "spawn_drift_m": math.dist(config.target_pose[:3], pose_values(frozen)[:3]),
                "attributes": dict(target.attributes),
                "camera_attributes": {
                    "rgb": dict(rig.rgb_sensor.attributes),
                    "instance": dict(rig.instance_sensor.attributes),
                },
            }
        )
        write_json(root / "metadata.json", metadata)
        vertices = np.asarray(
            [[p.x, p.y, p.z] for p in target.bounding_box.get_world_vertices(frozen)]
        )
        intrinsics = build_projection_matrix(
            width=config.camera.width, height=config.camera.height, fov_deg=config.camera.fov_deg
        )
        for view in views:
            node = {
                "class_index": index,
                "blueprint_id": blueprint_id,
                "view": asdict(view),
                "target_pose": list(pose_values(frozen)),
            }
            collision = blocked_by((view.x, view.y, view.z), boxes, clearance_m=config.clearance_m)
            if collision:
                node.update(
                    status="rejected",
                    reason="static_geometry",
                    blocking_box_indices=list(collision),
                )
            else:
                desired = transform(
                    carla, (view.x, view.y, view.z, view.pitch, view.yaw, view.roll)
                )
                rig.set_transform(desired)
                for _ in range(config.warmup_ticks):
                    barrier = int(world.tick(config.client.timeout_seconds))
                pair = await_view(world, rig, desired, min_frame=barrier, config=config)
                if not poses_close(target.get_transform(), frozen, position_m=0.01, angle_deg=0.1):
                    raise RuntimeError("frozen target moved during capture")
                for image in (pair.rgb, pair.instance):
                    if (image.width, image.height) != (config.camera.width, config.camera.height):
                        raise RuntimeError("camera dimensions differ from configuration")
                box = project_bounding_box(
                    vertices, np.asarray(pair.rgb.transform.get_inverse_matrix()), intrinsics
                )
                node.update(
                    status="captured",
                    distance_m=math.dist(
                        pose_values(pair.rgb.transform)[:3], pose_values(frozen)[:3]
                    ),
                )
                node["camera_position_error_m"] = max(
                    math.dist(pose_values(image.transform)[:3], pose_values(desired)[:3])
                    for image in (pair.rgb, pair.instance)
                )
                node["camera_angle_error_deg"] = max(
                    abs((actual - requested + 180) % 360 - 180)
                    for image in (pair.rgb, pair.instance)
                    for actual, requested in zip(
                        pose_values(image.transform)[3:], pose_values(desired)[3:], strict=True
                    )
                )
                save_images(root, node, pair, box)
                node["instance_pixels_heuristic"] = count_dominant_vehicle_instance_pixels(
                    pair.instance.raw_data,
                    width=config.camera.width,
                    height=config.camera.height,
                    projected_box=box,
                    target_semantic_tags=target.semantic_tags,
                )
            stream.write(json.dumps(node, allow_nan=False) + "\n")
            stream.flush()
            nodes.append(node)
            report(f"class {index + 1}/{len(config.blueprint_ids)} {view.key}: {node['status']}")
    finally:
        failures = actors.destroy_all()
        cleanup.extend(asdict(f) for f in failures)
        if failures:
            report("Actor cleanup failed; inspect summary.json")


def capture(carla, config, output, report=lambda text: None):
    root = Path(output)
    # Never overwrite a prior run, including an incomplete one.
    root.mkdir(parents=True, exist_ok=False)
    external_report = report

    def report(message):
        stamp = datetime.now(timezone.utc).isoformat()
        with (root / "capture.log").open("a", encoding="utf-8") as log:
            log.write(f"{stamp} {message}\n")
        external_report(message)

    nodes, cleanup = [], []
    summary = {
        "schema_version": 1,
        "status": "incomplete",
        "planned": 24 * len(config.blueprint_ids),
        "class_count": len(config.blueprint_ids),
        "captured": 0,
        "rejected": 0,
        "cleanup_failures": cleanup,
        "qualification": "preview_only_unreviewed",
    }
    metadata = {
        "schema_version": 1,
        "git_revision": _git_revision(),
        "targets": [],
        "geometry_scope": "inflated static AABB point exclusion only; no edge validation",
        "instance_metric": "dominant vehicle instance heuristic, not exact actor-ID mask",
    }
    write_json(root / "config.json", config.resolved)
    (root / "capture_config.resolved.yaml").write_text(
        yaml.safe_dump(config.resolved, sort_keys=False), encoding="utf-8"
    )
    metadata["config_sha256"] = digest(root / "config.json")
    write_json(root / "metadata.json", metadata)
    world, original = None, None
    try:
        client, versions = connect(carla, config.client)
        world = client.get_world()
        name = str(world.get_map().name)
        if name.split("/")[-1] != config.map_name.split("/")[-1]:
            raise RuntimeError(f"map mismatch: expected {config.map_name}, got {name}")
        existing = world.get_actors()
        if existing.filter("vehicle.*") or existing.filter("walker.pedestrian.*"):
            raise RuntimeError("world occupied by vehicles/walkers; stop other traffic clients")
        snapshot = _inventory(world, config.client, versions)
        available = {item["id"] for item in snapshot["vehicles"]}
        if not set(config.blueprint_ids) <= available:
            raise ValueError("selected blueprints are missing from current four-wheel inventory")
        write_json(root / "inventory.json", snapshot)
        report("Starting stationary preview capture")
        boxes, counts = geometry(carla, world)
        write_json(
            root / "region_geometry.json",
            {
                "scope": metadata["geometry_scope"],
                "counts": counts,
                "aabbs": boxes,
                "clearance_m": config.clearance_m,
                "manual_review": "pending",
            },
        )
        metadata.update(
            versions=versions,
            map=name,
            weather=_weather(world),
            geometry_counts=counts,
            geometry_file="region_geometry.json",
        )
        write_json(root / "metadata.json", metadata)
        original = world.get_settings()
        apply_camera_capture_settings(world, config.fixed_delta_seconds)
        with (root / "nodes.jsonl").open("w", encoding="utf-8") as stream:
            for index, blueprint_id in enumerate(config.blueprint_ids):
                _capture_class(
                    carla,
                    world,
                    config,
                    root,
                    index,
                    blueprint_id,
                    boxes,
                    nodes,
                    metadata,
                    stream,
                    cleanup,
                    report,
                )
                if cleanup:
                    raise RuntimeError("actor cleanup failed; refusing to spawn another class")
        if not any(n["status"] == "captured" for n in nodes):
            raise RuntimeError("all preview positions blocked; choose another spawn point")
        summary["status"] = "complete"
    except BaseException as error:
        summary["error"] = f"{type(error).__name__}: {error}"
        report(summary["error"])
        raise
    finally:
        if world is not None and original is not None:
            try:
                world.apply_settings(original)
            except Exception as error:
                cleanup.append(
                    {"actor_id": "world", "operation": "restore_settings", "message": str(error)}
                )
        if cleanup:
            summary["status"] = "incomplete"
        summary["captured"] = sum(n["status"] == "captured" for n in nodes)
        summary["rejected"] = sum(n["status"] == "rejected" for n in nodes)
        if not (root / "nodes.jsonl").exists():
            (root / "nodes.jsonl").touch()
        summary["errors"] = int("error" in summary)
        summary["not_recorded"] = summary["planned"] - len(nodes)
        summary["max_camera_position_error_m"] = max(
            (n.get("camera_position_error_m", 0) for n in nodes), default=0
        )
        summary["max_camera_angle_error_deg"] = max(
            (n.get("camera_angle_error_deg", 0) for n in nodes), default=0
        )
        summary["max_spawn_drift_m"] = max(
            (t["spawn_drift_m"] for t in metadata["targets"]), default=0
        )
        report(f"Finished: {summary['status']}; cleanup failures: {len(cleanup)}")
        finish_artifacts(root, nodes, summary)
    if cleanup:
        raise RuntimeError("cleanup incomplete; inspect summary.json")
    return summary
