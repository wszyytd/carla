"""CARLA-only boundary, injected for offline tests. No module-level CARLA import."""

import math
import os
import random
import time
from contextlib import contextmanager
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from src.scout_quality import FrameInbox, StabilityGate

from ..aod.capture import _weather, connect, geometry, transform
from ..aod.preview import pose_values, poses_close
from ..runtime import OwnedActors, apply_camera_capture_settings
from ..sensors import configure_required_attributes, listen_to_inbox
from .artifacts import finalize, initialize, write_frame, write_json, write_jsonl
from .config import config_hash, parse_config
from .geometry import GEOMETRY_SCOPE, point_reason, validate_geometry
from .validate import SensorAlignmentError, validate_bundle


@contextmanager
def output_lock(root):
    """OS advisory lock releases on process death; persistent sidecar avoids unlink races."""
    root = Path(root).resolve()
    root.parent.mkdir(parents=True, exist_ok=True)
    path = root.parent / ("." + root.name + ".capture.lock")
    with path.open("a+b") as stream:
        stream.seek(0, os.SEEK_END)
        if stream.tell() == 0:
            stream.write(b"0")
            stream.flush()
        stream.seek(0)
        if os.name == "nt":
            import msvcrt

            try:
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError as error:
                raise ValueError("another capture holds output lock") from error
        else:
            import fcntl

            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                raise ValueError("another capture holds output lock") from error
        try:
            yield
        finally:
            stream.seek(0)
            if os.name == "nt":
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def _assert_empty_world(world):
    actors = world.get_actors()
    if any(
        actors.filter(pattern)
        for pattern in ("vehicle.*", "walker.*", "controller.*", "static.prop.*")
    ):
        raise RuntimeError("world occupied by external vehicles/walkers/controllers/props")


def camera_attributes(cfg, name):
    camera = cfg["camera"]
    attributes = {
        "image_size_x": camera["width"],
        "image_size_y": camera["height"],
        "fov": camera["fov_deg"],
        "sensor_tick": 0,
        "lens_k": 0,
        "lens_kcube": 0,
        "lens_circle_multiplier": 0,
    }
    if name == "rgb":
        attributes.update(camera["exposure"])
    return attributes


def weather_profile(name):
    """Versioned project profiles; never trust a native preset name as readback proof."""
    return {
        "cloudiness": 60.0 if name == "CloudyNoon" else 5.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 0.0,
        "sun_azimuth_angle": 0.0,
        "sun_altitude_angle": 15.0 if name == "ClearSunset" else 75.0,
        "fog_density": 0.0,
        "fog_distance": 1000.0,
        "fog_falloff": 0.1,
        "wetness": 0.0,
        "scattering_intensity": 1.0,
        "mie_scattering_scale": 0.03,
        "rayleigh_scattering_scale": 0.0331,
        "dust_storm": 0.0,
    }


def environment_preflight(world, cfg):
    """Read-only capability check; null means the client lacks the required 0.10 API."""
    query = getattr(world, "is_weather_enabled", None)
    enabled = bool(query()) if callable(query) else None
    map_name = str(world.get_map().name)
    errors = []
    if map_name.split("/")[-1] != cfg["scene"]["map"].split("/")[-1]:
        errors.append(f"map mismatch: {map_name}")
    if enabled is False:
        errors.append(
            "weather unavailable: is_weather_enabled()=False; this map has no CARLA weather "
            "actor. Check server log for 'Missing weather class!' / 'weather is disabled'; "
            "repair map/GameMode weather assets before capture (see server-operations.md)."
        )
    elif enabled is None:
        errors.append(
            "weather capability unknown: is_weather_enabled API missing; install the "
            "Python wheel shipped with the running CARLA 0.10.0 server."
        )
    try:
        _assert_empty_world(world)
    except RuntimeError as error:
        errors.append(str(error))
    return {
        "read_only": True,
        "map": map_name,
        "weather_enabled": enabled,
        "actual_weather": _weather(world),
        "passed": not errors,
        "errors": errors,
    }


def doctor(carla, cfg):
    """Inspect the running server without setting weather, ticking or spawning actors."""
    cfg = parse_config(cfg)
    client, versions = connect(carla, SimpleNamespace(**cfg["client"]))
    result = environment_preflight(client.get_world(), cfg)
    result.update(
        carla_client_version=versions[0],
        carla_server_version=versions[1],
        python_api_path=getattr(carla, "__file__", None),
        scope="capabilities only; does not verify weather application or RGB quality",
        exit_code=0 if result["passed"] else 3,
    )
    return result


def apply_weather_verified(world, requested, cfg, diagnostics):
    """Bound the asynchronous setter/readback gap; never accept a mismatched value."""
    value = world.get_weather()
    for key, wanted in requested.items():
        setattr(value, key, wanted)
    diagnostics.update(requested=requested, ticks=0, matched=False)
    world.set_weather(value)
    deadline = time.monotonic() + cfg["client"]["timeout_seconds"]
    for tick in range(min(20, cfg["capture"]["max_frame_ticks"]) + 1):
        actual = _weather(world)
        diagnostics.update(actual=actual, ticks=tick)
        if all(
            key in actual and math.isclose(actual[key], wanted, abs_tol=1e-4, rel_tol=0)
            for key, wanted in requested.items()
        ):
            diagnostics["matched"] = True
            return actual
        remaining = deadline - time.monotonic()
        if remaining <= 0 or tick == min(20, cfg["capture"]["max_frame_ticks"]):
            break
        _assert_empty_world(world)
        world.tick(remaining)
    raise RuntimeError(
        f"weather readback mismatch after {diagnostics['ticks']} ticks: "
        f"requested={requested}, actual={diagnostics['actual']}; "
        "weather actor exists but did not retain requested values; inspect server log "
        "and stop other weather controllers"
    )


def spawn_sensors(world, carla, cfg, actors, inbox, pose):
    channels = {"rgb": "sensor.camera.rgb", "depth": "sensor.camera.depth"}
    if cfg["camera"]["instance"]:
        channels["instance"] = "sensor.camera.instance_segmentation"
    sensors, effective = {}, {}
    library = world.get_blueprint_library()
    for name, blueprint_id in channels.items():
        blueprint = library.find(blueprint_id)
        attributes = camera_attributes(cfg, name)
        configure_required_attributes(blueprint, attributes)
        sensor = actors.add(world.spawn_actor(blueprint, transform(carla, pose)))
        if sensor is None:
            raise RuntimeError(f"could not spawn {name}")
        sensors[name] = sensor
        effective[name] = {key: str(sensor.attributes[key]) for key in attributes}
        for key, wanted in attributes.items():
            actual = effective[name][key]
            equal = (
                actual == str(wanted)
                if isinstance(wanted, str)
                else abs(float(actual) - wanted) <= 1e-4
            )
            if not equal:
                raise RuntimeError(f"actual {name}.{key} differs from configured value")
        listen_to_inbox(sensor, inbox, name)
    return sensors, effective


def await_bundle(world, inbox, desired, node, cfg, min_frame):
    deadline = time.monotonic() + cfg["capture"]["timeout_seconds"]
    gate = StabilityGate(cfg["capture"]["warmup_seconds"], cfg["quality"]["stability_threshold"])
    quality = cfg["quality"]
    rejected = 0
    last_alignment_error = None
    for _ in range(cfg["capture"]["max_frame_ticks"]):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        _assert_empty_world(world)
        world.tick(min(remaining, cfg["client"]["timeout_seconds"]))
        # Callbacks may arrive just after tick; bounded waits avoid outrunning GPU queues.
        time.sleep(min(0.01, max(0, deadline - time.monotonic())))
        ready = inbox.ready(
            min_frame,
            lambda pose: poses_close(
                pose,
                desired,
                position_m=quality["position_error_m"],
                angle_deg=quality["angle_error_deg"],
            ),
        )
        for bundle in ready:
            try:
                validate_bundle(bundle, node, cfg)
            except SensorAlignmentError as failure:
                rejected += 1
                last_alignment_error = {
                    "reason": str(failure),
                    "frames": {key: int(im.frame) for key, im in bundle.items()},
                    "timestamps": {key: repr(im.timestamp) for key, im in bundle.items()},
                }
                # A valid frame after this gap must establish stability again.
                gate = StabilityGate(
                    cfg["capture"]["warmup_seconds"], quality["stability_threshold"]
                )
                continue
            if gate.add(bundle["rgb"]):
                return bundle, {
                    **gate.diagnostics,
                    "rejected_alignment_bundles": rejected,
                    "last_alignment_error": last_alignment_error,
                }
    raise TimeoutError(
        f"no stable same-frame RGB-D at requested pose; stability={gate.diagnostics}; "
        f"rejected_alignment_bundles={rejected}; last_alignment_error={last_alignment_error}"
    )


def capture(carla, cfg, output, *, resume=False, report=print):
    cfg = parse_config(cfg)
    with output_lock(output):
        return _capture(carla, cfg, Path(output), resume, report)


def _capture(carla, cfg, root, resume, report):
    scene, nodes, edges = initialize(root, cfg, resume=resume)
    actors = OwnedActors()
    cleanup, lights = [], []
    world, settings, weather = None, None, None
    error, interrupted = None, False
    python_state, numpy_state = random.getstate(), np.random.get_state()

    def log(message):
        with (root / "capture.log").open("a", encoding="utf-8") as stream:
            stream.write(f"{datetime.now(timezone.utc).isoformat()} {message}\n")
        try:
            report(message)
        except Exception:
            # A failed progress UI must not suppress cleanup or the on-disk result.
            pass

    try:
        client, versions = connect(carla, SimpleNamespace(**cfg["client"]))
        world = client.get_world()
        scene.update(carla_client_version=versions[0], carla_server_version=versions[1])
        scene["preflight"] = environment_preflight(world, cfg)
        write_json(root / "scene.json", scene)
        if not scene["preflight"]["passed"]:
            raise RuntimeError("; ".join(scene["preflight"]["errors"]))
        map_name = scene["preflight"]["map"]
        random.seed(cfg["scene"]["seed"])
        np.random.seed(cfg["scene"]["seed"])
        settings, weather = world.get_settings(), world.get_weather()
        apply_camera_capture_settings(world, cfg["scene"]["fixed_delta_seconds"])
        requested_weather = weather_profile(cfg["scene"]["weather"])
        scene["weather_request"] = requested_weather
        scene["weather_application"] = {}
        scene["actual_weather"] = apply_weather_verified(
            world, requested_weather, cfg, scene["weather_application"]
        )
        lights = [
            (light, light.is_frozen(), light.get_state())
            for light in world.get_actors().filter("traffic.traffic_light*")
        ]
        for light, _, _ in lights:
            light.freeze(True)
            light.set_state(carla.TrafficLightState.Red)
        boxes, counts = geometry(carla, world)
        # Sort before hashing: native query order is not a scene property.
        boxes = sorted(boxes)
        channels = ("rgb", "depth", "instance") if cfg["camera"]["instance"] else ("rgb", "depth")
        expected_attributes = {
            name: {k: str(v) for k, v in camera_attributes(cfg, name).items()} for name in channels
        }
        environment = {
            "versions": versions,
            "weather_profile_version": 2,
            "map": map_name,
            "weather": _weather(world),
            "geometry_sha256": config_hash(boxes),
            "geometry_counts": counts,
            "camera_attributes": expected_attributes,
            "traffic_lights": "frozen_red",
            "dynamic_actors": 0,
        }
        if scene["environment"] is not None and scene["environment"] != environment:
            raise RuntimeError("environment mismatch on resume; use a new scene directory")
        scene["environment"] = environment
        scene.update(
            carla_client_version=versions[0],
            carla_server_version=versions[1],
            actual_weather=environment["weather"],
        )
        recovered = {n["node_id"]: n for n in nodes if n["status"] == "captured"}
        validate_geometry(nodes, edges, cfg, boxes)
        for i, node in enumerate(nodes):
            if node["node_id"] in recovered:
                if not node["valid"]:
                    raise RuntimeError("geometry mismatch for previously captured node")
                nodes[i] = recovered[node["node_id"]]
                nodes[i]["status"] = "captured"
        write_json(
            root / "region_geometry.json",
            {
                "aabbs": boxes,
                "counts": counts,
                "scope": GEOMETRY_SCOPE,
                "clearance_m": cfg["graph"]["clearance_m"],
                "edge_method": "exact closed-segment slab intersection; no sampling gaps",
                "configured_edge_step_m": cfg["graph"]["edge_step_m"],
                "manual_review": "pending",
            },
        )
        scene.update(status="incomplete", error=None, cleanup_failures=[])
        write_json(root / "scene.json", scene)
        write_jsonl(root / "nodes.jsonl", nodes)
        write_jsonl(root / "edges.jsonl", edges)
        pending = [n for n in nodes if n["valid"] and n["status"] != "captured"]
        inbox = FrameInbox(channels)
        if pending:
            sensors, effective = spawn_sensors(
                world, carla, cfg, actors, inbox, pending[0]["requested_transform"]
            )
            # Store actual native formatting independently of the canonical resume signature.
            scene["actual_sensor_attributes"] = effective
            write_json(root / "scene.json", scene)
        for i, node in enumerate(nodes):
            if not node["valid"] or node["status"] == "captured":
                continue
            log(f"[{i + 1}/{len(nodes)}] {node['node_id']}")
            try:
                _assert_empty_world(world)
                if _weather(world) != environment["weather"]:
                    raise RuntimeError("weather changed during capture")
                desired = transform(carla, node["requested_transform"])
                for sensor in sensors.values():
                    sensor.set_transform(desired)
                inbox.clear()
                min_frame = int(world.get_snapshot().frame) + 1
                bundle, diagnostics = await_bundle(world, inbox, desired, node, cfg, min_frame)
                for image in bundle.values():
                    reason = point_reason(pose_values(image.transform)[:3], cfg, boxes)
                    if reason:
                        raise ValueError(f"actual sensor pose: {reason}")
                nodes[i] = write_frame(root, node, bundle, cfg, diagnostics)
                write_jsonl(root / "nodes.jsonl", nodes)
            except (TimeoutError, ValueError) as failure:
                node.update(
                    valid=False,
                    status="failed",
                    invalid_reason="capture_failed",
                    diagnostic={"error": f"{type(failure).__name__}: {failure}"},
                )
                log(f"{node['node_id']}: capture_failed: {failure}")
                write_jsonl(root / "nodes.jsonl", nodes)
            except BaseException as failure:
                node.update(
                    valid=False,
                    status="failed",
                    invalid_reason="capture_failed",
                    diagnostic={"error": f"{type(failure).__name__}: {failure}"},
                )
                raise
    except BaseException as failure:
        interrupted = isinstance(failure, KeyboardInterrupt)
        error = f"{type(failure).__name__}: {failure}"
        log(error)
    finally:
        cleanup.extend(asdict(f) for f in actors.destroy_all())
        restore = [("traffic_light", lambda item=item: _restore_light(*item)) for item in lights]
        if world is not None and weather is not None:
            restore.append(("weather", lambda: world.set_weather(weather)))
        if world is not None and settings is not None:
            restore.append(("world_settings", lambda: world.apply_settings(settings)))
        for name, operation in restore:
            try:
                operation()
            except Exception as failure:
                cleanup.append({"actor_id": name, "operation": "restore", "message": str(failure)})
        random.setstate(python_state)
        np.random.set_state(numpy_state)
        log(f"cleanup failures={len(cleanup)}; finalizing")
        result = finalize(root, cfg, scene, nodes, edges, cleanup, error)
    result["interrupted"] = interrupted
    result["exit_code"] = (
        130 if interrupted else (3 if error or cleanup else (0 if result["passed"] else 5))
    )
    return result


def _restore_light(light, frozen, state):
    try:
        light.set_state(state)
    finally:
        light.freeze(frozen)
