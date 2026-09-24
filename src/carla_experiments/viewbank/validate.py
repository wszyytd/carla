"""Offline structural, geometric and sensor-quality checks; no CARLA import."""

import math
from pathlib import Path

import numpy as np
from PIL import Image

from ..aod.preview import pose_values
from ..metrics import build_projection_matrix
from .artifacts import DEPTH_SEMANTICS, digest, read_frame, read_json, read_jsonl, relative_file
from .config import config_hash, parse_config
from .geometry import graph_report, local_pose, point_reason, pose_matrix, validate_geometry
from .grid import build_grid, start_node_id


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def pose_errors(first, second):
    _require(
        len(first) == len(second) == 6 and all(math.isfinite(v) for v in (*first, *second)),
        "nonfinite/invalid pose",
    )
    return math.dist(first[:3], second[:3]), max(
        abs((a - b + 180) % 360 - 180) for a, b in zip(first[3:], second[3:], strict=True)
    )


def _check_pose(actual, requested, cfg):
    position, angle = pose_errors(actual, requested)
    _require(position <= cfg["quality"]["position_error_m"], "position error exceeds threshold")
    _require(angle <= cfg["quality"]["angle_error_deg"], "angle error exceeds threshold")
    return position, angle


def check_camera_model(node, cfg):
    camera = cfg["camera"]
    _require(node["image_size"] == [camera["width"], camera["height"]], "camera size mismatch")
    _require(
        math.isfinite(node["fov_deg"]) and abs(node["fov_deg"] - camera["fov_deg"]) <= 1e-4,
        "actual FOV differs from configuration",
    )
    expected = build_projection_matrix(
        width=camera["width"], height=camera["height"], fov_deg=node["fov_deg"]
    )
    matrix = np.asarray(node["intrinsics"])
    _require(
        matrix.shape == (3, 3) and np.allclose(matrix, expected, rtol=0, atol=1e-10),
        "intrinsics do not match actual camera",
    )


def validate_bundle(bundle, node, cfg):
    names = {"rgb", "depth"} | ({"instance"} if cfg["camera"]["instance"] else set())
    _require(set(bundle) == names, "sensor channels mismatch")
    _require(len({im.frame for im in bundle.values()}) == 1, "sensor frame mismatch")
    times = [im.timestamp for im in bundle.values()]
    _require(all(math.isfinite(t) and t >= 0 for t in times), "nonfinite sensor timestamp")
    _require(
        max(times) - min(times) <= cfg["quality"]["timestamp_error_seconds"],
        "sensor timestamp mismatch",
    )
    for image in bundle.values():
        _require(
            math.isfinite(image.fov) and abs(image.fov - node["fov_deg"]) <= 1e-4,
            "actual sensor FOV mismatch",
        )
        _require([image.width, image.height] == node["image_size"], "sensor dimensions mismatch")
        _require(type(image.frame) is int and image.frame >= 0, "invalid frame ID")
        _check_pose(pose_values(image.transform), node["requested_transform"], cfg)
        _check_pose(pose_values(image.transform), pose_values(bundle["rgb"].transform), cfg)


def inspect_camera_files(folder, node, cfg):
    camera = read_json(folder / "camera.json")
    check_camera_model(node, cfg)
    width, height = node["image_size"]
    _require(
        camera["schema_version"] == 1 and camera["node_id"] == node["node_id"],
        "camera schema/ID mismatch",
    )
    _require(
        node["status"] == "captured" and node["valid"] and node["invalid_reason"] is None,
        "frame node status mismatch",
    )
    for key in (
        "requested_transform",
        "actual_transform",
        "frame",
        "timestamp",
        "intrinsics",
        "image_size",
        "fov_deg",
        "local_pose",
    ):
        _require(camera[key] == node[key], f"camera/node {key} mismatch")
    _require(camera["depth_semantics"] == DEPTH_SEMANTICS, "depth semantics mismatch")
    errors = _check_pose(camera["actual_transform"], node["requested_transform"], cfg)
    _require(
        camera["local_pose"] == local_pose(camera["actual_transform"], cfg["grid"]["origin_m"]),
        "local pose mismatch",
    )
    matrix = np.asarray(camera["sensor_to_world"])
    _require(
        matrix.shape == (4, 4)
        and np.allclose(matrix, pose_matrix(camera["actual_transform"]), atol=1e-8, rtol=0),
        "extrinsic matrix mismatch",
    )
    _require(
        np.allclose(matrix @ np.asarray(camera["world_to_sensor"]), np.eye(4), atol=1e-8, rtol=0),
        "inverse extrinsic mismatch",
    )
    names = {"rgb", "depth"} | ({"instance"} if cfg["camera"]["instance"] else set())
    _require(set(camera["sensors"]) == names, "camera sensor records incomplete")
    _require(
        type(camera["frame"]) is int
        and camera["frame"] >= 0
        and math.isfinite(camera["timestamp"])
        and camera["timestamp"] >= 0,
        "invalid frame/timestamp",
    )
    timestamps = [v["timestamp"] for v in camera["sensors"].values()]
    _require(
        max(timestamps) - min(timestamps) <= cfg["quality"]["timestamp_error_seconds"],
        "sensor timestamp span mismatch",
    )
    for sensor in camera["sensors"].values():
        _require(
            math.isfinite(sensor["fov_deg"]) and abs(sensor["fov_deg"] - node["fov_deg"]) <= 1e-4,
            "actual sensor FOV mismatch",
        )
        _require(sensor["frame"] == camera["frame"], "sensor frame mismatch")
        _require(
            math.isfinite(sensor["timestamp"])
            and abs(sensor["timestamp"] - camera["timestamp"])
            <= cfg["quality"]["timestamp_error_seconds"],
            "sensor timestamp mismatch",
        )
        _require(sensor["image_size"] == [width, height], "sensor dimensions mismatch")
        _check_pose(sensor["transform"], camera["requested_transform"], cfg)
        _check_pose(sensor["transform"], camera["actual_transform"], cfg)
    for name in ("rgb.png", "mask.png", *(["instance.png"] if cfg["camera"]["instance"] else [])):
        with Image.open(folder / name) as image:
            image.load()
            _require(
                image.format == "PNG" and image.size == (width, height),
                f"{name} dimensions/format mismatch",
            )
            _require(image.mode == ("L" if name == "mask.png" else "RGB"), f"{name} mode mismatch")
    depth = np.load(folder / "depth.npy", allow_pickle=False)
    _require(depth.dtype == np.dtype("float32"), "depth dtype must be float32")
    _require(depth.shape == (height, width), "depth dimensions mismatch")
    finite = np.isfinite(depth)
    q = cfg["quality"]
    _require(
        float(finite.mean()) >= q["min_finite_depth_fraction"],
        "finite depth fraction below threshold",
    )
    _require(np.all((depth[finite] >= 0) & (depth[finite] <= 1000)), "depth outside encoding range")
    expected_mask = finite & (depth >= q["depth_min_m"]) & (depth < q["depth_max_m"])
    with Image.open(folder / "mask.png") as image:
        mask = np.asarray(image)
    _require(np.array_equal(mask, expected_mask.astype(np.uint8) * 255), "depth mask mismatch")
    _require(
        float(expected_mask.mean()) >= q["min_valid_depth_fraction"],
        "valid depth fraction below threshold",
    )
    return {
        "position_error_m": errors[0],
        "angle_error_deg": errors[1],
        "finite_depth_fraction": float(finite.mean()),
        "valid_depth_fraction": float(expected_mask.mean()),
    }


def inspect_dataset(root, require_complete=True):
    root = Path(root)
    report = {
        "schema_version": 1,
        "passed": False,
        "errors": [],
        "planned": 0,
        "captured": 0,
        "rejected": 0,
        "graph": {},
        "node_quality": {},
    }
    errors = report["errors"]
    try:
        scene = read_json(root / "scene.json")
        _require(scene["schema_version"] == 1, "unsupported schema_version")
        cfg = parse_config(read_json(root / "config.resolved.json"))
        report["planned"] = len(build_grid(cfg)[0])
        _require(scene["config_sha256"] == config_hash(cfg), "config checksum mismatch")
        _require(
            scene["scene_id"] == cfg["scene_id"] and scene["camera_model"] == cfg["camera"],
            "scene identity/camera mismatch",
        )
        _require(scene["depth_semantics"] == DEPTH_SEMANTICS, "scene depth semantics mismatch")
        _require(bool(scene["environment"]), "missing CARLA environment provenance")
        nodes, edges = read_jsonl(root / "nodes.jsonl"), read_jsonl(root / "edges.jsonl")
        expected_nodes, expected_edges = build_grid(cfg)
        geometry = read_json(root / "region_geometry.json")
        _require(
            scene["environment"]["geometry_sha256"] == config_hash(sorted(geometry["aabbs"])),
            "environment geometry checksum mismatch",
        )
        validate_geometry(expected_nodes, expected_edges, cfg, geometry["aabbs"])
        report["planned"] = len(expected_nodes)
        _require(
            [n["node_id"] for n in nodes] == [n["node_id"] for n in expected_nodes],
            "node IDs/count/order mismatch",
        )
        _require(
            [e["edge_id"] for e in edges] == [e["edge_id"] for e in expected_edges],
            "edge IDs/count/order mismatch",
        )
        lookup = {n["node_id"]: n for n in nodes}
        for node, expected in zip(nodes, expected_nodes, strict=True):
            try:
                for field in (
                    "grid_index",
                    "requested_transform",
                    "frame_dir",
                    "image_size",
                ):
                    _require(node[field] == expected[field], f"node {field} mismatch")
                check_camera_model(node, cfg)
                if not expected["valid"]:
                    _require(
                        not node["valid"]
                        and node["invalid_reason"] == expected["invalid_reason"]
                        and node["status"] == "rejected",
                        "geometry rejection mismatch",
                    )
                    _require(
                        not relative_file(root, node["frame_dir"]).exists(),
                        "rejected node has frame files",
                    )
                    report["rejected"] += 1
                else:
                    _require(
                        node["status"] == "captured",
                        f"{node['invalid_reason'] or 'pending'}: node not captured",
                    )
                    _require(
                        point_reason(node["actual_transform"][:3], cfg, geometry["aabbs"]) is None,
                        "actual sensor center outside ROI or inside obstacle",
                    )
                    recovered = read_frame(root, expected, cfg)
                    _require(recovered == node, "node differs from frame receipt")
                    base = node["frame_dir"]
                    _require(
                        node["observation"]
                        == {
                            k: f"{base}/{v}"
                            for k, v in (
                                ("rgb", "rgb.png"),
                                ("depth", "depth.npy"),
                                ("mask", "mask.png"),
                                ("camera", "camera.json"),
                            )
                        },
                        "observation references mismatch",
                    )
                    _require(
                        node["diagnostic"]
                        == (
                            {"instance": f"{base}/instance.png"}
                            if cfg["camera"]["instance"]
                            else {}
                        ),
                        "diagnostic references mismatch",
                    )
                    for group in ("observation", "diagnostic"):
                        for value in node[group].values():
                            _require(
                                relative_file(root, value).is_file(), "missing referenced file"
                            )
                    report["node_quality"][node["node_id"]] = inspect_camera_files(
                        root / base, node, cfg
                    )
                    report["captured"] += 1
            except (ValueError, OSError, KeyError, TypeError) as error:
                errors.append(f"{node['node_id']}: {error}")
        for edge, expected in zip(edges, expected_edges, strict=True):
            for field in ("source", "target", "action", "translation_m", "rotation_deg", "cost"):
                _require(edge[field] == expected[field], f"edge {edge['edge_id']} {field} mismatch")
            reason = expected["invalid_reason"]
            if reason is None and any(
                lookup[edge[k]]["status"] != "captured" for k in ("source", "target")
            ):
                reason = "capture_failed"
            _require(
                edge["valid"] == (reason is None) and edge["invalid_reason"] == reason,
                "edge validity mismatch",
            )
        start = start_node_id(cfg, expected_nodes)
        _require(scene["start_node"] == start, "start node mismatch")
        graph = graph_report(nodes, edges, start)
        report["graph"] = graph
        if not graph["start_valid"] or not graph["start_nonisolated"]:
            errors.append("start node invalid or isolated")
        if cfg["quality"]["require_connected"] and not graph["connected"]:
            errors.append("valid graph disconnected")
        if not report["captured"]:
            errors.append("no captured nodes")
        expected_dirs = {n["node_id"] for n in nodes if n["status"] == "captured"}
        if {p.name for p in (root / "frames").iterdir()} != expected_dirs:
            errors.append("partial/unreferenced frame files or directories")
        if list(root.glob("*.tmp")):
            errors.append("partial manifest files")
        if scene.get("cleanup_failures") or scene.get("error"):
            errors.append("capture/cleanup error recorded")
        if require_complete and scene.get("status") != "complete":
            errors.append("incomplete scene")
    except (ValueError, OSError, KeyError, TypeError, IndexError) as error:
        errors.append(str(error))
    report["passed"] = not errors
    return report


def check_dataset(root):
    root = Path(root)
    report = inspect_dataset(root)
    try:
        checksums = read_json(root / "checksums.json")
        files = {
            p.relative_to(root).as_posix()
            for p in root.rglob("*")
            if p.is_file() and p != root / "checksums.json"
        }
        _require(
            set(checksums) == files, "checksum inventory mismatch (missing/partial/untracked file)"
        )
        for name, expected in checksums.items():
            _require(digest(relative_file(root, name)) == expected, f"checksum mismatch: {name}")
        quality = read_json(root / "quality.json")
        _require(
            quality["passed"] is True and quality["status"] == "complete",
            "quality incomplete/failed",
        )
        for key in ("planned", "captured", "rejected", "graph", "node_quality"):
            _require(quality[key] == report[key], f"quality {key} inconsistent")
    except (ValueError, OSError, KeyError, TypeError) as error:
        report["errors"].append(str(error))
    report["passed"] = not report["errors"]
    return report
