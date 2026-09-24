"""Atomic manifests and immutable, independently recoverable frame transactions."""

import copy
import json
import os
import uuid
from pathlib import Path, PurePosixPath

import numpy as np
from PIL import Image

from src.scout_quality import decode_depth

from ..aod.artifacts import digest
from ..aod.artifacts import write_json as plain_write_json
from ..aod.capture import _git_revision
from ..aod.preview import decode_rgb, pose_values
from ..metrics import build_projection_matrix
from .config import config_hash, parse_config
from .geometry import GEOMETRY_SCOPE, local_pose, pose_matrix
from .grid import build_grid, start_node_id

DEPTH_SEMANTICS = {
    "file": "depth.npy",
    "dtype": "float32",
    "units": "metres",
    "quantity": "CARLA ray distance",
    "encoding_range_m": [0, 1000],
    "z_depth_conversion": "z=d/sqrt(1+((u-cx)/fx)^2+((v-cy)/fy)^2)",
    "pixel_coordinates": "u=column,v=row; integer pixel coordinates; cx=width/2,cy=height/2",
    "z_depth_file": None,
}


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def invalid(value):
        raise ValueError(f"nonfinite JSON value: {value}")

    value = json.loads(
        Path(path).read_text(encoding="utf-8"), object_pairs_hook=unique, parse_constant=invalid
    )
    if not isinstance(value, dict):
        raise ValueError(f"JSON artifact must be an object: {Path(path).name}")
    return value


def _sync_file(path):
    with Path(path).open("rb+") as stream:
        os.fsync(stream.fileno())


def _sync_directory(path):
    if os.name != "nt":
        descriptor = os.open(path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def write_json(path, value):
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    plain_write_json(temp, value)
    _sync_file(temp)
    os.replace(temp, path)
    _sync_directory(path.parent)


def write_jsonl(path, rows):
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
    _sync_directory(path.parent)


def read_jsonl(path):
    rows = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            raise ValueError("blank JSONL row")
        row = json.loads(line)
        if not isinstance(row, dict):
            raise ValueError("JSONL rows must be objects")
        rows.append(row)
    return rows


def relative_file(root, value):
    if (
        not isinstance(value, str)
        or not value
        or "\\" in value
        or ":" in value
        or PurePosixPath(value).is_absolute()
        or ".." in value.split("/")
        or str(PurePosixPath(value)) != value
    ):
        raise ValueError(f"unsafe relative path: {value!r}")
    root = Path(root).resolve()
    path = root / value
    if not path.resolve().is_relative_to(root):
        raise ValueError(f"path escapes dataset: {value}")
    # Refuse internal symlinks as well: moving the dataset must preserve identity.
    if any(p.is_symlink() for p in (path, *path.parents) if p != root and p.is_relative_to(root)):
        raise ValueError(f"symlink in dataset: {value}")
    return path


def scene_record(cfg, nodes):
    return {
        "schema_version": 1,
        "scene_id": cfg["scene_id"],
        "status": "incomplete",
        "config_sha256": config_hash(cfg),
        "config_file": "config.resolved.json",
        "git_commit": _git_revision(),
        "map": cfg["scene"]["map"],
        "weather": cfg["scene"]["weather"],
        "seed": cfg["scene"]["seed"],
        "environment": None,
        "start_node": start_node_id(cfg, nodes),
        "local_coordinates": {
            "origin_m": cfg["grid"]["origin_m"],
            "axes": "parallel to CARLA world; left handed x forward y right z up",
            "rotation": "CARLA degrees; pose array x,y,z,pitch,yaw,roll",
            "height": "grid z_world_m is absolute world z, not AGL",
        },
        "camera_model": cfg["camera"],
        "depth_semantics": DEPTH_SEMANTICS,
        "geometry_scope": GEOMETRY_SCOPE,
        "planner_visible_fields": [
            "executed_node.observation.rgb",
            "executed_node.observation.depth",
            "executed_node.observation.mask",
            "executed_node.actual_transform",
            "executed_node.intrinsics",
            "valid adjacency",
            "public edge costs",
        ],
        "planner_access_rule": (
            "only current executed node RGB-D; backend must enforce visited access"
        ),
        "evaluation_only": ["diagnostic", "instance.png", "region_geometry.json", "quality.json"],
        "excluded_planner_data": [
            "unvisited RGB-D",
            "instance labels",
            "target labels",
            "true coverage gain",
            "Oracle coverage",
        ],
        "dynamic_actor_policy": (
            "reject external vehicles/walkers/controllers/props; freeze traffic lights"
        ),
        "cleanup_failures": [],
        "error": None,
    }


def _quarantine(root, path):
    root, path = Path(root).resolve(), Path(path).resolve()
    if not path.is_relative_to(root) or path == root:
        raise ValueError("recovery path outside dataset")
    destination = root / "diagnostic" / "recovery"
    destination.mkdir(parents=True, exist_ok=True)
    os.replace(path, destination / (path.name + "-" + uuid.uuid4().hex))


def initialize(root, cfg, resume=False):
    cfg = parse_config(cfg)
    root = Path(root)
    nodes, edges = build_grid(cfg)
    if root.exists():
        if not resume:
            raise FileExistsError(f"output exists: {root}; use --resume for matching capture")
        scene = read_json(root / "scene.json")
        if (
            scene.get("schema_version") != 1
            or scene.get("config_sha256") != config_hash(cfg)
            or parse_config(read_json(root / "config.resolved.json")) != cfg
        ):
            raise ValueError("config/schema mismatch; refusing resume")
        # The deterministic plan plus validated frame receipts is the recovery authority.
        # Manifests can be stale if the process died between directory and JSONL commits.
        for path in list(root.glob("*.tmp")) + list((root / "frames").glob(".*.tmp")):
            _quarantine(root, path)
        for i, node in enumerate(nodes):
            folder = relative_file(root, node["frame_dir"])
            if folder.exists():
                try:
                    nodes[i] = read_frame(root, node, cfg)
                except (ValueError, OSError, KeyError, TypeError):
                    _quarantine(root, folder)
        return scene, nodes, edges
    root.mkdir(parents=True, exist_ok=False)
    (root / "frames").mkdir()
    scene = scene_record(cfg, nodes)
    write_json(root / "config.resolved.json", cfg)
    write_json(root / "scene.json", scene)
    write_jsonl(root / "nodes.jsonl", nodes)
    write_jsonl(root / "edges.jsonl", edges)
    (root / "capture.log").touch()
    return scene, nodes, edges


def read_frame(root, expected_node, cfg):
    from .validate import check_camera_model, inspect_camera_files

    folder = relative_file(root, expected_node["frame_dir"])
    receipt = read_json(folder / "receipt.json")
    if receipt.get("config_sha256") != config_hash(cfg):
        raise ValueError("frame config checksum mismatch")
    node = receipt["node"]
    for field in (
        "node_id",
        "grid_index",
        "frame_dir",
        "requested_transform",
        "image_size",
    ):
        if node[field] != expected_node[field]:
            raise ValueError(f"frame receipt {field} mismatch")
    check_camera_model(node, cfg)
    required = {"rgb.png", "depth.npy", "mask.png", "camera.json"}
    if cfg["camera"]["instance"]:
        required.add("instance.png")
    if set(receipt["sha256"]) != required:
        raise ValueError("frame receipt incomplete")
    for name, checksum in receipt["sha256"].items():
        if digest(relative_file(folder, name)) != checksum:
            raise ValueError(f"frame checksum mismatch: {name}")
    if {p.name for p in folder.iterdir()} != required | {"receipt.json"}:
        raise ValueError("partial or unexpected frame files")
    inspect_camera_files(folder, node, cfg)
    return node


def write_frame(root, node, bundle, cfg, diagnostics):
    from .validate import inspect_camera_files, validate_bundle

    root = Path(root)
    folder = relative_file(root, node["frame_dir"])
    if folder.exists():
        return read_frame(root, node, cfg)
    validate_bundle(bundle, node, cfg)
    temp = folder.with_name("." + folder.name + ".tmp")
    temp.mkdir(exist_ok=False)
    rgb, depth = bundle["rgb"], decode_depth(bundle["depth"])
    Image.fromarray(decode_rgb(rgb)).save(temp / "rgb.png")
    np.save(temp / "depth.npy", depth, allow_pickle=False)
    quality = cfg["quality"]
    mask = np.isfinite(depth) & (depth >= quality["depth_min_m"]) & (depth < quality["depth_max_m"])
    Image.fromarray(mask.astype(np.uint8) * 255).save(temp / "mask.png")
    if cfg["camera"]["instance"]:
        Image.fromarray(decode_rgb(bundle["instance"])).save(temp / "instance.png")
    actual = list(pose_values(rgb.transform))
    saved = copy.deepcopy(node)
    saved.update(
        status="captured",
        valid=True,
        invalid_reason=None,
        actual_transform=actual,
        fov_deg=float(rgb.fov),
        intrinsics=build_projection_matrix(
            width=rgb.width, height=rgb.height, fov_deg=float(rgb.fov)
        ).tolist(),
        local_pose=local_pose(actual, cfg["grid"]["origin_m"]),
        frame=int(rgb.frame),
        timestamp=float(rgb.timestamp),
        observation={
            key: f"{node['frame_dir']}/{name}"
            for key, name in (
                ("rgb", "rgb.png"),
                ("depth", "depth.npy"),
                ("mask", "mask.png"),
                ("camera", "camera.json"),
            )
        },
        diagnostic={"instance": f"{node['frame_dir']}/instance.png"}
        if cfg["camera"]["instance"]
        else {},
    )
    camera = {
        "schema_version": 1,
        "node_id": node["node_id"],
        "frame": saved["frame"],
        "timestamp": saved["timestamp"],
        "requested_transform": node["requested_transform"],
        "actual_transform": actual,
        "local_pose": saved["local_pose"],
        "intrinsics": saved["intrinsics"],
        "image_size": node["image_size"],
        "fov_deg": saved["fov_deg"],
        "sensor_to_world": pose_matrix(actual),
        "world_to_sensor": np.linalg.inv(pose_matrix(actual)).tolist(),
        "optical_to_sensor_axes": [[0, 0, 1], [1, 0, 0], [0, -1, 0]],
        "optical_axes": "x right, y down, z forward; no PyTorch3D conversion applied",
        "depth_semantics": DEPTH_SEMANTICS,
        "sensors": {
            name: {
                "frame": int(im.frame),
                "timestamp": float(im.timestamp),
                "transform": list(pose_values(im.transform)),
                "image_size": [im.width, im.height],
                "fov_deg": float(im.fov),
            }
            for name, im in bundle.items()
        },
        "diagnostic": {"stability": diagnostics},
    }
    write_json(temp / "camera.json", camera)
    inspect_camera_files(temp, saved, cfg)
    hashes = {p.name: digest(p) for p in sorted(temp.iterdir())}
    write_json(
        temp / "receipt.json", {"config_sha256": config_hash(cfg), "node": saved, "sha256": hashes}
    )
    for path in temp.iterdir():
        _sync_file(path)
    _sync_directory(temp)
    # Same filesystem directory rename: readers see either no node or all validated files.
    os.rename(temp, folder)
    _sync_directory(folder.parent)
    return saved


def write_checksums(root):
    root = Path(root)
    hashes = {
        p.relative_to(root).as_posix(): digest(p)
        for p in sorted(root.rglob("*"))
        if p.is_file() and p != root / "checksums.json"
    }
    write_json(root / "checksums.json", hashes)


def finalize(root, cfg, scene, nodes, edges, cleanup, error):
    from .validate import inspect_dataset

    root = Path(root)
    lookup = {n["node_id"]: n for n in nodes}
    for edge in edges:
        if edge["valid"] and any(
            lookup[edge[k]]["status"] != "captured" for k in ("source", "target")
        ):
            edge.update(valid=False, invalid_reason="capture_failed")
    scene.update(status="incomplete", cleanup_failures=cleanup, error=error)
    write_json(root / "scene.json", scene)
    write_jsonl(root / "nodes.jsonl", nodes)
    write_jsonl(root / "edges.jsonl", edges)
    report = inspect_dataset(root, require_complete=False)
    if cleanup:
        report["errors"].append("cleanup failures")
    if error:
        report["errors"].append(error)
    report["passed"] = not report["errors"]
    scene["status"] = "complete" if report["passed"] else "incomplete"
    report["status"] = scene["status"]
    report["server_visual_review"] = "pending"
    write_json(root / "scene.json", scene)
    write_json(root / "quality.json", report)
    write_json(
        root / "summary.json",
        {
            k: report[k]
            for k in ("status", "passed", "planned", "captured", "rejected", "errors", "graph")
        },
    )
    write_checksums(root)
    return report
