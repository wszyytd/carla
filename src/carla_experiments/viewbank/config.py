"""Strict, normalized JSON-compatible configuration (no simulator dependency)."""

import copy
import hashlib
import json
import math
import re
from pathlib import Path

import yaml

FIELDS = {
    "root": "schema_version scene_id client scene grid camera graph capture quality",
    "client": "host port timeout_seconds",
    "scene": "map seed fixed_delta_seconds weather roi_min_m roi_max_m",
    "grid": "origin_m x_offsets_m y_offsets_m z_world_m yaw_deg pitch_deg",
    "camera": "width height fov_deg instance exposure",
    "exposure": "exposure_mode iso shutter_speed fstop exposure_compensation motion_blur_intensity",
    "graph": "clearance_m edge_step_m start_index translation_cost_per_m rotation_cost_per_deg",
    "capture": "warmup_seconds timeout_seconds max_frame_ticks",
    "quality": (
        "position_error_m angle_error_deg timestamp_error_seconds stability_threshold "
        "min_finite_depth_fraction min_valid_depth_fraction depth_min_m depth_max_m "
        "require_connected"
    ),
}


def _fields(value, section):
    expected = set(FIELDS[section].split())
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError(f"{section}: requires exactly {sorted(expected)}")


def _number(value, name, low, high, integer=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or not low <= value <= high
        or (integer and not isinstance(value, int))
    ):
        raise ValueError(
            f"{name}: expected {'integer' if integer else 'number'} in [{low}, {high}]"
        )
    return int(value) if integer else float(value)


def parse_config(raw):
    cfg = copy.deepcopy(raw)
    _fields(cfg, "root")
    for key in FIELDS["root"].split()[2:]:
        _fields(cfg[key], key)
    _fields(cfg["camera"]["exposure"], "exposure")
    if type(cfg["schema_version"]) is not int or cfg["schema_version"] != 1:
        raise ValueError("unsupported schema_version")
    if not isinstance(cfg["scene_id"], str) or not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_-]{0,95}", cfg["scene_id"]
    ):
        raise ValueError("scene_id must be a portable identifier")
    for section, key in (("client", "host"), ("scene", "map")):
        if not isinstance(cfg[section][key], str) or not cfg[section][key].strip():
            raise ValueError(f"{section}.{key}: nonempty string required")
    if cfg["scene"]["weather"] not in ("ClearNoon", "ClearSunset", "CloudyNoon"):
        raise ValueError("weather must be ClearNoon, ClearSunset or CloudyNoon")
    for section, key, low, high, integer in (
        ("client", "port", 1, 65535, True),
        ("client", "timeout_seconds", 0.01, 3600, False),
        ("scene", "seed", 0, 2**32 - 1, True),
        ("scene", "fixed_delta_seconds", 0.001, 0.1, False),
        ("camera", "width", 1, 8192, True),
        ("camera", "height", 1, 8192, True),
        ("camera", "fov_deg", 1, 179, False),
        ("grid", "pitch_deg", -90, 90, False),
        ("graph", "clearance_m", 0, 100, False),
        ("graph", "edge_step_m", 0.001, 1000, False),
        ("graph", "translation_cost_per_m", 0, 1e6, False),
        ("graph", "rotation_cost_per_deg", 0, 1e6, False),
        ("capture", "warmup_seconds", 0, 600, False),
        ("capture", "timeout_seconds", 0.01, 3600, False),
        ("capture", "max_frame_ticks", 1, 100000, True),
        ("quality", "position_error_m", 0, 10, False),
        ("quality", "angle_error_deg", 0, 180, False),
        ("quality", "timestamp_error_seconds", 0, 0.01, False),
        ("quality", "stability_threshold", 0, 255, False),
        ("quality", "min_finite_depth_fraction", 0, 1, False),
        ("quality", "min_valid_depth_fraction", 0, 1, False),
        ("quality", "depth_min_m", 0, 1000, False),
        ("quality", "depth_max_m", 0, 1000, False),
    ):
        cfg[section][key] = _number(cfg[section][key], f"{section}.{key}", low, high, integer)
    for section, key in (("camera", "instance"), ("quality", "require_connected")):
        if type(cfg[section][key]) is not bool:
            raise ValueError(f"{section}.{key}: boolean required")
    for section, key in (("grid", "origin_m"), ("scene", "roi_min_m"), ("scene", "roi_max_m")):
        seq = cfg[section][key]
        if not isinstance(seq, list) or len(seq) != 3:
            raise ValueError(f"{key}: three coordinates required")
        cfg[section][key] = [_number(v, key, -1e6, 1e6) for v in seq]
    grid = cfg["grid"]
    for key in ("x_offsets_m", "y_offsets_m", "z_world_m", "yaw_deg"):
        seq = grid[key]
        if not isinstance(seq, list) or not 1 <= len(seq) <= 1000:
            raise ValueError(f"{key}: nonempty list required (at most 1000)")
        seq = [_number(v, key, -1e6, 1e6) for v in seq]
        if key == "yaw_deg":
            seq = [v % 360 for v in seq]
        if len(set(seq)) != len(seq) or (key != "yaw_deg" and seq != sorted(seq)):
            raise ValueError(f"{key}: unique ordered coordinates required")
        grid[key] = seq
    dims = [len(grid[k]) for k in ("x_offsets_m", "y_offsets_m", "z_world_m", "yaw_deg")]
    if math.prod(dims) > 100000:
        raise ValueError("grid exceeds 100000 nodes")
    start = cfg["graph"]["start_index"]
    if not isinstance(start, list) or len(start) != 4:
        raise ValueError("start_index requires four grid indices")
    for v, size in zip(start, dims, strict=True):
        _number(v, "start_index", 0, size - 1, True)
    if any(
        a >= b for a, b in zip(cfg["scene"]["roi_min_m"], cfg["scene"]["roi_max_m"], strict=True)
    ):
        raise ValueError("ROI minimum must be below maximum")
    if cfg["quality"]["depth_min_m"] >= cfg["quality"]["depth_max_m"]:
        raise ValueError("depth_min_m must be below depth_max_m")
    exposure = cfg["camera"]["exposure"]
    if exposure["exposure_mode"] != "manual" or exposure["motion_blur_intensity"] != 0:
        raise ValueError("fixed manual exposure and zero motion blur required")
    for key in ("iso", "shutter_speed", "fstop"):
        exposure[key] = _number(exposure[key], key, 0.001, 1e6)
    exposure["exposure_compensation"] = _number(
        exposure["exposure_compensation"], "exposure_compensation", -20, 20
    )
    exposure["motion_blur_intensity"] = _number(
        exposure["motion_blur_intensity"], "motion_blur_intensity", 0, 0
    )
    return cfg


def config_hash(value):
    return hashlib.sha256(
        json.dumps(
            value, sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
    ).hexdigest()


def load_config(path):
    # Reject duplicate YAML keys instead of silently accepting a misspecified experiment.
    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node):
        pairs = loader.construct_pairs(node, deep=True)
        result = {}
        for key, value in pairs:
            if not isinstance(key, str) or key in result:
                raise ValueError(f"duplicate or non-string YAML key: {key}")
            result[key] = value
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    return parse_config(yaml.load(Path(path).read_text(encoding="utf-8"), Loader=UniqueLoader))
