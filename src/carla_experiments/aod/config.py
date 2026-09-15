"""Validated preview configuration generated from a server inventory."""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from ..config import CameraConfig, ClientConfig, parse_client_config


def _number(value: Any, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
    return result


def _integer(value: Any, name: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def _vector(value: Any, name: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must have {length} entries")
    return tuple(_number(v, name, -1e7, 1e7) for v in value)


@dataclass(frozen=True)
class PreviewConfig:
    client: ClientConfig
    map_name: str
    camera: CameraConfig
    target_pose: tuple[float, ...]
    center: tuple[float, ...]
    ground_z: float
    blueprint_ids: tuple[str, ...]
    fixed_delta_seconds: float
    warmup_ticks: int
    settle_ticks: int
    max_frame_ticks: int
    timeout_seconds: float
    clearance_m: float
    resolved: dict[str, Any]


def parse_preview_config(raw: Mapping[str, Any]) -> PreviewConfig:
    if not isinstance(raw, Mapping) or raw.get("schema_version") != 1:
        raise ValueError("expected preview schema_version=1")
    try:
        world, camera, scene, capture = (
            raw[key] for key in ("world", "camera", "scene", "capture")
        )
        if not all(isinstance(x, Mapping) for x in (world, camera, scene, capture)):
            raise ValueError("configuration sections must be mappings")
        client = parse_client_config(raw)
        map_name = world["map"]
        if not isinstance(map_name, str) or not map_name.strip():
            raise ValueError("world.map must be a nonempty name")
        dt = _number(world["fixed_delta_seconds"], "fixed step", 0.001, 0.1)
        period = _number(camera["sensor_tick_seconds"], "sensor period", dt, 10)
        if not math.isclose(period / dt, round(period / dt), abs_tol=1e-8):
            raise ValueError("sensor period must be an integer multiple of fixed step")
        camera_config = CameraConfig(
            _integer(camera["width"], "width", 16, 8192),
            _integer(camera["height"], "height", 16, 8192),
            _number(camera["fov_deg"], "FOV", 1, 179),
            period,
        )
        ids = scene["blueprint_ids"]
        if (
            not isinstance(ids, list)
            or not 1 <= len(ids) <= 10
            or not all(isinstance(v, str) and v.startswith("vehicle.") for v in ids)
            or len(ids) != len(set(ids))
        ):
            raise ValueError("scene.blueprint_ids must be unique vehicle blueprint IDs")
        return PreviewConfig(
            client,
            map_name,
            camera_config,
            _vector(scene["target_pose"], "target pose", 6),
            _vector(scene["observation_center"], "observation center", 3),
            _number(scene["ground_z"], "ground reference", -1e5, 1e5),
            tuple(ids),
            dt,
            _integer(capture["warmup_ticks"], "warmup ticks", 1, 1000),
            _integer(capture["settle_ticks"], "settle ticks", 1, 1000),
            _integer(capture["max_frame_ticks"], "maximum frame ticks", 1, 10000),
            _number(capture["timeout_seconds"], "capture timeout", 0.01, 600),
            _number(capture["clearance_m"], "clearance", 0, 100),
            copy.deepcopy(dict(raw)),
        )
    except (KeyError, TypeError) as error:
        raise ValueError(f"missing or malformed preview field: {error}") from error


def prepare_config(
    inventory: Mapping[str, Any],
    *,
    spawn_index: int,
    blueprint_ids: Sequence[str] | None = None,
    ground_z: float | None = None,
) -> dict[str, Any]:
    if inventory.get("schema_version") != 1:
        raise ValueError("unsupported inventory schema")
    try:
        available = sorted(v["id"] for v in inventory["vehicles"] if v["number_of_wheels"] == 4)
        selected = list(blueprint_ids) if blueprint_ids is not None else available[:3]
        if not selected or any(v not in available for v in selected):
            raise ValueError("selected blueprint is not in four-wheel inventory")
        spawn = next((p for p in inventory["spawn_points"] if p["index"] == spawn_index), None)
        if spawn is None:
            raise ValueError(f"spawn index {spawn_index} is not in inventory")
        pose = list(_vector(spawn["pose"], "spawn pose", 6))
        ground = pose[2] if ground_z is None else ground_z
        result = {
            "schema_version": 1,
            "client": copy.deepcopy(inventory["client"]),
            "world": {"map": inventory["map"], "fixed_delta_seconds": 0.05},
            "camera": {"width": 1920, "height": 1080, "fov_deg": 60.0, "sensor_tick_seconds": 0.1},
            "scene": {
                "spawn_index": spawn_index,
                "target_pose": pose,
                "ground_z": ground,
                "ground_reference": "spawn_point_unverified" if ground_z is None else "explicit",
                "observation_center": [pose[0], pose[1], ground + 1.0],
                "blueprint_ids": selected,
            },
            "capture": {
                "warmup_ticks": 6,
                "settle_ticks": 40,
                "max_frame_ticks": 120,
                "timeout_seconds": 30.0,
                "clearance_m": 1.0,
            },
        }
        parse_preview_config(result)
        return result
    except (KeyError, TypeError) as error:
        raise ValueError(f"malformed inventory: {error}") from error
