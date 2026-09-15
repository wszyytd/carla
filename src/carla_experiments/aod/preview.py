"""Pure geometry and image checks for stationary multiview capture."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..metrics import ProjectedBox


@dataclass(frozen=True)
class View:
    key: str
    row: int
    column: int
    x: float
    y: float
    z: float
    pitch: float
    yaw: float
    roll: float = 0.0


def preview_views(center: tuple[float, float, float], *, ground_z: float) -> tuple[View, ...]:
    offsets = ((10, 0), (-10, 0), (0, 10), (0, -10), (10, 10), (10, -10), (-10, 10), (-10, -10))
    views = []
    cx, cy, cz = center
    if not all(math.isfinite(v) for v in (*center, ground_z)):
        raise ValueError("view coordinates must be finite")
    for row, height in enumerate((20, 30, 40)):
        for column, (dx, dy) in enumerate(offsets):
            x, y, z = cx + dx, cy + dy, ground_z + height
            pitch = math.degrees(math.atan2(cz - z, math.hypot(dx, dy)))
            yaw = math.degrees(math.atan2(-dy, -dx))
            views.append(View(f"h{height}_p{column + 1}", row, column, x, y, z, pitch, yaw))
    return tuple(views)


def pose_values(transform: Any) -> tuple[float, ...]:
    location, rotation = transform.location, transform.rotation
    return tuple(
        float(v)
        for v in (location.x, location.y, location.z, rotation.pitch, rotation.yaw, rotation.roll)
    )


def poses_close(
    first: Any, second: Any, *, position_m: float = 0.1, angle_deg: float = 1.0
) -> bool:
    a, b = pose_values(first), pose_values(second)
    if not all(math.isfinite(v) for v in (*a, *b)):
        return False
    return math.dist(a[:3], b[:3]) <= position_m and all(
        abs((x - y + 180) % 360 - 180) <= angle_deg for x, y in zip(a[3:], b[3:], strict=True)
    )


def matches_view(pair: Any, desired: Any, *, min_frame: int) -> bool:
    if not (pair.frame > min_frame and pair.rgb.frame == pair.instance.frame == pair.frame):
        return False
    if not math.isclose(pair.rgb.timestamp, pair.instance.timestamp, rel_tol=0, abs_tol=1e-6):
        return False
    return poses_close(pair.rgb.transform, desired) and poses_close(
        pair.instance.transform, desired
    )


def blocked_by(position, boxes, *, clearance_m: float = 1.0) -> tuple[int, ...]:
    if not math.isfinite(clearance_m) or clearance_m < 0:
        raise ValueError("clearance must be finite and nonnegative")
    return tuple(
        index
        for index, (lower, upper) in enumerate(boxes)
        if all(lower[i] - clearance_m <= position[i] <= upper[i] + clearance_m for i in range(3))
    )


def decode_rgb(image: Any) -> np.ndarray:
    width, height = int(image.width), int(image.height)
    if width <= 0 or height <= 0 or len(image.raw_data) != width * height * 4:
        raise ValueError("unexpected image bytes or dimensions")
    bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(height, width, 4)
    return bgra[:, :, [2, 1, 0]].copy()


@dataclass(frozen=True)
class Crop:
    image: np.ndarray
    bbox: tuple[float, float, float, float]
    box_valid: bool


def crop_target(rgb: np.ndarray, box: ProjectedBox, *, size: int = 300) -> Crop:
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8 or size <= 0:
        raise ValueError("expected uint8 RGB image and positive crop size")
    empty = np.zeros((size, size, 3), dtype=np.uint8)
    height, width = rgb.shape[:2]
    bounds = (box.x_min, box.y_min, box.x_max, box.y_max)
    if (
        not box.in_front
        or not all(math.isfinite(v) for v in bounds)
        or min(box.x_max, width) <= max(box.x_min, 0)
        or min(box.y_max, height) <= max(box.y_min, 0)
    ):
        return Crop(empty, (0.0, 0.0, 0.0, 0.0), False)
    left = math.floor((box.x_min + box.x_max) / 2 - size / 2)
    top = math.floor((box.y_min + box.y_max) / 2 - size / 2)
    x0, y0 = max(left, 0), max(top, 0)
    x1, y1 = min(left + size, width), min(top + size, height)
    if x1 <= x0 or y1 <= y0:
        return Crop(empty, (0.0, 0.0, 0.0, 0.0), False)
    empty[y0 - top : y1 - top, x0 - left : x1 - left] = rgb[y0:y1, x0:x1]
    bbox = tuple(
        float(np.clip(v / size, 0, 1))
        for v in (
            max(box.x_min, 0) - left,
            max(box.y_min, 0) - top,
            min(box.x_max, width) - left,
            min(box.y_max, height) - top,
        )
    )
    valid = bbox[0] < bbox[2] and bbox[1] < bbox[3]
    return Crop(empty, bbox if valid else (0.0, 0.0, 0.0, 0.0), valid)
