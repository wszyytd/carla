"""Target projection, visibility, synchronization, and performance metrics."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .config import ObservationConfig
from .trajectories.baselines import UavLimits, Vec3, norm


@dataclass(frozen=True)
class ProjectedBox:
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    in_front: bool

    @property
    def short_side_px(self) -> float:
        return min(self.x_max - self.x_min, self.y_max - self.y_min)

    @property
    def center(self) -> tuple[float, float]:
        return (self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0


@dataclass(frozen=True)
class ObservationMetrics:
    projected_box: ProjectedBox
    instance_pixels: int
    center_error_fraction: float
    distance_m: float
    within_edge_margin: bool
    large_enough: bool
    centered: bool
    within_distance: bool
    enough_instance_pixels: bool
    within_speed_limit: bool
    within_acceleration_limit: bool
    within_jerk_limit: bool
    within_gimbal_rate_limit: bool
    valid: bool
    invalid_reasons: tuple[str, ...]


@dataclass(frozen=True)
class MotionMetrics:
    velocities: tuple[Vec3, ...]
    accelerations: tuple[Vec3, ...]
    jerks: tuple[Vec3, ...]
    speeds: tuple[float, ...]
    acceleration_magnitudes: tuple[float, ...]
    jerk_magnitudes: tuple[float, ...]


def build_projection_matrix(*, width: int, height: int, fov_deg: float) -> NDArray[np.float64]:
    """Build the pinhole intrinsics for a horizontal field of view."""

    focal = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    return np.array(
        [
            [focal, 0.0, width / 2.0],
            [0.0, focal, height / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def project_bounding_box(
    world_vertices: NDArray[np.float64],
    world_to_camera: NDArray[np.float64],
    intrinsics: NDArray[np.float64],
) -> ProjectedBox:
    """Project UE world vertices without clipping image-space extrema."""

    vertices = np.asarray(world_vertices, dtype=np.float64)
    inverse = np.asarray(world_to_camera, dtype=np.float64)
    camera_matrix = np.asarray(intrinsics, dtype=np.float64)
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError("world_vertices must have shape (N, 3)")
    if vertices.shape[0] == 0:
        raise ValueError("world_vertices must not be empty")
    if inverse.shape != (4, 4):
        raise ValueError("world_to_camera must have shape (4, 4)")
    if camera_matrix.shape != (3, 3):
        raise ValueError("intrinsics must have shape (3, 3)")

    homogeneous = np.column_stack((vertices, np.ones(vertices.shape[0], dtype=np.float64)))
    ue_camera = (inverse @ homogeneous.T).T[:, :3]
    carla_camera = np.column_stack(
        (ue_camera[:, 1], -ue_camera[:, 2], ue_camera[:, 0])
    )
    if np.any(carla_camera[:, 2] <= 0.0):
        nan = float("nan")
        return ProjectedBox(nan, nan, nan, nan, False)

    pixels_homogeneous = (camera_matrix @ carla_camera.T).T
    pixels = pixels_homogeneous[:, :2] / pixels_homogeneous[:, 2, np.newaxis]
    return ProjectedBox(
        x_min=float(np.min(pixels[:, 0])),
        y_min=float(np.min(pixels[:, 1])),
        x_max=float(np.max(pixels[:, 0])),
        y_max=float(np.max(pixels[:, 1])),
        in_front=True,
    )


def count_dominant_vehicle_instance_pixels(
    raw: bytes,
    *,
    width: int,
    height: int,
    projected_box: ProjectedBox,
    vehicle_semantic_tag: int = 10,
) -> int:
    """Count the dominant vehicle instance inside one projected target box.

    CARLA's instance color is not its Python actor ID.  This heuristic is only
    valid for the present single-target, no-background-traffic pilot; it does
    not identify a particular vehicle in a multi-vehicle experiment.
    """
    expected_length = width * height * 4
    if len(raw) != expected_length:
        raise ValueError(
            f"raw image length must be {expected_length} bytes, received {len(raw)}"
        )
    if not projected_box.in_front:
        return 0

    bounds = (
        projected_box.x_min,
        projected_box.y_min,
        projected_box.x_max,
        projected_box.y_max,
    )
    if not all(math.isfinite(value) for value in bounds):
        return 0

    x_min = max(0, math.floor(projected_box.x_min))
    y_min = max(0, math.floor(projected_box.y_min))
    x_max = min(width, math.ceil(projected_box.x_max))
    y_max = min(height, math.ceil(projected_box.y_max))
    if x_min >= x_max or y_min >= y_max:
        return 0

    bgra = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
    crop = bgra[y_min:y_max, x_min:x_max]
    vehicle_pixels = crop[crop[:, :, 2] == vehicle_semantic_tag]
    if len(vehicle_pixels) == 0:
        return 0
    _, counts = np.unique(vehicle_pixels[:, (1, 0)], axis=0, return_counts=True)
    return int(np.max(counts))


def evaluate_observation(
    *,
    projected_box: ProjectedBox,
    instance_pixels: int,
    distance_m: float,
    speed_mps: float,
    acceleration_mps2: float,
    jerk_mps3: float,
    gimbal_rate_deg_s: float,
    width: int,
    height: int,
    config: ObservationConfig,
    limits: UavLimits,
) -> ObservationMetrics:
    """Evaluate every independent observation and camera-motion constraint."""

    if projected_box.in_front:
        center_x, center_y = projected_box.center
        center_error = math.hypot(center_x - width / 2.0, center_y - height / 2.0)
        center_error_fraction = center_error / math.hypot(width / 2.0, height / 2.0)
    else:
        center_error_fraction = float("inf")

    margin_x = width * config.edge_margin_fraction
    margin_y = height * config.edge_margin_fraction
    within_edge_margin = projected_box.in_front and (
        projected_box.x_min >= margin_x
        and projected_box.y_min >= margin_y
        and projected_box.x_max <= width - margin_x
        and projected_box.y_max <= height - margin_y
    )
    checks = {
        "behind_camera": projected_box.in_front,
        "edge_margin": within_edge_margin,
        "short_side": projected_box.in_front
        and projected_box.short_side_px >= config.min_bbox_short_side_px,
        "center_error": center_error_fraction <= config.max_center_error_fraction,
        "distance": distance_m <= config.max_distance_m,
        "instance_pixels": instance_pixels >= config.min_instance_pixels,
        "speed": speed_mps <= limits.max_speed_mps,
        "acceleration": acceleration_mps2 <= limits.max_acceleration_mps2,
        "jerk": jerk_mps3 <= limits.max_jerk_mps3,
        "gimbal_rate": gimbal_rate_deg_s <= limits.max_gimbal_rate_deg_s,
    }
    invalid_reasons = tuple(name for name, passed in checks.items() if not passed)
    return ObservationMetrics(
        projected_box=projected_box,
        instance_pixels=instance_pixels,
        center_error_fraction=center_error_fraction,
        distance_m=distance_m,
        within_edge_margin=within_edge_margin,
        large_enough=checks["short_side"],
        centered=checks["center_error"],
        within_distance=checks["distance"],
        enough_instance_pixels=checks["instance_pixels"],
        within_speed_limit=checks["speed"],
        within_acceleration_limit=checks["acceleration"],
        within_jerk_limit=checks["jerk"],
        within_gimbal_rate_limit=checks["gimbal_rate"],
        valid=not invalid_reasons,
        invalid_reasons=invalid_reasons,
    )


def path_length(positions: tuple[Vec3, ...]) -> float:
    return sum(
        norm(current - previous)
        for previous, current in zip(positions, positions[1:], strict=False)
    )


def _differentiate(samples: tuple[Vec3, ...], dt: float) -> tuple[Vec3, ...]:
    if not samples:
        return ()
    zero = Vec3(0.0, 0.0, 0.0)
    return (zero,) + tuple(
        (current - previous) * (1.0 / dt)
        for previous, current in zip(samples, samples[1:], strict=False)
    )


def derive_motion_metrics(positions: tuple[Vec3, ...], *, dt: float) -> MotionMetrics:
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be a positive finite number")
    velocities = _differentiate(positions, dt)
    accelerations = _differentiate(velocities, dt)
    jerks = _differentiate(accelerations, dt)
    return MotionMetrics(
        velocities=velocities,
        accelerations=accelerations,
        jerks=jerks,
        speeds=tuple(norm(item) for item in velocities),
        acceleration_magnitudes=tuple(norm(item) for item in accelerations),
        jerk_magnitudes=tuple(norm(item) for item in jerks),
    )
