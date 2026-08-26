from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ClientConfig:
    """Validated settings for one CARLA client connection."""

    host: str
    port: int
    timeout_seconds: float


@dataclass(frozen=True)
class WorldConfig:
    map: str
    fixed_delta_seconds: float


@dataclass(frozen=True)
class TrafficManagerConfig:
    port: int
    random_seed: int
    synchronous_mode: bool


@dataclass(frozen=True)
class RouteConfig:
    waypoint_spacing_m: float
    window_length_m: float
    min_turn_each_direction_deg: float
    candidate_rank: int
    target_speed_mps: float
    completion_tolerance_m: float
    max_cross_track_error_m: float
    speed_tolerance_fraction: float
    max_duration_seconds: float


@dataclass(frozen=True)
class TargetConfig:
    blueprint_filter: str


@dataclass(frozen=True)
class CameraConfig:
    width: int
    height: int
    fov_deg: float
    sensor_tick_seconds: float

    @property
    def resolution(self) -> tuple[int, int]:
        return self.width, self.height


@dataclass(frozen=True)
class UavConfig:
    altitude_m: float
    initial_offset_x_m: float
    initial_offset_y_m: float
    max_speed_mps: float
    max_acceleration_mps2: float
    max_jerk_mps3: float
    max_gimbal_rate_deg_s: float


@dataclass(frozen=True)
class ObservationConfig:
    edge_margin_fraction: float
    min_bbox_short_side_px: float
    max_center_error_fraction: float
    max_distance_m: float
    min_instance_pixels: int
    min_valid_fraction: float
    max_continuous_invalid_seconds: float


@dataclass(frozen=True)
class OutputConfig:
    root: Path
    warmup_frames: int
    sample_every_frames: int


@dataclass(frozen=True)
class PathCostConfig:
    client: ClientConfig
    world: WorldConfig
    traffic_manager: TrafficManagerConfig
    route: RouteConfig
    target: TargetConfig
    camera: CameraConfig
    uav: UavConfig
    observation: ObservationConfig
    output: OutputConfig
    random_seed: int


def load_config(path: str | Path) -> dict[str, Any]:
    """Load one YAML configuration whose root is a mapping."""

    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)

    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value


def parse_client_config(config: Mapping[str, Any]) -> ClientConfig:
    """Validate and normalize the ``client`` section of a configuration."""

    client = config.get("client")
    if not isinstance(client, Mapping):
        raise ValueError("client must be a mapping")

    host = client.get("host")
    if not isinstance(host, str) or not host.strip():
        raise ValueError("client.host must be a non-empty string")

    port = client.get("port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("client.port must be an integer from 1 to 65535")

    timeout = client.get("timeout_seconds")
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or not math.isfinite(timeout)
        or timeout <= 0
    ):
        raise ValueError("client.timeout_seconds must be a positive finite number")

    return ClientConfig(host=host.strip(), port=port, timeout_seconds=float(timeout))


def _mapping(config: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = config.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a mapping")
    return value


def _string(config: Mapping[str, Any], key: str, path: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path} must be a non-empty string")
    return value.strip()


def _boolean(config: Mapping[str, Any], key: str, path: str) -> bool:
    value = config.get(key)
    if not isinstance(value, bool):
        raise ValueError(f"{path} must be a boolean")
    return value


def _integer(
    config: Mapping[str, Any],
    key: str,
    path: str,
    *,
    minimum: int = 0,
    maximum: int | None = None,
) -> int:
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path} must be an integer")
    if value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"{path} is outside the allowed range")
    return value


def _number(
    config: Mapping[str, Any],
    key: str,
    path: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    minimum_inclusive: bool = False,
    maximum_inclusive: bool = True,
) -> float:
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{path} must be a finite number")
    if minimum is not None:
        below = normalized < minimum if minimum_inclusive else normalized <= minimum
        if below:
            raise ValueError(f"{path} is outside the allowed range")
    if maximum is not None:
        above = normalized > maximum if maximum_inclusive else normalized >= maximum
        if above:
            raise ValueError(f"{path} is outside the allowed range")
    return normalized


def parse_path_cost_config(config: Mapping[str, Any]) -> PathCostConfig:
    """Validate and normalize the complete path-cost pilot configuration."""

    random_seed = _integer(config, "random_seed", "random_seed")
    world_values = _mapping(config, "world")
    traffic_values = _mapping(config, "traffic_manager")
    route_values = _mapping(config, "route")
    target_values = _mapping(config, "target")
    camera_values = _mapping(config, "camera")
    uav_values = _mapping(config, "uav")
    observation_values = _mapping(config, "observation")
    output_values = _mapping(config, "output")

    world = WorldConfig(
        map=_string(world_values, "map", "world.map"),
        fixed_delta_seconds=_number(
            world_values,
            "fixed_delta_seconds",
            "world.fixed_delta_seconds",
            minimum=0.0,
        ),
    )
    traffic_manager = TrafficManagerConfig(
        port=_integer(
            traffic_values, "port", "traffic_manager.port", minimum=1, maximum=65535
        ),
        random_seed=_integer(
            traffic_values, "random_seed", "traffic_manager.random_seed"
        ),
        synchronous_mode=_boolean(
            traffic_values, "synchronous_mode", "traffic_manager.synchronous_mode"
        ),
    )
    route = RouteConfig(
        waypoint_spacing_m=_number(
            route_values, "waypoint_spacing_m", "route.waypoint_spacing_m", minimum=0.0
        ),
        window_length_m=_number(
            route_values, "window_length_m", "route.window_length_m", minimum=0.0
        ),
        min_turn_each_direction_deg=_number(
            route_values,
            "min_turn_each_direction_deg",
            "route.min_turn_each_direction_deg",
            minimum=0.0,
        ),
        candidate_rank=_integer(route_values, "candidate_rank", "route.candidate_rank"),
        target_speed_mps=_number(
            route_values, "target_speed_mps", "route.target_speed_mps", minimum=0.0
        ),
        completion_tolerance_m=_number(
            route_values,
            "completion_tolerance_m",
            "route.completion_tolerance_m",
            minimum=0.0,
        ),
        max_cross_track_error_m=_number(
            route_values,
            "max_cross_track_error_m",
            "route.max_cross_track_error_m",
            minimum=0.0,
        ),
        speed_tolerance_fraction=_number(
            route_values,
            "speed_tolerance_fraction",
            "route.speed_tolerance_fraction",
            minimum=0.0,
            maximum=1.0,
            minimum_inclusive=True,
        ),
        max_duration_seconds=_number(
            route_values, "max_duration_seconds", "route.max_duration_seconds", minimum=0.0
        ),
    )
    target = TargetConfig(
        blueprint_filter=_string(target_values, "blueprint_filter", "target.blueprint_filter")
    )
    camera = CameraConfig(
        width=_integer(camera_values, "width", "camera.width", minimum=1),
        height=_integer(camera_values, "height", "camera.height", minimum=1),
        fov_deg=_number(
            camera_values,
            "fov_deg",
            "camera.fov_deg",
            minimum=0.0,
            maximum=180.0,
            maximum_inclusive=False,
        ),
        sensor_tick_seconds=_number(
            camera_values,
            "sensor_tick_seconds",
            "camera.sensor_tick_seconds",
            minimum=0.0,
        ),
    )
    ratio = camera.sensor_tick_seconds / world.fixed_delta_seconds
    if camera.sensor_tick_seconds < world.fixed_delta_seconds or not math.isclose(
        ratio, round(ratio), abs_tol=1e-9
    ):
        raise ValueError(
            "camera.sensor_tick_seconds must be an integer multiple of "
            "world.fixed_delta_seconds"
        )
    uav = UavConfig(
        altitude_m=_number(uav_values, "altitude_m", "uav.altitude_m", minimum=0.0),
        initial_offset_x_m=_number(
            uav_values,
            "initial_offset_x_m",
            "uav.initial_offset_x_m",
            minimum_inclusive=True,
        ),
        initial_offset_y_m=_number(
            uav_values,
            "initial_offset_y_m",
            "uav.initial_offset_y_m",
            minimum_inclusive=True,
        ),
        max_speed_mps=_number(
            uav_values, "max_speed_mps", "uav.max_speed_mps", minimum=0.0
        ),
        max_acceleration_mps2=_number(
            uav_values,
            "max_acceleration_mps2",
            "uav.max_acceleration_mps2",
            minimum=0.0,
        ),
        max_jerk_mps3=_number(
            uav_values, "max_jerk_mps3", "uav.max_jerk_mps3", minimum=0.0
        ),
        max_gimbal_rate_deg_s=_number(
            uav_values,
            "max_gimbal_rate_deg_s",
            "uav.max_gimbal_rate_deg_s",
            minimum=0.0,
        ),
    )
    observation = ObservationConfig(
        edge_margin_fraction=_number(
            observation_values,
            "edge_margin_fraction",
            "observation.edge_margin_fraction",
            minimum=0.0,
            maximum=0.5,
            minimum_inclusive=True,
            maximum_inclusive=False,
        ),
        min_bbox_short_side_px=_number(
            observation_values,
            "min_bbox_short_side_px",
            "observation.min_bbox_short_side_px",
            minimum=0.0,
        ),
        max_center_error_fraction=_number(
            observation_values,
            "max_center_error_fraction",
            "observation.max_center_error_fraction",
            minimum=0.0,
            maximum=1.0,
            minimum_inclusive=True,
        ),
        max_distance_m=_number(
            observation_values,
            "max_distance_m",
            "observation.max_distance_m",
            minimum=0.0,
        ),
        min_instance_pixels=_integer(
            observation_values,
            "min_instance_pixels",
            "observation.min_instance_pixels",
        ),
        min_valid_fraction=_number(
            observation_values,
            "min_valid_fraction",
            "observation.min_valid_fraction",
            minimum=0.0,
            maximum=1.0,
            minimum_inclusive=True,
        ),
        max_continuous_invalid_seconds=_number(
            observation_values,
            "max_continuous_invalid_seconds",
            "observation.max_continuous_invalid_seconds",
            minimum=0.0,
            minimum_inclusive=True,
        ),
    )
    output = OutputConfig(
        root=Path(_string(output_values, "root", "output.root")),
        warmup_frames=_integer(output_values, "warmup_frames", "output.warmup_frames"),
        sample_every_frames=_integer(
            output_values, "sample_every_frames", "output.sample_every_frames", minimum=1
        ),
    )
    return PathCostConfig(
        client=parse_client_config(config),
        world=world,
        traffic_manager=traffic_manager,
        route=route,
        target=target,
        camera=camera,
        uav=uav,
        observation=observation,
        output=output,
        random_seed=random_seed,
    )
