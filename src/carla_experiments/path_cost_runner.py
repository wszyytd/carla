from __future__ import annotations

import math
import statistics
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .actors import configure_target_path, spawn_target_vehicle
from .config import PathCostConfig
from .metrics import (
    ObservationMetrics,
    build_projection_matrix,
    count_instance_pixels,
    evaluate_observation,
    path_length,
    project_bounding_box,
)
from .runtime import SynchronousSession
from .sensors import FramePair, spawn_paired_camera_rig
from .storage import PilotArtifacts
from .trajectories.baselines import (
    MotionState,
    UavLimits,
    Vec3,
    angle_distance,
    command_baseline,
    look_at,
    norm,
)
from .trajectories.s_curve import select_s_curve_route


@dataclass(frozen=True)
class EpisodeSummary:
    experiment_id: str
    policy: str
    completed_route: bool
    target_execution_valid: bool
    observation_valid_fraction: float
    longest_invalid_seconds: float
    uav_path_length_m: float
    target_path_length_m: float
    path_length_ratio: float
    episode_success: bool
    measured_world_frames: int
    measured_sensor_frames: int


@dataclass(frozen=True)
class FrameContext:
    frame: int
    sim_time_s: float
    target_transform: Any
    target_speed_mps: float
    motion_state: MotionState
    camera_transform: Any
    jerk_mps3: float
    gimbal_rate_deg_s: float


def aggregate_observations(
    valid_flags: Sequence[bool],
    *,
    sensor_tick_seconds: float,
    min_valid_fraction: float,
    max_continuous_invalid_seconds: float,
) -> tuple[float, float, bool]:
    if not valid_flags:
        return 0.0, 0.0, False
    valid_fraction = sum(valid_flags) / len(valid_flags)
    longest_run = 0
    current_run = 0
    for valid in valid_flags:
        if valid:
            current_run = 0
        else:
            current_run += 1
            longest_run = max(longest_run, current_run)
    longest_invalid = longest_run * sensor_tick_seconds
    return (
        valid_fraction,
        longest_invalid,
        valid_fraction >= min_valid_fraction
        and longest_invalid <= max_continuous_invalid_seconds,
    )


def evaluate_target_execution(
    *,
    completed: bool,
    route_indices: Sequence[int],
    cross_track_errors_m: Sequence[float],
    speeds_mps: Sequence[float],
    max_cross_track_error_m: float,
    target_speed_mps: float,
    speed_tolerance_fraction: float,
) -> bool:
    if not completed or not route_indices or not cross_track_errors_m or not speeds_mps:
        return False
    monotonic = all(
        current >= previous
        for previous, current in zip(route_indices, route_indices[1:], strict=False)
    )
    speed_error = abs(statistics.median(speeds_mps) - target_speed_mps)
    return (
        monotonic
        and max(cross_track_errors_m) <= max_cross_track_error_m
        and speed_error <= target_speed_mps * speed_tolerance_fraction
    )


def _vec3(location: Any) -> Vec3:
    return Vec3(float(location.x), float(location.y), float(location.z))


def _distance(first: Vec3, second: Vec3) -> float:
    return norm(second - first)


def _plain(value: Any) -> Any:
    if isinstance(value, Path):
        return value.as_posix()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def _experiment_id(timestamp: datetime, policy: str, seed: int) -> str:
    normalized = timestamp.astimezone(timezone.utc)
    return f"{normalized:%Y%m%dT%H%M%SZ}-{policy}-seed{seed}"


def _camera_transform(carla_module: Any, state: MotionState) -> Any:
    return carla_module.Transform(
        carla_module.Location(state.position.x, state.position.y, state.position.z),
        carla_module.Rotation(
            pitch=state.gimbal.pitch,
            yaw=state.gimbal.yaw,
            roll=state.gimbal.roll,
        ),
    )


def _target_speed(target: Any) -> float:
    return norm(_vec3(target.get_velocity()))


def _nearest_route(target: Vec3, route_locations: tuple[Vec3, ...]) -> tuple[int, float]:
    distances = tuple(_distance(target, location) for location in route_locations)
    index = min(range(len(distances)), key=distances.__getitem__)
    return index, distances[index]


def _default_observation_evaluator(
    pair: FramePair,
    context: FrameContext,
    target: Any,
    config: PathCostConfig,
    intrinsics: np.ndarray,
    limits: UavLimits,
) -> ObservationMetrics:
    vertices = target.bounding_box.get_world_vertices(context.target_transform)
    world_vertices = np.array(
        [[vertex.x, vertex.y, vertex.z] for vertex in vertices],
        dtype=np.float64,
    )
    projected = project_bounding_box(
        world_vertices,
        np.asarray(context.camera_transform.get_inverse_matrix(), dtype=np.float64),
        intrinsics,
    )
    pixels = count_instance_pixels(
        bytes(pair.instance.raw_data),
        width=config.camera.width,
        height=config.camera.height,
        actor_id=int(target.id),
    )
    target_position = _vec3(context.target_transform.location)
    return evaluate_observation(
        projected_box=projected,
        instance_pixels=pixels,
        distance_m=_distance(context.motion_state.position, target_position),
        speed_mps=norm(context.motion_state.velocity),
        acceleration_mps2=norm(context.motion_state.acceleration),
        jerk_mps3=context.jerk_mps3,
        gimbal_rate_deg_s=context.gimbal_rate_deg_s,
        width=config.camera.width,
        height=config.camera.height,
        config=config.observation,
        limits=limits,
    )


def _trajectory_row(context: FrameContext, policy: str) -> dict[str, Any]:
    target = context.target_transform.location
    state = context.motion_state
    return {
        "frame": context.frame,
        "sim_time_s": context.sim_time_s,
        "target_x": target.x,
        "target_y": target.y,
        "target_z": target.z,
        "target_speed_mps": context.target_speed_mps,
        "uav_x": state.position.x,
        "uav_y": state.position.y,
        "uav_z": state.position.z,
        "uav_vx": state.velocity.x,
        "uav_vy": state.velocity.y,
        "uav_vz": state.velocity.z,
        "uav_ax": state.acceleration.x,
        "uav_ay": state.acceleration.y,
        "uav_az": state.acceleration.z,
        "gimbal_pitch": state.gimbal.pitch,
        "gimbal_yaw": state.gimbal.yaw,
        "policy": policy,
    }


def _metric_row(
    pair: FramePair, context: FrameContext, metrics: ObservationMetrics
) -> dict[str, Any]:
    box = metrics.projected_box
    state = context.motion_state
    return {
        "frame": context.frame,
        "sim_time_s": context.sim_time_s,
        "sensor_frame": pair.frame,
        "bbox_x_min": box.x_min,
        "bbox_y_min": box.y_min,
        "bbox_x_max": box.x_max,
        "bbox_y_max": box.y_max,
        "bbox_short_side_px": box.short_side_px,
        "instance_pixels": metrics.instance_pixels,
        "center_error_fraction": metrics.center_error_fraction,
        "distance_m": metrics.distance_m,
        "speed_mps": norm(state.velocity),
        "acceleration_mps2": norm(state.acceleration),
        "jerk_mps3": context.jerk_mps3,
        "gimbal_rate_deg_s": context.gimbal_rate_deg_s,
        "valid": metrics.valid,
        "invalid_reasons": metrics.invalid_reasons,
    }


def run_path_cost_episode(
    carla_module: Any,
    config: PathCostConfig,
    policy: str,
    *,
    clock: Callable[[], datetime] | None = None,
    session_factory: Callable[..., Any] = SynchronousSession,
    route_selector: Callable[..., Any] = select_s_curve_route,
    target_spawner: Callable[..., Any] = spawn_target_vehicle,
    target_configurator: Callable[..., Any] = configure_target_path,
    sensor_factory: Callable[..., Any] = spawn_paired_camera_rig,
    artifacts_factory: Callable[..., Any] = PilotArtifacts,
    observation_evaluator: Callable[..., ObservationMetrics] = _default_observation_evaluator,
) -> EpisodeSummary:
    """Run one synchronous Hover or Vertical Follow pilot episode."""

    now = clock or (lambda: datetime.now(timezone.utc))
    experiment_id = _experiment_id(now(), policy, config.random_seed)
    client = carla_module.Client(config.client.host, config.client.port)
    client.set_timeout(config.client.timeout_seconds)
    limits = UavLimits(
        config.uav.max_speed_mps,
        config.uav.max_acceleration_mps2,
        config.uav.max_jerk_mps3,
        config.uav.max_gimbal_rate_deg_s,
    )
    intrinsics = build_projection_matrix(
        width=config.camera.width,
        height=config.camera.height,
        fov_deg=config.camera.fov_deg,
    )

    with session_factory(client, config) as session:
        map_obj = session.world.get_map()
        if not map_obj.name.endswith(config.world.map):
            raise RuntimeError(
                f"configured map {config.world.map!r} does not match current map {map_obj.name!r}"
            )
        route = route_selector(map_obj, config.route)
        target = target_spawner(session.world, route, config.target, session.actors)
        target_configurator(
            target,
            route,
            session.traffic_manager,
            traffic_manager_port=config.traffic_manager.port,
            target_speed_mps=config.route.target_speed_mps,
        )

        route_locations = tuple(_vec3(item.transform.location) for item in route.waypoints)
        target_start = route_locations[0]
        hover_position = Vec3(
            target_start.x + config.uav.initial_offset_x_m,
            target_start.y + config.uav.initial_offset_y_m,
            target_start.z + config.uav.altitude_m,
        )
        motion_state = MotionState(
            position=hover_position,
            velocity=Vec3(0.0, 0.0, 0.0),
            acceleration=Vec3(0.0, 0.0, 0.0),
            gimbal=look_at(hover_position, target_start),
        )
        camera_transform = _camera_transform(carla_module, motion_state)
        rig = sensor_factory(
            session.world,
            config.camera,
            camera_transform,
            session.actors,
        )
        artifacts = artifacts_factory(
            config.output.root,
            experiment_id,
            resolved_config=_plain(asdict(config)),
            episode_metadata={
                "experiment_id": experiment_id,
                "map": map_obj.name,
                "policy": policy,
                "road_id": route.road_id,
                "section_id": route.section_id,
                "lane_id": route.lane_id,
                "start_s": route.start_s,
            },
        )

        for _ in range(config.output.warmup_frames):
            warmup_frame = session.tick()
            rig.drain_ready(warmup_frame)

        initial_target = _vec3(target.get_transform().location)
        target_positions = [initial_target]
        uav_positions = [motion_state.position]
        contexts: dict[int, FrameContext] = {}
        valid_flags: list[bool] = []
        route_indices: list[int] = []
        cross_track_errors: list[float] = []
        target_speeds: list[float] = []
        completed_route = False
        measured_world_frames = 0

        def process_pairs(pairs: Sequence[FramePair]) -> None:
            for pair in pairs:
                context = contexts.pop(pair.frame, None)
                if context is None:
                    continue
                metrics = observation_evaluator(
                    pair,
                    context,
                    target,
                    config,
                    intrinsics,
                    limits,
                )
                artifacts.append_frame_metrics(_metric_row(pair, context, metrics))
                if len(valid_flags) % config.output.sample_every_frames == 0:
                    artifacts.save_sample(
                        pair.frame,
                        bytes(pair.rgb.raw_data),
                        width=config.camera.width,
                        height=config.camera.height,
                    )
                valid_flags.append(bool(metrics.valid))

        maximum_steps = math.ceil(
            config.route.max_duration_seconds / config.world.fixed_delta_seconds
        )
        for step in range(maximum_steps):
            target_before_tick = _vec3(target.get_transform().location)
            previous_state = motion_state
            command = command_baseline(
                policy,
                motion_state,
                target_position=target_before_tick,
                hover_position=hover_position,
                altitude_m=config.uav.altitude_m,
                limits=limits,
                dt=config.world.fixed_delta_seconds,
            )
            motion_state = command.state
            camera_transform = _camera_transform(carla_module, motion_state)
            rig.set_transform(camera_transform)
            frame = session.tick()
            measured_world_frames += 1

            target_transform = target.get_transform()
            target_position = _vec3(target_transform.location)
            target_speed = _target_speed(target)
            pitch_delta = motion_state.gimbal.pitch - previous_state.gimbal.pitch
            yaw_delta = angle_distance(
                previous_state.gimbal.yaw,
                motion_state.gimbal.yaw,
            )
            jerk = norm(
                motion_state.acceleration - previous_state.acceleration
            ) / config.world.fixed_delta_seconds
            gimbal_rate = math.hypot(pitch_delta, yaw_delta) / config.world.fixed_delta_seconds
            context = FrameContext(
                frame=frame,
                sim_time_s=(step + 1) * config.world.fixed_delta_seconds,
                target_transform=target_transform,
                target_speed_mps=target_speed,
                motion_state=motion_state,
                camera_transform=camera_transform,
                jerk_mps3=jerk,
                gimbal_rate_deg_s=gimbal_rate,
            )
            contexts[frame] = context
            artifacts.append_trajectory(_trajectory_row(context, policy))
            process_pairs(rig.drain_ready(frame))

            route_index, cross_track = _nearest_route(target_position, route_locations)
            route_indices.append(route_index)
            cross_track_errors.append(cross_track)
            target_speeds.append(target_speed)
            target_positions.append(target_position)
            uav_positions.append(motion_state.position)
            completed_route = (
                _distance(target_position, route_locations[-1])
                <= config.route.completion_tolerance_m
            )
            if completed_route:
                break

        if measured_world_frames:
            last_frame = max(contexts, default=0)
            process_pairs(rig.drain_ready(last_frame))
            try:
                process_pairs(
                    rig.wait_for_ready(
                        last_frame,
                        timeout_seconds=min(
                            config.camera.sensor_tick_seconds * 2.0,
                            config.client.timeout_seconds,
                        ),
                    )
                )
            except TimeoutError:
                pass

        target_execution_valid = evaluate_target_execution(
            completed=completed_route,
            route_indices=route_indices,
            cross_track_errors_m=cross_track_errors,
            speeds_mps=target_speeds,
            max_cross_track_error_m=config.route.max_cross_track_error_m,
            target_speed_mps=config.route.target_speed_mps,
            speed_tolerance_fraction=config.route.speed_tolerance_fraction,
        )
        valid_fraction, longest_invalid, observation_valid = aggregate_observations(
            valid_flags,
            sensor_tick_seconds=config.camera.sensor_tick_seconds,
            min_valid_fraction=config.observation.min_valid_fraction,
            max_continuous_invalid_seconds=(
                config.observation.max_continuous_invalid_seconds
            ),
        )
        uav_distance = path_length(tuple(uav_positions))
        target_distance = path_length(tuple(target_positions))
        summary = EpisodeSummary(
            experiment_id=experiment_id,
            policy=policy,
            completed_route=completed_route,
            target_execution_valid=target_execution_valid,
            observation_valid_fraction=valid_fraction,
            longest_invalid_seconds=longest_invalid,
            uav_path_length_m=uav_distance,
            target_path_length_m=target_distance,
            path_length_ratio=(uav_distance / target_distance if target_distance > 0.0 else 0.0),
            episode_success=target_execution_valid and observation_valid,
            measured_world_frames=measured_world_frames,
            measured_sensor_frames=len(valid_flags),
        )
        artifacts.finalize(asdict(summary))
        return summary
