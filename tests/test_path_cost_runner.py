from dataclasses import dataclass, replace
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from src.carla_experiments.config import load_config, parse_path_cost_config
from src.carla_experiments.metrics import build_projection_matrix
from src.carla_experiments.path_cost_runner import (
    EpisodeSummary,
    FrameContext,
    _default_observation_evaluator,
    aggregate_observations,
    evaluate_target_execution,
    run_path_cost_episode,
)
from src.carla_experiments.runtime import OwnedActors
from src.carla_experiments.sensors import FramePair
from src.carla_experiments.trajectories.baselines import (
    MotionState,
    UavLimits,
    Vec3,
    look_at,
)


@dataclass(frozen=True)
class Location:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Rotation:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


@dataclass(frozen=True)
class Transform:
    location: Location
    rotation: Rotation

    def get_inverse_matrix(self) -> list[list[float]]:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]


class Client:
    latest = None

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.timeout = None
        Client.latest = self

    def set_timeout(self, timeout: float) -> None:
        self.timeout = timeout


CARLA = SimpleNamespace(Client=Client, Location=Location, Rotation=Rotation, Transform=Transform)


class FakeMap:
    name = "Carla/Maps/Town10HD_Opt"


class FakeWorld:
    def get_map(self) -> FakeMap:
        return FakeMap()


class FakeSession:
    latest = None

    def __init__(self, client, config) -> None:
        self.client = client
        self.config = config
        self.world = FakeWorld()
        self.traffic_manager = object()
        self.actors = OwnedActors()
        self.frame = 0
        self.exited = False
        FakeSession.latest = self

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.exited = True
        self.actors.destroy_all()
        return False

    def tick(self) -> int:
        self.frame += 1
        return self.frame


def make_route(*, final_x: float = 5.0):
    count = int(final_x) + 1
    points = tuple(
        SimpleNamespace(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=float(index),
            transform=Transform(Location(float(index), 0.0, 0.0), Rotation()),
        )
        for index in range(count)
    )
    return SimpleNamespace(
        waypoints=points,
        road_id=1,
        section_id=0,
        lane_id=1,
        start_s=0.0,
        length_m=final_x,
    )


class Target:
    id = 1437

    def __init__(self, session: FakeSession) -> None:
        self.session = session
        self.bounding_box = SimpleNamespace(get_world_vertices=lambda transform: ())

    def get_transform(self) -> Transform:
        return Transform(Location(float(self.session.frame), 0.0, 0.0), Rotation())

    def get_velocity(self) -> Location:
        return Location(8.0, 0.0, 0.0)


class FakeRig:
    latest = None

    def __init__(self, deliver_frames=(2, 4)) -> None:
        self.transforms: list[Transform] = []
        self.deliver_frames = deliver_frames
        self.delivered = False
        FakeRig.latest = self

    def set_transform(self, transform: Transform) -> None:
        self.transforms.append(transform)

    def drain_ready(self, max_frame: int) -> tuple[FramePair, ...]:
        if not self.delivered and max_frame >= 4 and self.deliver_frames:
            self.delivered = True
            return tuple(
                FramePair(
                    frame=frame,
                    rgb=SimpleNamespace(frame=frame, raw_data=b"rgb"),
                    instance=SimpleNamespace(frame=frame, raw_data=b"instance"),
                )
                for frame in self.deliver_frames
            )
        return ()

    def wait_for_ready(self, max_frame: int, timeout_seconds: float):
        if self.delivered:
            return ()
        raise TimeoutError(f"paired camera frames through {max_frame} timed out")


class FakeArtifacts:
    latest = None

    def __init__(self, output_root, experiment_id, **kwargs) -> None:
        self.experiment_id = experiment_id
        self.trajectory_rows: list[dict[str, object]] = []
        self.metric_rows: list[dict[str, object]] = []
        self.samples: list[int] = []
        self.summaries: list[dict[str, object]] = []
        FakeArtifacts.latest = self

    def append_trajectory(self, row) -> None:
        self.trajectory_rows.append(dict(row))

    def append_frame_metrics(self, row) -> None:
        self.metric_rows.append(dict(row))

    def save_sample(self, frame, raw, *, width, height):
        self.samples.append(frame)

    def finalize(self, summary):
        self.summaries.append(dict(summary))


def config(*, max_duration_seconds: float = 1.0):
    parsed = parse_path_cost_config(load_config("cfg/experiments/path_cost_pilot.yaml"))
    return replace(
        parsed,
        output=replace(parsed.output, warmup_frames=0),
        route=replace(
            parsed.route,
            completion_tolerance_m=0.1,
            max_duration_seconds=max_duration_seconds,
        ),
    )


def valid_metrics(*args, **kwargs):
    return SimpleNamespace(
        projected_box=SimpleNamespace(
            x_min=10.0,
            y_min=10.0,
            x_max=50.0,
            y_max=50.0,
            short_side_px=40.0,
        ),
        instance_pixels=1000,
        center_error_fraction=0.0,
        distance_m=40.0,
        valid=True,
        invalid_reasons=(),
    )


def test_default_evaluator_counts_visible_vehicle_without_reading_target_actor_id() -> None:
    """Catch a regression to actor-ID matching, which CARLA instance colors do not support."""

    class TargetWithoutReadableId:
        @property
        def id(self):
            raise AssertionError("target.id must not be used for instance-pixel matching")

        bounding_box = SimpleNamespace(
            get_world_vertices=lambda transform: tuple(
                Location(x, y, z)
                for x in (10.0,)
                for y in (-10.0, 10.0)
                for z in (-10.0, 10.0)
            )
        )

    pilot_config = replace(
        config(),
        camera=replace(config().camera, width=2, height=2, fov_deg=90.0),
    )
    camera_transform = Transform(Location(0.0, 0.0, 0.0), Rotation())
    state = MotionState(
        position=Vec3(0.0, 0.0, 0.0),
        velocity=Vec3(0.0, 0.0, 0.0),
        acceleration=Vec3(0.0, 0.0, 0.0),
        gimbal=look_at(Vec3(0.0, 0.0, 0.0), Vec3(10.0, 0.0, 0.0)),
    )
    context = FrameContext(
        frame=1,
        sim_time_s=0.05,
        target_transform=camera_transform,
        target_speed_mps=0.0,
        motion_state=state,
        camera_transform=camera_transform,
        jerk_mps3=0.0,
        gimbal_rate_deg_s=0.0,
    )
    pair = FramePair(
        frame=1,
        rgb=SimpleNamespace(frame=1, raw_data=b"rgb"),
        instance=SimpleNamespace(
            frame=1,
            raw_data=bytes(
                [
                    1,
                    2,
                    10,
                    255,
                    1,
                    2,
                    10,
                    255,
                    3,
                    4,
                    10,
                    255,
                    1,
                    2,
                    10,
                    255,
                ]
            ),
        ),
    )

    metrics = _default_observation_evaluator(
        pair,
        context,
        TargetWithoutReadableId(),
        pilot_config,
        build_projection_matrix(width=2, height=2, fov_deg=90.0),
        UavLimits(12.0, 4.0, 8.0, 90.0),
    )

    assert metrics.instance_pixels == 3


def test_runner_attributes_delayed_sensor_frames_to_matching_world_context() -> None:
    selected_route = make_route()
    calls = SimpleNamespace(route=0, spawn=0, configure=0)
    evaluated_jerks: list[float] = []

    def select_route(map_obj, route_config):
        calls.route += 1
        return selected_route

    def spawn_target(world, route, target_config, actors):
        calls.spawn += 1
        return Target(FakeSession.latest)

    def configure_target(*args, **kwargs):
        calls.configure += 1

    def evaluate_with_jerk(pair, context, *args):
        evaluated_jerks.append(context.jerk_mps3)
        return valid_metrics()

    summary = run_path_cost_episode(
        CARLA,
        config(),
        "hover",
        clock=lambda: datetime(2026, 8, 26, 12, 0, tzinfo=timezone.utc),
        session_factory=FakeSession,
        route_selector=select_route,
        target_spawner=spawn_target,
        target_configurator=configure_target,
        sensor_factory=lambda *args: FakeRig(),
        artifacts_factory=FakeArtifacts,
        observation_evaluator=evaluate_with_jerk,
    )

    artifacts = FakeArtifacts.latest
    assert isinstance(summary, EpisodeSummary)
    assert summary.completed_route is True
    assert summary.episode_success is True
    assert summary.measured_world_frames == 5
    assert summary.measured_sensor_frames == 2
    assert calls.route == calls.spawn == calls.configure == 1
    assert len(FakeRig.latest.transforms) == 5
    assert len(artifacts.trajectory_rows) == 5
    assert [row["frame"] for row in artifacts.metric_rows] == [2, 4]
    assert len(evaluated_jerks) == 2
    assert all(value <= 8.0 for value in evaluated_jerks)
    assert len(artifacts.summaries) == 1
    assert FakeSession.latest.exited is True
    assert Client.latest.timeout == 10.0


def test_runner_finalizes_failed_summary_on_route_and_sensor_timeout() -> None:
    summary = run_path_cost_episode(
        CARLA,
        config(max_duration_seconds=0.15),
        "hover",
        clock=lambda: datetime(2026, 8, 26, 12, 0, tzinfo=timezone.utc),
        session_factory=FakeSession,
        route_selector=lambda *args: make_route(final_x=100.0),
        target_spawner=lambda *args: Target(FakeSession.latest),
        target_configurator=lambda *args, **kwargs: None,
        sensor_factory=lambda *args: FakeRig(deliver_frames=()),
        artifacts_factory=FakeArtifacts,
        observation_evaluator=valid_metrics,
    )

    assert summary.completed_route is False
    assert summary.measured_sensor_frames == 0
    assert summary.episode_success is False
    assert len(FakeArtifacts.latest.summaries) == 1
    assert FakeArtifacts.latest.summaries[0]["episode_success"] is False
    assert FakeSession.latest.exited is True


@pytest.mark.parametrize(
    ("flags", "expected_fraction", "expected_longest", "expected_valid"),
    [
        ((), 0.0, 0.0, False),
        ((True, False), 0.5, 0.1, False),
        ((True, False, False, False), 0.25, 0.3, False),
        ((True, True), 1.0, 0.0, True),
    ],
)
def test_aggregate_observations_enforces_fraction_and_continuous_gap(
    flags, expected_fraction, expected_longest, expected_valid
) -> None:
    fraction, longest, valid = aggregate_observations(
        flags,
        sensor_tick_seconds=0.1,
        min_valid_fraction=0.995,
        max_continuous_invalid_seconds=0.2,
    )

    assert fraction == pytest.approx(expected_fraction)
    assert longest == pytest.approx(expected_longest)
    assert valid is expected_valid


@pytest.mark.parametrize(
    ("completed", "indices", "errors", "speeds"),
    [
        (False, (0, 1), (0.0, 0.0), (8.0, 8.0)),
        (True, (1, 0), (0.0, 0.0), (8.0, 8.0)),
        (True, (0, 1), (0.0, 2.1), (8.0, 8.0)),
        (True, (0, 1), (0.0, 0.0), (5.0, 5.0)),
    ],
)
def test_evaluate_target_execution_rejects_timeout_backward_deviation_and_speed(
    completed, indices, errors, speeds
) -> None:
    assert not evaluate_target_execution(
        completed=completed,
        route_indices=indices,
        cross_track_errors_m=errors,
        speeds_mps=speeds,
        max_cross_track_error_m=2.0,
        target_speed_mps=8.0,
        speed_tolerance_fraction=0.15,
    )
