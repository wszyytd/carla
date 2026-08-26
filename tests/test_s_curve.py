from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from src.carla_experiments.trajectories.s_curve import (
    RouteCandidate,
    score_lane_windows,
    score_waypoint_window,
    select_s_curve_route,
    wrapped_yaw_delta,
)


@dataclass(frozen=True)
class Location:
    x: float
    y: float = 0.0
    z: float = 0.0


@dataclass(frozen=True)
class Rotation:
    yaw: float


@dataclass(frozen=True)
class Transform:
    location: Location
    rotation: Rotation


@dataclass(frozen=True)
class Waypoint:
    road_id: int
    section_id: int
    lane_id: int
    s: float
    transform: Transform
    is_junction: bool = False


def waypoints(
    yaws: list[float],
    *,
    road_id: int = 1,
    section_id: int = 0,
    lane_id: int = 1,
    junction_index: int | None = None,
) -> tuple[Waypoint, ...]:
    return tuple(
        Waypoint(
            road_id=road_id,
            section_id=section_id,
            lane_id=lane_id,
            s=float(index * 2),
            transform=Transform(Location(float(index * 2)), Rotation(yaw)),
            is_junction=index == junction_index,
        )
        for index, yaw in enumerate(yaws)
    )


def test_score_waypoint_window_requires_turns_in_both_directions() -> None:
    candidate = score_waypoint_window(
        waypoints([0.0, 10.0, 20.0, 10.0, 0.0]),
        min_turn_each_direction_deg=8.0,
    )

    assert candidate is not None
    assert candidate.positive_turn_deg == pytest.approx(20.0)
    assert candidate.negative_turn_deg == pytest.approx(20.0)
    assert candidate.score == pytest.approx(20.0)
    assert score_waypoint_window(
        waypoints([0.0, 10.0, 20.0, 30.0]),
        min_turn_each_direction_deg=8.0,
    ) is None


def test_wrapped_yaw_delta_uses_shortest_signed_rotation() -> None:
    assert wrapped_yaw_delta(179.0, -179.0) == pytest.approx(2.0)
    assert wrapped_yaw_delta(-179.0, 179.0) == pytest.approx(-2.0)


def test_score_lane_windows_rejects_windows_that_enter_junctions() -> None:
    candidates = score_lane_windows(
        waypoints([0.0, 10.0, 20.0, 10.0, 0.0], junction_index=2),
        window_length_m=8.0,
        min_turn_each_direction_deg=8.0,
    )

    assert candidates == []


def test_score_lane_windows_orders_score_then_lane_metadata() -> None:
    candidates = score_lane_windows(
        (
            *waypoints([0.0, 10.0, 20.0, 10.0, 0.0], road_id=2),
            *waypoints([0.0, 10.0, 20.0, 10.0, 0.0], road_id=1),
            *waypoints([0.0, 6.0, 12.0, 6.0, 0.0], road_id=3),
        ),
        window_length_m=8.0,
        min_turn_each_direction_deg=5.0,
    )

    assert [(item.score, item.road_id) for item in candidates] == [
        (20.0, 1),
        (20.0, 2),
        (12.0, 3),
    ]


def test_select_s_curve_route_uses_rank_and_reports_empty_search() -> None:
    generated = (
        *waypoints([0.0, 10.0, 20.0, 10.0, 0.0], road_id=2),
        *waypoints([0.0, 10.0, 20.0, 10.0, 0.0], road_id=1),
    )
    map_obj = SimpleNamespace(generate_waypoints=lambda spacing: generated)
    config = SimpleNamespace(
        waypoint_spacing_m=2.0,
        window_length_m=8.0,
        min_turn_each_direction_deg=8.0,
        candidate_rank=1,
    )

    selected = select_s_curve_route(map_obj, config)

    assert isinstance(selected, RouteCandidate)
    assert selected.road_id == 2

    empty_map = SimpleNamespace(generate_waypoints=lambda spacing: ())
    error_config = SimpleNamespace(
        waypoint_spacing_m=2.0,
        window_length_m=120.0,
        min_turn_each_direction_deg=8.0,
        candidate_rank=0,
    )
    with pytest.raises(
        ValueError,
        match=(
            r"no S-like driving-lane window found: spacing=2\.0m, "
            r"length=120\.0m, min_turn=8\.0deg"
        ),
    ):
        select_s_curve_route(empty_map, error_config)
