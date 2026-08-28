from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from src.carla_experiments.trajectories.s_curve import (
    RouteCandidate,
    score_lane_windows,
    score_topology_paths,
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


@dataclass
class TopologyWaypoint:
    road_id: int
    section_id: int
    lane_id: int
    s: float
    transform: Transform
    id: int
    successors: list["TopologyWaypoint"]
    is_junction: bool = False

    def next(self, distance: float) -> list["TopologyWaypoint"]:
        return self.successors


@dataclass
class FailingJunctionTopologyWaypoint(TopologyWaypoint):
    def next(self, distance: float) -> list["TopologyWaypoint"]:
        raise AssertionError("topology traversal must not expand a junction")


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


def test_score_waypoint_window_scores_total_turn_and_rejects_below_threshold() -> None:
    candidate = score_waypoint_window(
        waypoints([0.0, 10.0, 20.0, 10.0, 0.0]),
        min_total_turn_deg=40.0,
    )

    assert candidate is not None
    assert candidate.positive_turn_deg == pytest.approx(20.0)
    assert candidate.negative_turn_deg == pytest.approx(20.0)
    assert candidate.score == pytest.approx(40.0)
    assert score_waypoint_window(
        waypoints([0.0, 10.0, 20.0, 30.0]),
        min_total_turn_deg=40.0,
    ) is None


def test_score_waypoint_window_accepts_one_direction_total_turn_threshold() -> None:
    candidate = score_waypoint_window(
        waypoints([0.0, 20.0, 40.0, 60.0, 80.0]),
        min_total_turn_deg=60.0,
    )

    assert candidate is not None
    assert candidate.positive_turn_deg == pytest.approx(80.0)
    assert candidate.negative_turn_deg == pytest.approx(0.0)
    assert candidate.score == pytest.approx(80.0)
    assert score_waypoint_window(
        waypoints([0.0, 10.0, 20.0, 30.0, 40.0]),
        min_total_turn_deg=60.0,
    ) is None


def test_wrapped_yaw_delta_uses_shortest_signed_rotation() -> None:
    assert wrapped_yaw_delta(179.0, -179.0) == pytest.approx(2.0)
    assert wrapped_yaw_delta(-179.0, 179.0) == pytest.approx(-2.0)


def test_score_lane_windows_rejects_windows_that_enter_junctions() -> None:
    candidates = score_lane_windows(
        waypoints([0.0, 10.0, 20.0, 10.0, 0.0], junction_index=2),
        window_length_m=8.0,
        min_total_turn_deg=8.0,
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
        min_total_turn_deg=5.0,
    )

    assert [(item.score, item.road_id) for item in candidates] == [
        (40.0, 1),
        (40.0, 2),
        (24.0, 3),
    ]


def test_select_s_curve_route_uses_rank_and_reports_empty_search() -> None:
    def topology_chain(road_id: int, first_id: int) -> list[TopologyWaypoint]:
        chain = [
            TopologyWaypoint(
                road_id=road_id,
                section_id=0,
                lane_id=1,
                s=float(index * 2),
                transform=Transform(Location(float(index * 2)), Rotation(yaw)),
                id=first_id + index,
                successors=[],
            )
            for index, yaw in enumerate([0.0, 10.0, 20.0, 10.0, 0.0])
        ]
        for waypoint, successor in zip(chain, chain[1:], strict=False):
            waypoint.successors.append(successor)
        return chain

    route_two = topology_chain(2, 10)
    route_one = topology_chain(1, 20)
    map_obj = SimpleNamespace(
        generate_waypoints=lambda spacing: (route_two[0], route_one[0])
    )
    config = SimpleNamespace(
        waypoint_spacing_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=8.0,
        candidate_rank=1,
    )

    selected = select_s_curve_route(map_obj, config)

    assert isinstance(selected, RouteCandidate)
    assert selected.road_id == 2

    empty_map = SimpleNamespace(generate_waypoints=lambda spacing: ())
    error_config = SimpleNamespace(
        waypoint_spacing_m=2.0,
        window_length_m=120.0,
        min_total_turn_deg=8.0,
        candidate_rank=0,
    )
    with pytest.raises(
        ValueError,
        match=(
            r"no curved driving-lane window found: spacing=2\.0m, "
            r"length=120\.0m, min_total_turn=8\.0deg, "
            r"seeds=0, completed_paths=0"
        ),
    ):
        select_s_curve_route(empty_map, error_config)


def test_select_s_curve_route_reports_completed_non_s_paths() -> None:
    chain = [
        TopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=float(index * 2),
            transform=Transform(
                Location(float(index * 2)), Rotation(float(index * 10))
            ),
            id=index,
            successors=[],
        )
        for index in range(5)
    ]
    for waypoint, successor in zip(chain, chain[1:], strict=False):
        waypoint.successors.append(successor)

    map_obj = SimpleNamespace(generate_waypoints=lambda spacing: (chain[0],))
    config = SimpleNamespace(
        waypoint_spacing_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=60.0,
        candidate_rank=0,
    )

    with pytest.raises(
        ValueError,
        match=(
            r"no curved driving-lane window found: spacing=2\.0m, "
            r"length=8\.0m, min_total_turn=60\.0deg, "
            r"seeds=1, completed_paths=1"
        ),
    ):
        select_s_curve_route(map_obj, config)


def test_select_s_curve_route_follows_topology_across_road_boundaries() -> None:
    chain = [
        TopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=0.0,
            transform=Transform(Location(0.0), Rotation(0.0)),
            id=0,
            successors=[],
        ),
        TopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=2.0,
            transform=Transform(Location(2.0), Rotation(10.0)),
            id=1,
            successors=[],
        ),
        TopologyWaypoint(
            road_id=2,
            section_id=0,
            lane_id=1,
            s=0.0,
            transform=Transform(Location(4.0), Rotation(20.0)),
            id=2,
            successors=[],
        ),
        TopologyWaypoint(
            road_id=2,
            section_id=1,
            lane_id=1,
            s=2.0,
            transform=Transform(Location(6.0), Rotation(10.0)),
            id=3,
            successors=[],
        ),
        TopologyWaypoint(
            road_id=3,
            section_id=1,
            lane_id=1,
            s=0.0,
            transform=Transform(Location(8.0), Rotation(0.0)),
            id=4,
            successors=[],
        ),
    ]
    for waypoint, successor in zip(chain, chain[1:], strict=False):
        waypoint.successors.append(successor)

    map_obj = SimpleNamespace(generate_waypoints=lambda spacing: (chain[0],))
    config = SimpleNamespace(
        waypoint_spacing_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=8.0,
        candidate_rank=0,
    )

    selected = select_s_curve_route(map_obj, config)

    assert all(
        actual is expected
        for actual, expected in zip(selected.waypoints, chain, strict=True)
    )


def test_score_topology_paths_sorts_reverse_successors_with_heterogeneous_ids() -> None:
    seed = TopologyWaypoint(
        road_id=0,
        section_id=0,
        lane_id=1,
        s=0.0,
        transform=Transform(Location(0.0), Rotation(0.0)),
        id=0,
        successors=[],
    )

    def branch(ids: tuple[object, ...]) -> list[TopologyWaypoint]:
        nodes = [
            TopologyWaypoint(
                road_id=1,
                section_id=0,
                lane_id=1,
                s=float((index + 1) * 2),
                transform=Transform(Location(float((index + 1) * 2)), Rotation(yaw)),
                id=ids[index],
                successors=[],
            )
            for index, yaw in enumerate([10.0, 20.0, 10.0, 0.0])
        ]
        for waypoint, successor in zip(nodes, nodes[1:], strict=False):
            waypoint.successors.append(successor)
        return nodes

    string_id_branch = branch(("branch-a", "branch-a-2", "branch-a-3", "branch-a-4"))
    integer_id_branch = branch((2, 22, 23, 24))
    seed.successors.extend((integer_id_branch[0], string_id_branch[0]))

    candidates = score_topology_paths(
        (seed,),
        step_distance_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=8.0,
    )

    assert [candidate.waypoints[1].id for candidate in candidates] == ["branch-a", 2]


def test_score_topology_paths_rejects_chain_that_enters_junction() -> None:
    chain: list[TopologyWaypoint] = [
        TopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=0.0,
            transform=Transform(Location(0.0), Rotation(0.0)),
            id=0,
            successors=[],
        ),
        TopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=2.0,
            transform=Transform(Location(2.0), Rotation(10.0)),
            id=1,
            successors=[],
        ),
        FailingJunctionTopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=4.0,
            transform=Transform(Location(4.0), Rotation(20.0)),
            id=2,
            successors=[],
            is_junction=True,
        ),
    ]
    for waypoint, successor in zip(chain, chain[1:], strict=False):
        waypoint.successors.append(successor)

    candidates = score_topology_paths(
        (chain[0],),
        step_distance_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=8.0,
    )

    assert candidates == []


def test_score_topology_paths_terminates_on_cycle_without_complete_path() -> None:
    cycle = [
        TopologyWaypoint(
            road_id=1,
            section_id=0,
            lane_id=1,
            s=float(index * 2),
            transform=Transform(Location(float(index * 2)), Rotation(yaw)),
            id=index,
            successors=[],
        )
        for index, yaw in enumerate([0.0, 10.0, 20.0])
    ]
    for waypoint, successor in zip(cycle, (*cycle[1:], cycle[0]), strict=True):
        waypoint.successors.append(successor)

    candidates = score_topology_paths(
        (cycle[0],),
        step_distance_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=8.0,
    )

    assert candidates == []


def test_score_topology_paths_retains_at_most_64_paths_per_seed() -> None:
    seed = TopologyWaypoint(
        road_id=1,
        section_id=0,
        lane_id=1,
        s=0.0,
        transform=Transform(Location(0.0), Rotation(0.0)),
        id=0,
        successors=[],
    )
    completed_path = TopologyWaypoint(
        road_id=2,
        section_id=0,
        lane_id=1,
        s=0.0,
        transform=Transform(Location(2.0), Rotation(0.0)),
        id=100,
        successors=[],
    )
    active_paths = [
        TopologyWaypoint(
            road_id=2,
            section_id=0,
            lane_id=1,
            s=1.0,
            transform=Transform(Location(1.0), Rotation(0.0)),
            id=index,
            successors=[],
        )
        for index in range(1, 65)
    ]
    for waypoint in active_paths:
        waypoint.successors.append(
            TopologyWaypoint(
                road_id=2,
                section_id=0,
                lane_id=1,
                s=2.0,
                transform=Transform(Location(2.0), Rotation(0.0)),
                id=waypoint.id + 100,
                successors=[],
            )
        )
    seed.successors.extend((completed_path, *active_paths))

    candidates = score_topology_paths(
        (seed,),
        step_distance_m=2.0,
        window_length_m=2.0,
        min_total_turn_deg=0.0,
    )

    assert len(candidates) == 64


def test_score_topology_paths_eliminates_duplicate_path_identities() -> None:
    def chain() -> list[TopologyWaypoint]:
        nodes = [
            TopologyWaypoint(
                road_id=1,
                section_id=0,
                lane_id=1,
                s=float(index * 2),
                transform=Transform(Location(float(index * 2)), Rotation(yaw)),
                id=index,
                successors=[],
            )
            for index, yaw in enumerate([0.0, 10.0, 20.0, 10.0, 0.0])
        ]
        for waypoint, successor in zip(nodes, nodes[1:], strict=False):
            waypoint.successors.append(successor)
        return nodes

    first = chain()
    duplicate = chain()

    candidates = score_topology_paths(
        (first[0], duplicate[0]),
        step_distance_m=2.0,
        window_length_m=8.0,
        min_total_turn_deg=8.0,
    )

    assert len(candidates) == 1
