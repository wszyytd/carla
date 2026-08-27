from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from ..config import RouteConfig


@dataclass(frozen=True)
class RouteCandidate:
    waypoints: tuple[Any, ...]
    road_id: int
    section_id: int
    lane_id: int
    start_s: float
    end_s: float
    length_m: float
    positive_turn_deg: float
    negative_turn_deg: float
    score: float


@dataclass(frozen=True)
class _TopologyPathSearchResult:
    candidates: list[RouteCandidate]
    seed_count: int
    completed_path_count: int


def wrapped_yaw_delta(previous_deg: float, current_deg: float) -> float:
    """Return the shortest signed rotation from ``previous`` to ``current``."""

    return (current_deg - previous_deg + 180.0) % 360.0 - 180.0


def _distance(first: Any, second: Any) -> float:
    return math.sqrt(
        (second.x - first.x) ** 2
        + (second.y - first.y) ** 2
        + (second.z - first.z) ** 2
    )


def _polyline_length(waypoints: tuple[Any, ...]) -> float:
    return sum(
        _distance(first.transform.location, second.transform.location)
        for first, second in zip(waypoints, waypoints[1:], strict=False)
    )


def score_waypoint_window(
    waypoints: tuple[Any, ...], *, min_turn_each_direction_deg: float
) -> RouteCandidate | None:
    """Score one same-lane window, rejecting non-S and junction windows."""

    if len(waypoints) < 2 or any(getattr(item, "is_junction", False) for item in waypoints):
        return None

    deltas = [
        wrapped_yaw_delta(
            first.transform.rotation.yaw,
            second.transform.rotation.yaw,
        )
        for first, second in zip(waypoints, waypoints[1:], strict=False)
    ]
    positive = sum(delta for delta in deltas if delta > 0.0)
    negative = -sum(delta for delta in deltas if delta < 0.0)
    if positive < min_turn_each_direction_deg or negative < min_turn_each_direction_deg:
        return None

    first = waypoints[0]
    last = waypoints[-1]
    score = min(positive, negative)
    return RouteCandidate(
        waypoints=waypoints,
        road_id=int(first.road_id),
        section_id=int(first.section_id),
        lane_id=int(first.lane_id),
        start_s=float(first.s),
        end_s=float(last.s),
        length_m=_polyline_length(waypoints),
        positive_turn_deg=positive,
        negative_turn_deg=negative,
        score=score,
    )


def _oriented_lane(waypoints: list[Any]) -> tuple[Any, ...]:
    ordered = sorted(waypoints, key=lambda item: float(item.s))
    if len(ordered) < 2:
        return tuple(ordered)

    first = ordered[0]
    second = ordered[1]
    yaw_rad = math.radians(float(first.transform.rotation.yaw))
    forward_x = math.cos(yaw_rad)
    forward_y = math.sin(yaw_rad)
    displacement_x = second.transform.location.x - first.transform.location.x
    displacement_y = second.transform.location.y - first.transform.location.y
    if forward_x * displacement_x + forward_y * displacement_y < 0.0:
        ordered.reverse()
    return tuple(ordered)


def _lane_windows(waypoints: tuple[Any, ...], minimum_length_m: float) -> Iterable[tuple[Any, ...]]:
    for start in range(len(waypoints) - 1):
        length = 0.0
        for end in range(start + 1, len(waypoints)):
            length += _distance(
                waypoints[end - 1].transform.location,
                waypoints[end].transform.location,
            )
            if length >= minimum_length_m:
                yield waypoints[start : end + 1]
                break


def score_lane_windows(
    waypoints: Iterable[Any],
    *,
    window_length_m: float,
    min_turn_each_direction_deg: float,
) -> list[RouteCandidate]:
    """Return deterministic S-like route candidates from generated waypoints."""

    lanes: dict[tuple[int, int, int], list[Any]] = defaultdict(list)
    for waypoint in waypoints:
        key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        lanes[key].append(waypoint)

    candidates: list[RouteCandidate] = []
    for lane_waypoints in lanes.values():
        for window in _lane_windows(_oriented_lane(lane_waypoints), window_length_m):
            candidate = score_waypoint_window(
                window,
                min_turn_each_direction_deg=min_turn_each_direction_deg,
            )
            if candidate is not None:
                candidates.append(candidate)

    return sorted(
        candidates,
        key=lambda item: (
            -item.score,
            item.road_id,
            item.section_id,
            item.lane_id,
            item.start_s,
        ),
    )


def _waypoint_identity(waypoint: Any) -> tuple[Any, ...]:
    waypoint_id = getattr(waypoint, "id", None)
    if waypoint_id is not None:
        return ("id", waypoint_id)

    location = waypoint.transform.location
    return (
        "metadata",
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
        float(waypoint.s),
        round(float(location.x), 3),
        round(float(location.y), 3),
        round(float(location.z), 3),
    )


def _successor_key(waypoint: Any) -> tuple[Any, ...]:
    location = waypoint.transform.location
    return (
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
        float(waypoint.s),
        round(float(location.x), 3),
        round(float(location.y), 3),
        round(float(location.z), 3),
        float(waypoint.transform.rotation.yaw),
        repr(_waypoint_identity(waypoint)),
    )


def _score_topology_paths_with_stats(
    seeds: Iterable[Any],
    *,
    step_distance_m: float,
    window_length_m: float,
    min_turn_each_direction_deg: float,
) -> _TopologyPathSearchResult:
    """Find S-like candidates and retain topology traversal diagnostics."""

    candidates: list[RouteCandidate] = []
    seen_path_identities: set[tuple[tuple[Any, ...], ...]] = set()
    seed_count = 0
    completed_path_count = 0
    for seed in seeds:
        seed_count += 1
        if getattr(seed, "is_junction", False):
            continue

        active_paths = [(seed,)]
        completed_paths: list[tuple[Any, ...]] = []
        while active_paths:
            path = active_paths.pop(0)
            path_identities = {_waypoint_identity(waypoint) for waypoint in path}
            for successor in sorted(
                path[-1].next(step_distance_m), key=_successor_key
            ):
                if getattr(successor, "is_junction", False):
                    continue

                successor_identity = _waypoint_identity(successor)
                if successor_identity in path_identities:
                    continue

                if len(active_paths) + len(completed_paths) >= 64:
                    break

                extended_path = (*path, successor)
                if _polyline_length(extended_path) >= window_length_m:
                    completed_paths.append(extended_path)
                    completed_path_count += 1
                else:
                    active_paths.append(extended_path)

        for path in completed_paths:
            candidate = score_waypoint_window(
                path,
                min_turn_each_direction_deg=min_turn_each_direction_deg,
            )
            if candidate is None:
                continue

            path_identity = tuple(_waypoint_identity(waypoint) for waypoint in path)
            if path_identity in seen_path_identities:
                continue
            seen_path_identities.add(path_identity)
            candidates.append(candidate)

    return _TopologyPathSearchResult(
        candidates=sorted(
            candidates,
            key=lambda item: (
                -item.score,
                item.road_id,
                item.section_id,
                item.lane_id,
                item.start_s,
            ),
        ),
        seed_count=seed_count,
        completed_path_count=completed_path_count,
    )


def score_topology_paths(
    seeds: Iterable[Any],
    *,
    step_distance_m: float,
    window_length_m: float,
    min_turn_each_direction_deg: float,
) -> list[RouteCandidate]:
    """Return deterministic S-like candidates found by forward topology traversal."""

    return _score_topology_paths_with_stats(
        seeds,
        step_distance_m=step_distance_m,
        window_length_m=window_length_m,
        min_turn_each_direction_deg=min_turn_each_direction_deg,
    ).candidates


def select_s_curve_route(map_obj: Any, config: RouteConfig) -> RouteCandidate:
    """Select a configured deterministic rank from one CARLA map."""

    seeds = map_obj.generate_waypoints(config.waypoint_spacing_m)
    search_result = _score_topology_paths_with_stats(
        seeds,
        step_distance_m=config.waypoint_spacing_m,
        window_length_m=config.window_length_m,
        min_turn_each_direction_deg=config.min_turn_each_direction_deg,
    )
    candidates = search_result.candidates
    if config.candidate_rank < len(candidates):
        return candidates[config.candidate_rank]

    raise ValueError(
        "no S-like driving-lane window found: "
        f"spacing={config.waypoint_spacing_m:.1f}m, "
        f"length={config.window_length_m:.1f}m, "
        f"min_turn={config.min_turn_each_direction_deg:.1f}deg, "
        f"seeds={search_result.seed_count}, "
        f"completed_paths={search_result.completed_path_count}"
    )
