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


def select_s_curve_route(map_obj: Any, config: RouteConfig) -> RouteCandidate:
    """Select a configured deterministic rank from one CARLA map."""

    candidates = score_lane_windows(
        map_obj.generate_waypoints(config.waypoint_spacing_m),
        window_length_m=config.window_length_m,
        min_turn_each_direction_deg=config.min_turn_each_direction_deg,
    )
    if config.candidate_rank < len(candidates):
        return candidates[config.candidate_rank]

    raise ValueError(
        "no S-like driving-lane window found: "
        f"spacing={config.waypoint_spacing_m:.1f}m, "
        f"length={config.window_length_m:.1f}m, "
        f"min_turn={config.min_turn_each_direction_deg:.1f}deg"
    )
