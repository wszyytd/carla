"""Target and background actor creation, selection, tracking, and cleanup."""

from __future__ import annotations

from typing import Any

from .config import TargetConfig
from .runtime import OwnedActors
from .trajectories.s_curve import RouteCandidate


def _wheel_count(blueprint: Any) -> int | None:
    if not blueprint.has_attribute("number_of_wheels"):
        return None
    attribute = blueprint.get_attribute("number_of_wheels")
    as_int = getattr(attribute, "as_int", None)
    if callable(as_int):
        return int(as_int())
    try:
        return int(str(attribute))
    except (TypeError, ValueError):
        return None


def spawn_target_vehicle(
    world: Any,
    route: RouteCandidate,
    config: TargetConfig,
    actors: OwnedActors,
) -> Any:
    """Deterministically choose and spawn one four-wheel target vehicle."""

    library = world.get_blueprint_library()
    eligible = sorted(
        (
            blueprint
            for blueprint in library.filter(config.blueprint_filter)
            if _wheel_count(blueprint) == 4
        ),
        key=lambda blueprint: blueprint.id,
    )
    if not eligible:
        raise RuntimeError(
            "no eligible four-wheel blueprint matched "
            f"target.blueprint_filter={config.blueprint_filter!r}"
        )

    blueprint = eligible[0]
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "path_cost_target")
    vehicle = world.try_spawn_actor(blueprint, route.waypoints[0].transform)
    if vehicle is None:
        raise RuntimeError(
            "failed to spawn target vehicle at "
            f"road={route.road_id}, section={route.section_id}, lane={route.lane_id}, "
            f"start_s={route.start_s:.1f}"
        )
    return actors.add(vehicle)


def configure_target_path(
    vehicle: Any,
    route: RouteCandidate,
    traffic_manager: Any,
    *,
    traffic_manager_port: int,
    target_speed_mps: float,
) -> None:
    """Enable autopilot and constrain the target to the selected lane path."""

    if len(route.waypoints) < 2:
        raise ValueError("target route must contain at least two waypoints")

    vehicle.set_autopilot(True, traffic_manager_port)
    traffic_manager.auto_lane_change(vehicle, False)
    traffic_manager.random_left_lanechange_percentage(vehicle, 0.0)
    traffic_manager.random_right_lanechange_percentage(vehicle, 0.0)
    traffic_manager.set_desired_speed(vehicle, target_speed_mps)
    traffic_manager.set_path(
        vehicle,
        [waypoint.transform.location for waypoint in route.waypoints[1:]],
    )
