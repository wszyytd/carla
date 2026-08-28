from types import SimpleNamespace

import pytest

from src.carla_experiments.actors import configure_target_path, spawn_target_vehicle
from src.carla_experiments.runtime import OwnedActors


class Attribute:
    def __init__(self, value: int) -> None:
        self.value = value

    def as_int(self) -> int:
        return self.value


class Blueprint:
    def __init__(self, blueprint_id: str, wheels: int) -> None:
        self.id = blueprint_id
        self.attributes = {
            "number_of_wheels": Attribute(wheels),
            "role_name": Attribute(0),
        }
        self.set_calls: list[tuple[str, str]] = []

    def has_attribute(self, name: str) -> bool:
        return name in self.attributes

    def get_attribute(self, name: str) -> Attribute:
        return self.attributes[name]

    def set_attribute(self, name: str, value: str) -> None:
        self.set_calls.append((name, value))


class BlueprintLibrary:
    def __init__(self, blueprints: list[Blueprint]) -> None:
        self.blueprints = blueprints
        self.filter_calls: list[str] = []

    def filter(self, pattern: str) -> list[Blueprint]:
        self.filter_calls.append(pattern)
        return self.blueprints


class World:
    def __init__(self, blueprints: list[Blueprint], vehicle: object | None = None) -> None:
        self.library = BlueprintLibrary(blueprints)
        self.vehicle = vehicle if vehicle is not None else SimpleNamespace(id=42)
        self.spawn_calls: list[tuple[Blueprint, object]] = []

    def get_blueprint_library(self) -> BlueprintLibrary:
        return self.library

    def try_spawn_actor(self, blueprint: Blueprint, transform: object) -> object | None:
        self.spawn_calls.append((blueprint, transform))
        return self.vehicle


def route(count: int = 3) -> SimpleNamespace:
    points = tuple(
        SimpleNamespace(transform=SimpleNamespace(location=f"location-{index}"))
        for index in range(count)
    )
    return SimpleNamespace(
        waypoints=points,
        road_id=7,
        section_id=2,
        lane_id=-1,
        start_s=14.0,
    )


def test_spawn_target_vehicle_selects_first_sorted_four_wheel_blueprint() -> None:
    blueprints = [
        Blueprint("vehicle.z", 4),
        Blueprint("vehicle.a", 4),
        Blueprint("vehicle.two_wheel", 2),
    ]
    world = World(blueprints)
    actors = OwnedActors()

    vehicle = spawn_target_vehicle(
        world,
        route(),
        SimpleNamespace(blueprint_filter="vehicle.*"),
        actors,
    )

    assert vehicle.id == 42
    assert world.library.filter_calls == ["vehicle.*"]
    assert world.spawn_calls[0][0].id == "vehicle.a"
    assert world.spawn_calls[0][0].set_calls == [("role_name", "path_cost_target")]
    assert actors.destroy_all() == ()


def test_spawn_target_vehicle_rejects_empty_eligible_set() -> None:
    world = World([Blueprint("vehicle.two_wheel", 2)])

    with pytest.raises(RuntimeError, match=r"target\.blueprint_filter"):
        spawn_target_vehicle(
            world,
            route(),
            SimpleNamespace(blueprint_filter="vehicle.*"),
            OwnedActors(),
        )


def test_spawn_target_vehicle_reports_route_when_spawn_fails() -> None:
    world = World([Blueprint("vehicle.a", 4)], vehicle=None)
    world.vehicle = None

    with pytest.raises(
        RuntimeError,
        match=r"road=7, section=2, lane=-1, start_s=14\.0",
    ):
        spawn_target_vehicle(
            world,
            route(),
            SimpleNamespace(blueprint_filter="vehicle.*"),
            OwnedActors(),
        )


class Vehicle:
    def __init__(self, events: list[tuple[object, ...]]) -> None:
        self.events = events

    def set_autopilot(self, enabled: bool, port: int) -> None:
        self.events.append(("autopilot", enabled, port))


class TrafficManager:
    def __init__(self, events: list[tuple[object, ...]]) -> None:
        self.events = events

    def auto_lane_change(self, vehicle: Vehicle, enabled: bool) -> None:
        self.events.append(("auto_lane_change", enabled))

    def random_left_lanechange_percentage(self, vehicle: Vehicle, percentage: float) -> None:
        self.events.append(("left_lanechange", percentage))

    def random_right_lanechange_percentage(self, vehicle: Vehicle, percentage: float) -> None:
        self.events.append(("right_lanechange", percentage))

    def set_desired_speed(self, vehicle: Vehicle, speed: float) -> None:
        self.events.append(("desired_speed", speed))

    def set_path(self, vehicle: Vehicle, locations: list[object]) -> None:
        self.events.append(("path", locations))


def test_configure_target_path_calls_traffic_manager_in_exact_order() -> None:
    events: list[tuple[object, ...]] = []

    configure_target_path(
        Vehicle(events),
        route(),
        TrafficManager(events),
        traffic_manager_port=8000,
        target_speed_mps=8.0,
    )

    assert events == [
        ("autopilot", True, 8000),
        ("auto_lane_change", False),
        ("left_lanechange", 0.0),
        ("right_lanechange", 0.0),
        ("desired_speed", 28.8),
        ("path", ["location-1", "location-2"]),
    ]


def test_configure_target_path_rejects_short_route_before_autopilot() -> None:
    events: list[tuple[object, ...]] = []

    with pytest.raises(ValueError, match="at least two waypoints"):
        configure_target_path(
            Vehicle(events),
            route(count=1),
            TrafficManager(events),
            traffic_manager_port=8000,
            target_speed_mps=8.0,
        )

    assert events == []
