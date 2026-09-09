from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from src.carla_experiments.runtime import OwnedActors, SynchronousSession


@dataclass
class Settings:
    synchronous_mode: bool
    fixed_delta_seconds: float | None


class FakeTrafficManager:
    def __init__(self) -> None:
        self.synchronous_calls: list[bool] = []
        self.seed_calls: list[int] = []

    def set_random_device_seed(self, seed: int) -> None:
        self.seed_calls.append(seed)

    def set_synchronous_mode(self, enabled: bool) -> None:
        self.synchronous_calls.append(enabled)


class FakeWorld:
    def __init__(self, original: Settings) -> None:
        self.original = original
        self.applied_settings: list[Settings] = []
        self.tick_calls = 0

    def get_settings(self) -> Settings:
        if not self.applied_settings:
            if not hasattr(self, "_returned_original"):
                self._returned_original = True
                return self.original
            return Settings(
                synchronous_mode=self.original.synchronous_mode,
                fixed_delta_seconds=self.original.fixed_delta_seconds,
            )
        return self.applied_settings[-1]

    def apply_settings(self, settings: Settings) -> None:
        self.applied_settings.append(settings)

    def tick(self) -> int:
        self.tick_calls += 1
        return 100 + self.tick_calls


class FakeClient:
    def __init__(self, world: FakeWorld, traffic_manager: FakeTrafficManager) -> None:
        self.world = world
        self.traffic_manager = traffic_manager
        self.requested_port: int | None = None

    def get_world(self) -> FakeWorld:
        return self.world

    def get_trafficmanager(self, port: int) -> FakeTrafficManager:
        self.requested_port = port
        return self.traffic_manager


def config() -> SimpleNamespace:
    return SimpleNamespace(
        world=SimpleNamespace(fixed_delta_seconds=0.05),
        traffic_manager=SimpleNamespace(port=8000, random_seed=20260826),
    )


def test_synchronous_session_restores_settings_after_body_error() -> None:
    original = Settings(synchronous_mode=False, fixed_delta_seconds=None)
    world = FakeWorld(original)
    traffic_manager = FakeTrafficManager()
    client = FakeClient(world, traffic_manager)

    with pytest.raises(RuntimeError, match="boom"):
        with SynchronousSession(client, config()):
            applied = world.applied_settings[-1]
            assert applied.synchronous_mode is True
            assert applied.fixed_delta_seconds == 0.05
            raise RuntimeError("boom")

    assert world.applied_settings[-1] is original
    assert traffic_manager.synchronous_calls == [True, False]
    assert traffic_manager.seed_calls == [20260826]
    assert client.requested_port == 8000


class FakeActor:
    def __init__(self, actor_id: str, events: list[str], *, sensor: bool = False) -> None:
        self.id = actor_id
        self.events = events
        self.sensor = sensor

    def stop(self) -> None:
        if not self.sensor:
            raise AttributeError("not a sensor")
        self.events.append(f"stop:{self.id}")

    def destroy(self) -> None:
        self.events.append(f"destroy:{self.id}")


class FakeVehicle:
    def __init__(self, actor_id: str, events: list[str]) -> None:
        self.id = actor_id
        self.events = events

    def destroy(self) -> None:
        self.events.append(f"destroy:{self.id}")


def test_owned_actors_stop_sensors_then_destroy_everything_in_reverse_order() -> None:
    events: list[str] = []
    actors = OwnedActors()
    camera_a = actors.add(FakeActor("A", events, sensor=True))
    camera_b = actors.add(FakeActor("B", events, sensor=True))
    vehicle = actors.add(FakeVehicle("C", events))

    assert (camera_a.id, camera_b.id, vehicle.id) == ("A", "B", "C")
    assert actors.add(None) is None
    assert actors.destroy_all() == ()
    assert events == ["stop:B", "stop:A", "destroy:C", "destroy:B", "destroy:A"]
    assert actors.destroy_all() == ()


def test_owned_actors_reports_before_and_after_each_cleanup_call() -> None:
    events: list[str] = []
    messages: list[str] = []
    actors = OwnedActors()
    actors.add(FakeActor("A", events, sensor=True))
    actors.add(FakeActor("B", events, sensor=True))
    actors.add(FakeVehicle("C", events))

    actors.destroy_all(progress=messages.append)

    assert messages == [
        "清理：停止 actor[1] 前",
        "清理：停止 actor[1] 后",
        "清理：停止 actor[0] 前",
        "清理：停止 actor[0] 后",
        "清理：销毁 actor[2] 前",
        "清理：销毁 actor[2] 后",
        "清理：销毁 actor[1] 前",
        "清理：销毁 actor[1] 后",
        "清理：销毁 actor[0] 前",
        "清理：销毁 actor[0] 后",
    ]
    assert events == ["stop:B", "stop:A", "destroy:C", "destroy:B", "destroy:A"]


def test_session_reports_cleanup_brackets_without_reordering_native_calls() -> None:
    original = Settings(False, None)
    world = FakeWorld(original)
    traffic_manager = FakeTrafficManager()
    messages: list[str] = []

    with SynchronousSession(FakeClient(world, traffic_manager), config()) as session:
        session.set_progress_reporter(messages.append)

    assert messages == [
        "会话清理开始",
        "清理：恢复 Traffic Manager 异步模式 前",
        "清理：恢复 Traffic Manager 异步模式 后",
        "清理：恢复世界设置 前",
        "清理：恢复世界设置 后",
        "会话清理完成",
    ]
    assert traffic_manager.synchronous_calls == [True, False]
    assert world.applied_settings[-1] is original


def test_session_tick_calls_world_once_and_records_cleanup_failures() -> None:
    original = Settings(False, None)
    world = FakeWorld(original)
    traffic_manager = FakeTrafficManager()

    with SynchronousSession(FakeClient(world, traffic_manager), config()) as session:
        assert session.tick() == 101
        assert world.tick_calls == 1
        broken = SimpleNamespace(
            id=77,
            stop=lambda: (_ for _ in ()).throw(RuntimeError("stop failed")),
            destroy=lambda: (_ for _ in ()).throw(RuntimeError("destroy failed")),
        )
        session.actors.add(broken)

    assert [(item.actor_id, item.operation) for item in session.cleanup_failures] == [
        (77, "stop"),
        (77, "destroy"),
    ]
