from types import SimpleNamespace

import pytest

from src.carla_experiments.client import WorldSummary, inspect_world, versions_compatible


class FakeActors:
    def __init__(self) -> None:
        self.filter_patterns: list[str] = []
        self.all_actors = [object() for _ in range(5)]

    def filter(self, pattern: str) -> list[object]:
        self.filter_patterns.append(pattern)
        counts = {"vehicle.*": 2, "walker.pedestrian.*": 1}
        return [object() for _ in range(counts[pattern])]

    def __len__(self) -> int:
        return len(self.all_actors)


class FakeWorld:
    def __init__(self) -> None:
        self.actors = FakeActors()

    def get_map(self) -> SimpleNamespace:
        return SimpleNamespace(name="Carla/Maps/Town10HD_Opt")

    def get_settings(self) -> SimpleNamespace:
        return SimpleNamespace(synchronous_mode=False)

    def get_actors(self) -> FakeActors:
        return self.actors


class FakeClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.timeout: float | None = None
        self.world = FakeWorld()

    def set_timeout(self, timeout: float) -> None:
        self.timeout = timeout

    def get_client_version(self) -> str:
        return "0.10.0"

    def get_server_version(self) -> str:
        return "0.10.1"

    def get_world(self) -> FakeWorld:
        return self.world


class FakeCarlaModule:
    def __init__(self) -> None:
        self.clients: list[FakeClient] = []

    def Client(self, host: str, port: int) -> FakeClient:
        client = FakeClient(host, port)
        self.clients.append(client)
        return client


def test_inspect_world_returns_read_only_summary() -> None:
    module = FakeCarlaModule()
    clock = iter([10.0, 10.25]).__next__

    summary = inspect_world(module, "localhost", 2000, 10.0, clock)

    assert summary == WorldSummary(
        client_version="0.10.0",
        server_version="0.10.1",
        map_name="Carla/Maps/Town10HD_Opt",
        synchronous_mode=False,
        vehicle_count=2,
        walker_count=1,
        actor_count=5,
        elapsed_seconds=0.25,
    )
    assert len(module.clients) == 1
    assert (module.clients[0].host, module.clients[0].port) == ("localhost", 2000)
    assert module.clients[0].timeout == 10.0
    assert module.clients[0].world.actors.filter_patterns == [
        "vehicle.*",
        "walker.pedestrian.*",
    ]


@pytest.mark.parametrize(
    ("client_version", "server_version"),
    [("0.10.0", "0.10.1"), ("0.10.0-dirty", "0.10.9")],
)
def test_versions_compatible_accepts_matching_major_minor(
    client_version: str, server_version: str
) -> None:
    assert versions_compatible(client_version, server_version)


def test_versions_compatible_rejects_different_major_minor() -> None:
    assert not versions_compatible("0.9.16", "0.10.0")


@pytest.mark.parametrize("version", ["", "development", "0", "0.x.1"])
def test_versions_compatible_rejects_malformed_version(version: str) -> None:
    with pytest.raises(ValueError, match="invalid CARLA version"):
        versions_compatible(version, "0.10.0")
