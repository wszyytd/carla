from __future__ import annotations

import re
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WorldSummary:
    """Read-only summary of one connected CARLA world."""

    client_version: str
    server_version: str
    map_name: str
    synchronous_mode: bool
    vehicle_count: int
    walker_count: int
    actor_count: int
    elapsed_seconds: float


def inspect_world(
    carla_module: Any,
    host: str,
    port: int,
    timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> WorldSummary:
    """Connect to CARLA and read world state without modifying the simulation."""

    started_at = clock()
    client = carla_module.Client(host, port)
    client.set_timeout(timeout_seconds)
    client_version = str(client.get_client_version())
    server_version = str(client.get_server_version())
    world = client.get_world()
    actors = world.get_actors()

    summary = WorldSummary(
        client_version=client_version,
        server_version=server_version,
        map_name=str(world.get_map().name),
        synchronous_mode=bool(world.get_settings().synchronous_mode),
        vehicle_count=len(actors.filter("vehicle.*")),
        walker_count=len(actors.filter("walker.pedestrian.*")),
        actor_count=len(actors),
        elapsed_seconds=0.0,
    )
    elapsed_seconds = clock() - started_at
    return WorldSummary(
        client_version=summary.client_version,
        server_version=summary.server_version,
        map_name=summary.map_name,
        synchronous_mode=summary.synchronous_mode,
        vehicle_count=summary.vehicle_count,
        walker_count=summary.walker_count,
        actor_count=summary.actor_count,
        elapsed_seconds=elapsed_seconds,
    )


def versions_compatible(client_version: str, server_version: str) -> bool:
    """Return whether CARLA client and server share major/minor versions."""

    return _major_minor(client_version) == _major_minor(server_version)


def _major_minor(version: str) -> tuple[int, int]:
    match = re.match(r"^(\d+)\.(\d+)(?:[.-].*)?$", version)
    if match is None:
        raise ValueError(f"invalid CARLA version: {version}")
    return int(match.group(1)), int(match.group(2))
