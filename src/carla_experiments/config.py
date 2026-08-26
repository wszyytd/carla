from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ClientConfig:
    """Validated settings for one CARLA client connection."""

    host: str
    port: int
    timeout_seconds: float


def load_config(path: str | Path) -> dict[str, Any]:
    """Load one YAML configuration whose root is a mapping."""

    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)

    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value


def parse_client_config(config: Mapping[str, Any]) -> ClientConfig:
    """Validate and normalize the ``client`` section of a configuration."""

    client = config.get("client")
    if not isinstance(client, Mapping):
        raise ValueError("client must be a mapping")

    host = client.get("host")
    if not isinstance(host, str) or not host.strip():
        raise ValueError("client.host must be a non-empty string")

    port = client.get("port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("client.port must be an integer from 1 to 65535")

    timeout = client.get("timeout_seconds")
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or not math.isfinite(timeout)
        or timeout <= 0
    ):
        raise ValueError("client.timeout_seconds must be a positive finite number")

    return ClientConfig(host=host.strip(), port=port, timeout_seconds=float(timeout))
