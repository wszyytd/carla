from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_config(path: str | Path) -> dict[str, Any]:
    """Load one YAML configuration whose root is a mapping."""

    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)

    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value
