from pathlib import Path

import pytest

from src.carla_experiments.config import load_config


def test_load_config_returns_mapping(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("host: localhost\nport: 2000\n", encoding="utf-8")

    assert load_config(path) == {"host": "localhost", "port": 2000}


def test_load_config_treats_empty_document_as_mapping(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")

    assert load_config(path) == {}


def test_load_config_rejects_non_mapping_root(tmp_path: Path) -> None:
    path = tmp_path / "list.yaml"
    path.write_text("- localhost\n- 2000\n", encoding="utf-8")

    with pytest.raises(ValueError, match="configuration root must be a mapping"):
        load_config(path)
