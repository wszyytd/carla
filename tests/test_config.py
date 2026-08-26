from pathlib import Path

import pytest

from src.carla_experiments.config import ClientConfig, load_config, parse_client_config


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


def test_parse_client_config_returns_normalized_values() -> None:
    assert parse_client_config(
        {"client": {"host": "localhost", "port": 2000, "timeout_seconds": 10}}
    ) == ClientConfig("localhost", 2000, 10.0)


@pytest.mark.parametrize("value", [None, [], "localhost"])
def test_parse_client_config_requires_client_mapping(value: object) -> None:
    with pytest.raises(ValueError, match=r"client must be a mapping"):
        parse_client_config({"client": value})


@pytest.mark.parametrize("value", ["", "   ", 123, None])
def test_parse_client_config_rejects_invalid_host(value: object) -> None:
    with pytest.raises(ValueError, match=r"client\.host"):
        parse_client_config(
            {"client": {"host": value, "port": 2000, "timeout_seconds": 10}}
        )


@pytest.mark.parametrize("value", [True, 0, 65536, 2000.5, "2000"])
def test_parse_client_config_rejects_invalid_port(value: object) -> None:
    with pytest.raises(ValueError, match=r"client\.port"):
        parse_client_config(
            {"client": {"host": "localhost", "port": value, "timeout_seconds": 10}}
        )


@pytest.mark.parametrize("value", [True, 0, -1, float("inf"), "10"])
def test_parse_client_config_rejects_invalid_timeout(value: object) -> None:
    with pytest.raises(ValueError, match=r"client\.timeout_seconds"):
        parse_client_config(
            {"client": {"host": "localhost", "port": 2000, "timeout_seconds": value}}
        )
