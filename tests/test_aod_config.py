import copy
import importlib

import pytest


def api():
    return importlib.import_module("src.carla_experiments.aod.config")


def inventory():
    return {
        "schema_version": 1,
        "client": {"host": "localhost", "port": 2000, "timeout_seconds": 10},
        "map": "/Game/Carla/Maps/Town10HD_Opt",
        "client_version": "0.10.0",
        "server_version": "0.10.0",
        "vehicles": [
            {"id": name, "number_of_wheels": 4} for name in ("vehicle.c", "vehicle.a", "vehicle.b")
        ],
        "spawn_points": [{"index": 0, "pose": [100, 200, 0.5, 0, 90, 0]}],
    }


def test_prepare_uses_actual_inventory_assets_and_stable_order():
    result = api().prepare_config(inventory(), spawn_index=0)
    assert result["scene"]["blueprint_ids"] == ["vehicle.a", "vehicle.b", "vehicle.c"]
    assert result["scene"]["target_pose"] == [100, 200, 0.5, 0, 90, 0]
    assert result["scene"]["ground_z"] == 0.5
    assert result["scene"]["ground_reference"] == "spawn_point_unverified"
    parsed = api().parse_preview_config(result)
    assert parsed.camera.width == 1920


def test_prepare_rejects_nonexistent_assets_and_spawn_points():
    with pytest.raises(ValueError, match="blueprint"):
        api().prepare_config(inventory(), spawn_index=0, blueprint_ids=["vehicle.unknown"])
    with pytest.raises(ValueError, match="spawn"):
        api().prepare_config(inventory(), spawn_index=100)


@pytest.mark.parametrize("value", [float("nan"), -1, 0, True])
def test_invalid_capture_timeout_rejected_before_server_connection(value):
    data = api().prepare_config(inventory(), spawn_index=0)
    data["capture"]["timeout_seconds"] = value
    with pytest.raises(ValueError):
        api().parse_preview_config(data)


def test_duplicate_classes_cannot_overwrite_capture_outputs():
    data = api().prepare_config(inventory(), spawn_index=0)
    data["scene"]["blueprint_ids"] = ["vehicle.a", "vehicle.a"]
    with pytest.raises(ValueError, match="unique"):
        api().parse_preview_config(data)


def test_config_rejects_sensor_period_not_aligned_with_fixed_step():
    data = api().prepare_config(inventory(), spawn_index=0)
    data["camera"]["sensor_tick_seconds"] = 0.075
    with pytest.raises(ValueError, match="multiple"):
        api().parse_preview_config(data)


def test_input_config_is_not_mutated():
    data = api().prepare_config(inventory(), spawn_index=0)
    before = copy.deepcopy(data)
    api().parse_preview_config(data)
    assert data == before
