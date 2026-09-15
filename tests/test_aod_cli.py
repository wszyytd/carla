from __future__ import annotations

import importlib
import json
import sys

import yaml


def test_prepare_and_validate_need_no_carla(tmp_path, monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "carla", None)
    module = importlib.import_module("src.aod")
    inventory = {
        "schema_version": 1,
        "map": "Town10HD_Opt",
        "client": {"host": "localhost", "port": 2000, "timeout_seconds": 1},
        "vehicles": [{"id": "vehicle.a", "number_of_wheels": 4}],
        "spawn_points": [{"index": 2, "pose": [1, 2, 3, 0, 0, 0]}],
    }
    source, output = tmp_path / "inventory.json", tmp_path / "preview.yaml"
    source.write_text(json.dumps(inventory))
    assert (
        module.main(
            ["prepare", "--inventory", str(source), "--spawn-index", "2", "--output", str(output)]
        )
        == 0
    )
    assert yaml.safe_load(output.read_text())["scene"]["target_pose"][:3] == [1, 2, 3]
    assert module.main(["validate", "--config", str(output)]) == 0
    assert "24" in capsys.readouterr().out
    assert (
        module.main(
            ["prepare", "--inventory", str(source), "--spawn-index", "2", "--output", str(output)]
        )
        != 0
    )


def test_sensor_timeout_returns_runtime_error_code(tmp_path, monkeypatch):
    from tests.test_aod_capture import World, config, fake_carla

    module = importlib.import_module("src.aod")
    world = World()

    def timeout(*args):
        raise TimeoutError("sensor/RPC timed out")

    world.tick = timeout
    monkeypatch.setitem(sys.modules, "carla", fake_carla(world))
    source = tmp_path / "config.yaml"
    source.write_text(yaml.safe_dump(config().resolved))
    assert (
        module.main(["capture", "--config", str(source), "--output", str(tmp_path / "preview")])
        == 3
    )
    assert not world.settings.synchronous_mode
