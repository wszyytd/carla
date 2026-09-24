import copy
import importlib
from pathlib import Path

import pytest
import yaml


def raw_config():
    return {
        "schema_version": 1,
        "scene_id": "test_scene",
        "client": {"host": "localhost", "port": 2000, "timeout_seconds": 2},
        "scene": {
            "map": "Carla/Maps/Town10HD_Opt",
            "seed": 7,
            "fixed_delta_seconds": 0.05,
            "weather": "ClearNoon",
            "roi_min_m": [-30, -30, -5],
            "roi_max_m": [30, 30, 50],
        },
        "grid": {
            "origin_m": [0, 0, 0],
            "x_offsets_m": [-20, 0, 20],
            "y_offsets_m": [-20, 0, 20],
            "z_world_m": [20, 40],
            "yaw_deg": [0, 90, 180, 270],
            "pitch_deg": -45,
        },
        "camera": {
            "width": 8,
            "height": 6,
            "fov_deg": 91.6,
            "instance": True,
            "exposure": {
                "exposure_mode": "manual",
                "iso": 100,
                "shutter_speed": 200,
                "fstop": 2.8,
                "exposure_compensation": 0,
                "motion_blur_intensity": 0,
            },
        },
        "graph": {
            "clearance_m": 1,
            "edge_step_m": 1,
            "start_index": [1, 1, 0, 0],
            "translation_cost_per_m": 1,
            "rotation_cost_per_deg": 0.01,
        },
        "capture": {"warmup_seconds": 0.8, "timeout_seconds": 2, "max_frame_ticks": 80},
        "quality": {
            "position_error_m": 0.1,
            "angle_error_deg": 0.1,
            "timestamp_error_seconds": 0.000001,
            "stability_threshold": 1,
            "min_finite_depth_fraction": 1,
            "min_valid_depth_fraction": 0.5,
            "depth_min_m": 0.1,
            "depth_max_m": 999,
            "require_connected": True,
        },
    }


def api():
    return importlib.import_module("src.carla_experiments.viewbank.config")


def test_config_normalization_hash_and_no_mutation():
    raw = raw_config()
    before = copy.deepcopy(raw)
    cfg = api().parse_config(raw)
    assert raw == before
    assert api().config_hash(cfg) == api().config_hash(api().parse_config(before))
    assert cfg["grid"]["z_world_m"] == [20, 40]


@pytest.mark.parametrize(
    "section,key,value",
    [
        (None, "typo", 1),
        ("camera", "typo", 1),
        ("camera", "width", True),
        ("grid", "x_offsets_m", [0, 0]),
        ("grid", "yaw_deg", [0, 360]),
        ("grid", "z_world_m", [float("nan")]),
        ("scene", "seed", -1),
        ("graph", "edge_step_m", 0),
        ("graph", "start_index", [9, 0, 0, 0]),
        ("quality", "min_finite_depth_fraction", 1.1),
        ("camera", "instance", "false"),
        (None, "scene_id", "../escape"),
    ],
)
def test_rejects_invalid_fields(section, key, value):
    raw = raw_config()
    (raw if section is None else raw[section])[key] = value
    with pytest.raises(ValueError):
        api().parse_config(raw)


def test_shipped_configs_have_72_and_18_nodes():
    from src.carla_experiments.viewbank.grid import build_grid

    for name, count in [("town10_aod_probe", 72), ("town10_aod_smoke", 18)]:
        raw = yaml.safe_load(Path(f"cfg/viewbank/{name}.yaml").read_text(encoding="utf-8"))
        cfg = api().parse_config(raw)
        assert len(build_grid(cfg)[0]) == count
