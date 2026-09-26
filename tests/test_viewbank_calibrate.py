"""Exposure calibration reuses capture; candidates never replace production config."""

import copy
import importlib
import json

import pytest

from tests.test_viewbank_capture import CaptureWorld, simulator
from tests.test_viewbank_config import raw_config


def api():
    return importlib.import_module("src.carla_experiments.viewbank.calibrate")


def test_candidates_keep_camera_and_two_identical_adjacent_poses():
    cfg = raw_config()
    original = copy.deepcopy(cfg)
    candidates = api().calibration_configs(cfg)
    assert cfg == original
    assert [c["case_id"] for c in candidates] == ["baseline", "ue5_low", "ue5_default", "ue5_high"]
    from src.carla_experiments.viewbank.grid import build_grid

    poses = []
    for c in candidates:
        nodes, edges = build_grid(c["capture_config"])
        assert len(nodes) == 2 and len(edges) == 2
        poses.append([n["requested_transform"] for n in nodes])
        assert c["capture_config"]["quality"] == cfg["quality"]
        assert c["full_config"]["grid"] == cfg["grid"]
        assert c["capture_config"]["camera"]["width"] == cfg["camera"]["width"]
        assert c["capture_config"]["camera"]["exposure"]["exposure_mode"] == "manual"
    assert all(p == poses[0] for p in poses)
    assert candidates[2]["capture_config"]["camera"]["exposure"]["iso"] == 300000
    assert candidates[2]["capture_config"]["camera"]["exposure"]["shutter_speed"] == 15


def test_calibration_runs_real_fake_capture_and_exports_reviewable_configs(tmp_path):
    cfg = raw_config()
    root = tmp_path / "calibration"
    result = api().calibrate(simulator(CaptureWorld()), cfg, root)
    assert result["status"] == "diagnostic_complete"
    assert len(result["cases"]) == 4
    assert result["selection"] is None
    for case in result["cases"]:
        assert case["capture"]["passed"]
        assert len(case["images"]) == 2
        assert all(0 < im["rgb_mean"] < 255 for im in case["images"])
        assert (root / case["full_config_path"]).is_file()
        assert all((root / im["rgb_path"]).is_file() for im in case["images"])
    assert (root / "report.html").is_file()
    assert json.loads((root / "report.json").read_text())["selection"] is None
    with pytest.raises(FileExistsError):
        api().calibrate(simulator(CaptureWorld()), cfg, root)


def test_runtime_failure_stops_remaining_cases_and_saves_report(tmp_path, monkeypatch):
    calls = []

    def failed(*args, **kwargs):
        calls.append(1)
        return {"exit_code": 130, "passed": False, "interrupted": True, "errors": ["interrupted"]}

    monkeypatch.setattr(api(), "capture", failed)
    result = api().calibrate(None, raw_config(), tmp_path / "interrupt")
    assert result["exit_code"] == 130
    assert len(calls) == 1
    assert result["status"] == "interrupted"
    assert (tmp_path / "interrupt/report.json").is_file()


def test_partial_frame_diagnostics_do_not_hide_capture_failure(tmp_path):
    from PIL import Image

    folder = tmp_path / "cases/baseline/frames/.n_0000.tmp"
    folder.mkdir(parents=True)
    Image.new("RGB", (8, 6)).save(folder / "rgb.png")
    result = api().image_diagnostics(tmp_path, tmp_path / "cases/baseline")
    assert result[0]["accepted"] is False
    assert "diagnostic_error" in result[0]


def test_rejected_frames_are_visible_only_as_diagnostics(tmp_path, monkeypatch):
    from PIL import Image

    from src.carla_experiments.viewbank.artifacts import write_json

    def black_capture(carla, cfg, root):
        folder = root / "frames/.n_0000.tmp"
        folder.mkdir(parents=True)
        Image.new("RGB", (8, 6)).save(folder / "rgb.png")
        write_json(
            folder / "camera.json", {"node_id": "n_0000", "frame": 1, "actual_transform": [0] * 6}
        )
        return {"exit_code": 5, "passed": False, "errors": ["dark/overexposed frame"]}

    monkeypatch.setattr(api(), "capture", black_capture)
    result = api().calibrate(None, raw_config(), tmp_path / "black")
    assert result["exit_code"] == 5 and result["selection"] is None
    assert len(result["cases"]) == 4
    for case in result["cases"]:
        image = case["images"][0]
        assert image["rgb_mean"] == 0 and image["near_black_fraction"] == 1
        assert image["accepted"] is False
