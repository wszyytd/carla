"""Scale calibration retains physical image scale and legacy configurations."""

import copy
import importlib
import json
import sys

import numpy as np
import pytest
import yaml
from PIL import Image

from src.carla_experiments.aod.config import parse_preview_config, prepare_config
from src.carla_experiments.aod.preview import preview_views
from src.carla_experiments.metrics import ProjectedBox
from tests.test_aod_capture import World, config, fake_carla
from tests.test_aod_config import inventory


def test_new_defaults_and_old_config_are_distinct_and_resolved():
    raw = prepare_config(inventory(), spawn_index=0)
    assert raw["view_grid"] == {"heights_m": [60, 100, 140], "horizontal_offset_m": 60}
    new = parse_preview_config(raw)
    assert new.heights_m == (60, 100, 140) and new.horizontal_offset_m == 60
    legacy = copy.deepcopy(raw)
    del legacy["view_grid"]
    before = copy.deepcopy(legacy)
    old = parse_preview_config(legacy)
    assert old.heights_m == (20, 30, 40) and old.horizontal_offset_m == 10
    assert old.resolved["view_grid"]["heights_m"] == [20, 30, 40]
    assert legacy == before


@pytest.mark.parametrize(
    "grid",
    [
        {"heights_m": [60, 60, 140], "horizontal_offset_m": 60},
        {"heights_m": [140, 100, 60], "horizontal_offset_m": 60},
        {"heights_m": [60, 100], "horizontal_offset_m": 60},
        {"heights_m": [True, 100, 140], "horizontal_offset_m": 60},
        {"heights_m": [60, 100, float("nan")], "horizontal_offset_m": 60},
        {"heights_m": [60, 100, 140], "horizontal_offset_m": 0},
        {"heights_m": [60, 100, 140], "horizontal_offset_m": float("inf")},
    ],
)
def test_invalid_grid_fails_before_capture(grid):
    raw = prepare_config(inventory(), spawn_index=0)
    raw["view_grid"] = grid
    with pytest.raises(ValueError):
        parse_preview_config(raw)


def test_custom_grid_coordinates_and_height_keys():
    views = preview_views(
        (100, 200, 5), ground_z=3, heights_m=(60, 100, 140), horizontal_offset_m=60
    )
    assert len(views) == 24
    assert (views[0].x, views[0].y, views[0].z, views[0].key) == (160, 200, 63, "h60_p1")
    assert (views[7].x, views[7].y, views[7].z) == (40, 140, 63)
    assert views[16].z == 143 and views[16].key == "h140_p1"


def scale(box, width=1920, height=1080):
    return importlib.import_module("src.carla_experiments.aod.scale").measure_scale(
        box, width=width, height=height
    )


def test_scale_measures_full_box_and_window_truncation():
    info = scale(ProjectedBox(700, 300, 1100, 400, True))
    assert info["width_px"] == 400 and info["height_px"] == 100
    assert info["box_area_over_crop_area"] == pytest.approx(40000 / 90000)
    assert info["fraction_retained"] == pytest.approx(0.75)
    assert info["crop_truncated"] and not info["image_truncated"]
    assert info["band"] == "above_reference"


def test_image_edge_loss_is_not_mislabeled_as_crop_size_problem():
    info = scale(ProjectedBox(-20, 50, 80, 100, True))
    assert info["image_truncated"] and not info["crop_truncated"]
    assert info["fraction_retained"] == pytest.approx(0.8)
    assert info["band"] == "within_reference"


def test_invalid_projection_does_not_become_a_zero_pixel_target():
    info = scale(ProjectedBox(float("nan"), 0, 1, 1, False))
    assert info["long_side_px"] is None
    assert info["band"] == "unknown"


def test_capture_v2_has_configured_poses_scale_report_and_unscaled_sheets(tmp_path):
    capture = importlib.import_module("src.carla_experiments.aod.capture").capture
    raw = config().resolved
    raw["view_grid"] = {"heights_m": [60, 100, 140], "horizontal_offset_m": 60}
    root = tmp_path / "v2"
    result = capture(fake_carla(World()), parse_preview_config(raw), root)
    assert result["captured"] == 24
    nodes = [json.loads(line) for line in (root / "nodes.jsonl").read_text().splitlines()]
    assert nodes[0]["rgb_pose"][:3] == [60, 0, 60]
    assert nodes[-1]["rgb_pose"][:3] == [-60, -60, 140]
    report = json.loads((root / "scale_report.json").read_text())
    assert report["overall"]["captured"] == 24
    assert [g["height_m"] for g in report["groups"]] == [60, 100, 140]
    assert "scale" in nodes[0]
    with Image.open(root / "crop_sheet_00.png") as sheet:
        # First image is pasted unscaled at (6, 40), preserving even the black padding.
        patch = np.asarray(sheet.crop((6, 40, 306, 340)))
    with Image.open(root / nodes[0]["paths"]["crop"]) as original:
        assert np.array_equal(patch, np.asarray(original))
    checker = importlib.import_module("src.carla_experiments.aod.artifacts").check_artifacts
    assert checker(root)["status"] == "complete"


def test_prepare_cli_accepts_explicit_grid_without_carla(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "carla", None)
    cli = importlib.import_module("src.aod")
    source, destination = tmp_path / "inventory.json", tmp_path / "v2.yaml"
    source.write_text(json.dumps(inventory()))
    assert (
        cli.main(
            [
                "prepare",
                "--inventory",
                str(source),
                "--spawn-index",
                "0",
                "--heights",
                "70",
                "110",
                "150",
                "--horizontal-offset",
                "65",
                "--output",
                str(destination),
            ]
        )
        == 0
    )
    raw = yaml.safe_load(destination.read_text())
    assert raw["view_grid"]["heights_m"] == [70, 110, 150]
    assert raw["view_grid"]["horizontal_offset_m"] == 65


def test_compare_keeps_sources_and_refuses_changed_camera(tmp_path, monkeypatch):
    capture = importlib.import_module("src.carla_experiments.aod.capture").capture
    before, after, output = (tmp_path / name for name in ("before", "after", "compare"))
    old = config().resolved
    old["view_grid"] = {"heights_m": [20, 30, 40], "horizontal_offset_m": 10}
    new = copy.deepcopy(old)
    new["view_grid"] = {"heights_m": [60, 100, 140], "horizontal_offset_m": 60}
    capture(fake_carla(World()), parse_preview_config(old), before)
    capture(fake_carla(World()), parse_preview_config(new), after)
    # Recreate the old v1 metadata contract: no grid or saved scale measurements.
    from src.carla_experiments.aod.artifacts import digest, write_json

    legacy_config = json.loads((before / "config.json").read_text())
    legacy_config.pop("view_grid")
    write_json(before / "config.json", legacy_config)
    legacy_nodes = [json.loads(line) for line in (before / "nodes.jsonl").read_text().splitlines()]
    for node in legacy_nodes:
        node.pop("scale")
    (before / "nodes.jsonl").write_text(
        "".join(json.dumps(node) + "\n" for node in legacy_nodes), encoding="utf-8"
    )
    legacy_summary = json.loads((before / "summary.json").read_text())
    legacy_summary.pop("scale_overall")
    legacy_summary.pop("scale_report")
    write_json(before / "summary.json", legacy_summary)
    hashes = json.loads((before / "checksums.json").read_text())
    for name in ("config.json", "nodes.jsonl", "summary.json"):
        hashes[name] = digest(before / name)
    write_json(before / "checksums.json", hashes)
    snapshot = (before / "checksums.json").read_bytes()
    monkeypatch.setitem(sys.modules, "carla", None)
    cli = importlib.import_module("src.aod")
    assert (
        cli.main(
            ["compare", "--before", str(before), "--after", str(after), "--output", str(output)]
        )
        == 0
    )
    assert (output / "compare_00.png").exists()
    report = json.loads((output / "comparison.json").read_text())
    assert report["before"]["overall"]["captured"] == 24
    assert report["after"]["overall"]["captured"] == 24
    assert (before / "checksums.json").read_bytes() == snapshot
    assert (
        cli.main(
            ["compare", "--before", str(before), "--after", str(after), "--output", str(output)]
        )
        != 0
    )
    changed = copy.deepcopy(new)
    changed["camera"]["fov_deg"] = 90
    other = tmp_path / "other"
    capture(fake_carla(World()), parse_preview_config(changed), other)
    assert (
        cli.main(
            [
                "compare",
                "--before",
                str(before),
                "--after",
                str(other),
                "--output",
                str(tmp_path / "invalid-comparison"),
            ]
        )
        != 0
    )
