import csv
import json

import yaml
from PIL import Image

from src.carla_experiments.storage import (
    FRAME_METRIC_FIELDS,
    TRAJECTORY_FIELDS,
    PilotArtifacts,
)

EXPERIMENT_ID = "20260826T120000Z-hover-seed20260826"


def test_pilot_artifacts_create_stable_layout_and_headers(tmp_path) -> None:
    artifacts = PilotArtifacts(
        tmp_path,
        EXPERIMENT_ID,
        resolved_config={"random_seed": 20260826, "policy": "hover"},
        episode_metadata={"map": "Town10HD_Opt", "policy": "hover"},
    )

    assert artifacts.run_directory == tmp_path / EXPERIMENT_ID
    assert (artifacts.run_directory / "config.resolved.yaml").is_file()
    assert (artifacts.run_directory / "episode.json").is_file()
    assert (artifacts.run_directory / "trajectory.csv").is_file()
    assert (artifacts.run_directory / "frame_metrics.csv").is_file()
    assert (artifacts.run_directory / "samples").is_dir()
    with (artifacts.run_directory / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
        assert tuple(next(csv.reader(stream))) == TRAJECTORY_FIELDS
    with (artifacts.run_directory / "frame_metrics.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        assert tuple(next(csv.reader(stream))) == FRAME_METRIC_FIELDS
    artifacts.finalize({"episode_success": True})


def test_pilot_artifacts_append_rows_and_finalize_atomically(tmp_path) -> None:
    artifacts = PilotArtifacts(
        tmp_path,
        EXPERIMENT_ID,
        resolved_config={"b": 2, "a": 1},
        episode_metadata={"policy": "hover"},
    )
    trajectory = dict.fromkeys(TRAJECTORY_FIELDS, 0)
    frame_metrics = dict.fromkeys(FRAME_METRIC_FIELDS, 0)
    frame_metrics["invalid_reasons"] = ("edge_margin", "distance")

    artifacts.append_trajectory(trajectory)
    artifacts.append_frame_metrics(frame_metrics)
    artifacts.finalize({"policy": "hover", "episode_success": True})

    resolved = yaml.safe_load(
        (artifacts.run_directory / "config.resolved.yaml").read_text(encoding="utf-8")
    )
    summary = json.loads(
        (artifacts.run_directory / "summary.json").read_text(encoding="utf-8")
    )
    with (artifacts.run_directory / "frame_metrics.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        row = next(csv.DictReader(stream))
    assert list(resolved) == ["b", "a"]
    assert summary == {"episode_success": True, "policy": "hover"}
    assert row["invalid_reasons"] == "edge_margin;distance"
    assert not (artifacts.run_directory / "summary.json.tmp").exists()


def test_save_sample_converts_carla_bgra_to_rgb_png(tmp_path) -> None:
    artifacts = PilotArtifacts(
        tmp_path,
        EXPERIMENT_ID,
        resolved_config={},
        episode_metadata={},
    )

    sample = artifacts.save_sample(12, bytes([3, 2, 1, 255]), width=1, height=1)
    artifacts.finalize({"episode_success": False})

    with Image.open(sample) as image:
        assert image.mode == "RGB"
        assert image.getpixel((0, 0)) == (1, 2, 3)
    assert sample.name == "00000012.png"
