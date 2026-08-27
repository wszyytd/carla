"""Run identifiers, output directories, manifests, and atomic artifact writes."""

from __future__ import annotations

import csv
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any, TextIO

import numpy as np
import yaml
from PIL import Image

TRAJECTORY_FIELDS = (
    "frame",
    "sim_time_s",
    "target_x",
    "target_y",
    "target_z",
    "target_speed_mps",
    "uav_x",
    "uav_y",
    "uav_z",
    "uav_vx",
    "uav_vy",
    "uav_vz",
    "uav_ax",
    "uav_ay",
    "uav_az",
    "gimbal_pitch",
    "gimbal_yaw",
    "policy",
)

FRAME_METRIC_FIELDS = (
    "frame",
    "sim_time_s",
    "sensor_frame",
    "bbox_x_min",
    "bbox_y_min",
    "bbox_x_max",
    "bbox_y_max",
    "bbox_short_side_px",
    "instance_pixels",
    "center_error_fraction",
    "distance_m",
    "speed_mps",
    "acceleration_mps2",
    "jerk_mps3",
    "gimbal_rate_deg_s",
    "valid",
    "invalid_reasons",
)


class PilotArtifacts:
    """Incrementally write one reproducible path-cost episode."""

    def __init__(
        self,
        output_root: str | Path,
        experiment_id: str,
        *,
        resolved_config: Mapping[str, Any],
        episode_metadata: Mapping[str, Any],
    ) -> None:
        self.run_directory = Path(output_root) / experiment_id
        self.run_directory.mkdir(parents=True, exist_ok=False)
        self.samples_directory = self.run_directory / "samples"
        self.samples_directory.mkdir()
        self._write_yaml("config.resolved.yaml", resolved_config)
        self._write_json("episode.json", episode_metadata)

        self._trajectory_stream = self._open_csv("trajectory.csv")
        self._trajectory_writer = csv.DictWriter(
            self._trajectory_stream,
            fieldnames=TRAJECTORY_FIELDS,
        )
        self._trajectory_writer.writeheader()
        self._trajectory_stream.flush()

        self._frame_metrics_stream = self._open_csv("frame_metrics.csv")
        self._frame_metrics_writer = csv.DictWriter(
            self._frame_metrics_stream,
            fieldnames=FRAME_METRIC_FIELDS,
        )
        self._frame_metrics_writer.writeheader()
        self._frame_metrics_stream.flush()
        self._finalized = False

    def _open_csv(self, name: str) -> TextIO:
        return (self.run_directory / name).open("w", encoding="utf-8", newline="")

    def _write_yaml(self, name: str, value: Mapping[str, Any]) -> None:
        with (self.run_directory / name).open("w", encoding="utf-8") as stream:
            yaml.safe_dump(dict(value), stream, sort_keys=False, allow_unicode=True)

    def _write_json(self, name: str, value: Mapping[str, Any]) -> None:
        with (self.run_directory / name).open("w", encoding="utf-8") as stream:
            json.dump(
                dict(value),
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            stream.write("\n")

    def append_trajectory(self, row: Mapping[str, Any]) -> None:
        self._trajectory_writer.writerow(dict(row))
        self._trajectory_stream.flush()

    def append_frame_metrics(self, row: Mapping[str, Any]) -> None:
        serialized = dict(row)
        invalid_reasons = serialized.get("invalid_reasons")
        if isinstance(invalid_reasons, (tuple, list)):
            serialized["invalid_reasons"] = ";".join(map(str, invalid_reasons))
        self._frame_metrics_writer.writerow(serialized)
        self._frame_metrics_stream.flush()

    def save_sample(self, frame: int, raw: bytes, *, width: int, height: int) -> Path:
        expected_length = width * height * 4
        if len(raw) != expected_length:
            raise ValueError(
                f"raw image length must be {expected_length} bytes, received {len(raw)}"
            )
        bgra = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
        rgb = bgra[:, :, [2, 1, 0]]
        path = self.samples_directory / f"{frame:08d}.png"
        Image.fromarray(rgb).save(path)
        return path

    def finalize(self, summary: Mapping[str, Any]) -> Path:
        if self._finalized:
            raise RuntimeError("pilot artifacts already finalized")
        self._trajectory_stream.flush()
        self._frame_metrics_stream.flush()
        self._trajectory_stream.close()
        self._frame_metrics_stream.close()
        temporary = self.run_directory / "summary.json.tmp"
        final = self.run_directory / "summary.json"
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(
                dict(summary),
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            stream.write("\n")
        temporary.replace(final)
        self._finalized = True
        return final
