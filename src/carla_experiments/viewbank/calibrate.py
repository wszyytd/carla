"""Diagnostic exposure comparison; reuse the complete capture lifecycle and quality gates."""

import copy
import html
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from .artifacts import read_json, write_json
from .capture import capture, output_lock
from .config import config_hash, parse_config

SOURCE = (
    "https://github.com/carla-simulator/carla/blob/0.10.0/Unreal/CarlaUnreal/"
    "Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp"
)


def calibration_configs(cfg):
    """Same start and one adjacent position, with one manual exposure shared per case."""
    cfg = parse_config(cfg)
    grid = cfg["grid"]
    keys = ("x_offsets_m", "y_offsets_m", "z_world_m", "yaw_deg")
    start = cfg["graph"]["start_index"]
    axis = next((i for i, key in enumerate(keys[:3]) if len(grid[key]) > 1), None)
    if axis is None:
        raise ValueError("calibration requires two adjacent spatial grid positions")
    cases = []
    for name, compensation in (
        ("baseline", None),
        ("ue5_low", -0.5),
        ("ue5_default", 1.5),
        ("ue5_high", 3.5),
    ):
        full = copy.deepcopy(cfg)
        full["scene_id"] = cfg["scene_id"][:60] + "_" + name
        if compensation is not None:
            full["camera"]["exposure"].update(
                iso=300000.0,
                shutter_speed=15.0,
                fstop=9.8,
                exposure_compensation=compensation,
            )
        small = copy.deepcopy(full)
        small["scene_id"] += "_calibration"
        small["graph"]["start_index"] = [0, 0, 0, 0]
        for i, key in enumerate(keys):
            indices = [start[i]]
            if i == axis:
                neighbor = start[i] + 1 if start[i] + 1 < len(grid[key]) else start[i] - 1
                indices = sorted([start[i], neighbor])
                small["graph"]["start_index"][i] = indices.index(start[i])
            small["grid"][key] = [grid[key][j] for j in indices]
        cases.append(
            {
                "case_id": name,
                "full_config": parse_config(full),
                "capture_config": parse_config(small),
            }
        )
    return cases


def image_diagnostics(root, case_root):
    """Read even rejected temporary RGBs as diagnostics, never as accepted observations."""
    images = []
    for folder in sorted((case_root / "frames").glob("*")):
        if not (folder / "rgb.png").is_file():
            continue
        try:
            with Image.open(folder / "rgb.png") as image:
                rgb = np.asarray(image.convert("RGB"))
            camera = read_json(folder / "camera.json")
            intensity = rgb.mean(axis=2)
            images.append(
                {
                    "node_id": camera["node_id"],
                    "rgb_path": (folder / "rgb.png").relative_to(root).as_posix(),
                    "camera_path": (folder / "camera.json").relative_to(root).as_posix(),
                    "accepted": (folder / "receipt.json").is_file(),
                    "frame": camera["frame"],
                    "actual_transform": camera["actual_transform"],
                    "rgb_mean": float(rgb.mean()),
                    "rgb_p05_p50_p95": [float(v) for v in np.percentile(intensity, [5, 50, 95])],
                    "near_black_fraction": float((rgb.max(axis=2) < 5).mean()),
                    "clipped_channel_fraction": float((rgb >= 250).mean()),
                }
            )
        except (OSError, ValueError, KeyError, TypeError) as error:
            images.append(
                {
                    "rgb_path": (folder / "rgb.png").relative_to(root).as_posix(),
                    "accepted": False,
                    "diagnostic_error": f"{type(error).__name__}: {error}",
                }
            )
    return images


def save_report(root, report):
    write_json(root / "report.json", report)
    parts = [
        "<!doctype html><meta charset='utf-8'><title>Exposure calibration</title>",
        "<style>body{font:16px sans-serif;margin:24px}section{margin:24px 0}"
        "img{width:456px;max-width:100%;image-rendering:auto}figure{display:inline-block;"
        "vertical-align:top;margin:8px}pre{white-space:pre-wrap}</style>",
        "<h1>Diagnostic exposure comparison</h1><p>Manual review required. "
        "No automatic selection. Rejected images are diagnostic only.</p>",
    ]
    for case in report["cases"]:
        parts.append("<section><h2>" + html.escape(case["case_id"]) + "</h2>")
        parts.append("<pre>" + html.escape(str(case["exposure"])) + "</pre>")
        for image in case["images"]:
            parts.append(
                "<figure><img src='"
                + html.escape(image["rgb_path"], quote=True)
                + "'><figcaption>"
                + html.escape(
                    str({k: v for k, v in image.items() if k not in ("rgb_path", "camera_path")})
                )
                + "</figcaption></figure>"
            )
        parts.append("<pre>" + html.escape(str(case["capture"])) + "</pre></section>")
    (root / "report.html").write_text("\n".join(parts), encoding="utf-8")


def calibrate(carla, cfg, output):
    """Four two-node captures; no configuration selection or production-data overwrite."""
    cfg = parse_config(cfg)
    candidates = calibration_configs(cfg)
    root = Path(output)
    with output_lock(root):
        root.mkdir(parents=True, exist_ok=False)
        (root / "configs").mkdir()
        report = {
            "schema_version": 1,
            "kind": "exposure_calibration_diagnostic",
            "source_config_sha256": config_hash(cfg),
            "candidate_basis": SOURCE,
            "status": "running",
            "selection": None,
            "passed": False,
            "scope": "two shared viewpoints only; no full-bank exposure acceptance",
            "cases": [],
            "exit_code": 5,
        }
        write_json(root / "source.config.json", cfg)
        save_report(root, report)
        for candidate in candidates:
            name = candidate["case_id"]
            path = f"configs/{name}.yaml"
            (root / path).write_text(
                yaml.safe_dump(candidate["full_config"], sort_keys=False), encoding="utf-8"
            )
            case_root = root / "cases" / name
            result = capture(carla, candidate["capture_config"], case_root)
            report["cases"].append(
                {
                    "case_id": name,
                    "exposure": candidate["full_config"]["camera"]["exposure"],
                    "full_config_path": path,
                    "capture": result,
                    "images": image_diagnostics(root, case_root),
                }
            )
            save_report(root, report)
            if result["exit_code"] in (3, 130):
                report.update(
                    status="interrupted" if result["exit_code"] == 130 else "runtime_failed",
                    exit_code=result["exit_code"],
                )
                break
        else:
            passed = any(c["capture"]["passed"] for c in report["cases"])
            report.update(status="diagnostic_complete", passed=passed, exit_code=0 if passed else 5)
        save_report(root, report)
        return report
