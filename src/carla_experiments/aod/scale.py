"""Pixel-scale diagnostics; reference bands never filter captured observations."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from ..metrics import ProjectedBox

REFERENCE_LONG_SIDE = (40, 120)


def measure_scale(box, *, width, height, crop_size=300):
    if min(width, height, crop_size) <= 0:
        raise ValueError("image and crop dimensions must be positive")
    values = (box.x_min, box.y_min, box.x_max, box.y_max)
    valid = (
        box.in_front
        and all(math.isfinite(v) for v in values)
        and box.x_min < box.x_max
        and box.y_min < box.y_max
    )
    result = dict.fromkeys(
        (
            "width_px",
            "height_px",
            "long_side_px",
            "box_area_over_crop_area",
            "fraction_retained",
            "crop_truncated",
            "image_truncated",
        )
    )
    result.update(projection_valid=valid, band="unknown")
    if not valid:
        return result
    w, h = box.x_max - box.x_min, box.y_max - box.y_min
    left = math.floor((box.x_min + box.x_max) / 2 - crop_size / 2)
    top = math.floor((box.y_min + box.y_max) / 2 - crop_size / 2)
    right, bottom = left + crop_size, top + crop_size
    retained_width = max(0, min(box.x_max, width, right) - max(box.x_min, 0, left))
    retained_height = max(0, min(box.y_max, height, bottom) - max(box.y_min, 0, top))
    long_side = max(w, h)
    low, high = REFERENCE_LONG_SIDE
    band = (
        "below_reference"
        if long_side < low
        else "above_reference"
        if long_side > high
        else "within_reference"
    )
    result.update(
        width_px=w,
        height_px=h,
        long_side_px=long_side,
        box_area_over_crop_area=w * h / crop_size**2,
        fraction_retained=retained_width * retained_height / (w * h),
        crop_truncated=(
            box.x_min < left or box.y_min < top or box.x_max > right or box.y_max > bottom
        ),
        image_truncated=(box.x_min < 0 or box.y_min < 0 or box.x_max > width or box.y_max > height),
        band=band,
    )
    return result


def node_scale(node):
    if "scale" in node:
        return node["scale"]
    bounds = node.get("projected_box")
    box = ProjectedBox(*bounds, True) if bounds is not None else ProjectedBox(0, 0, 0, 0, False)
    width, height = node["image_size"]
    return measure_scale(box, width=width, height=height)


def _summarize(nodes, planned):
    captured = [n for n in nodes if n["status"] == "captured"]
    measurements = [node_scale(n) for n in captured]
    valid = [s for s in measurements if s["projection_valid"]]
    lengths = [s["long_side_px"] for s in valid]
    return {
        "planned": planned,
        "captured": len(captured),
        "rejected": sum(n["status"] == "rejected" for n in nodes),
        "not_recorded": planned - len(nodes),
        "valid_projections": len(valid),
        "unknown_projections": len(measurements) - len(valid),
        "crop_truncated": sum(s["crop_truncated"] for s in valid),
        "image_truncated": sum(s["image_truncated"] for s in valid),
        "band_counts": {
            band: sum(s["band"] == band for s in measurements)
            for band in ("below_reference", "within_reference", "above_reference", "unknown")
        },
        "long_side_px": (
            {"min": min(lengths), "median": float(np.median(lengths)), "max": max(lengths)}
            if lengths
            else None
        ),
    }


def scale_report(nodes, config):
    groups = []
    for index, blueprint in enumerate(config.blueprint_ids):
        for row, height in enumerate(config.heights_m):
            selected = [n for n in nodes if n["class_index"] == index and n["view"]["row"] == row]
            groups.append(
                {
                    "class_index": index,
                    "blueprint_id": blueprint,
                    "height_m": height,
                    **_summarize(selected, 8),
                }
            )
    return {
        "schema_version": 1,
        "reference_long_side_px": list(REFERENCE_LONG_SIDE),
        "reference_scope": "Passenger-car preview reference, not a universal acceptance threshold",
        "measurement": "Projected 3D box, not visible object pixels; no scale filtering",
        "overall": _summarize(nodes, len(config.blueprint_ids) * 24),
        "groups": groups,
    }


def render_crop_sheet(root, nodes, class_index, heights):
    """One source pixel per sheet pixel; no target-dependent zoom or resizing."""
    sheet = Image.new("RGB", (8 * 312, 3 * 352), "#202020")
    draw = ImageDraw.Draw(sheet)
    indexed = {
        (n["view"]["row"], n["view"]["column"]): n for n in nodes if n["class_index"] == class_index
    }
    for row, height in enumerate(heights):
        for column in range(8):
            x, y = column * 312, row * 352
            node = indexed.get((row, column))
            status = node["status"] if node else "not recorded"
            draw.text((x + 6, y + 3), f"H{height:g} P{column + 1}: {status}", fill="white")
            if node and status == "captured":
                info = node_scale(node)
                size = info["long_side_px"]
                label = (
                    f"L={size:.1f}px crop_cut={info['crop_truncated']}"
                    if size is not None
                    else "projection unknown"
                )
                draw.text((x + 6, y + 19), label, fill="#ffb060")
                with Image.open(Path(root) / node["paths"]["crop"]) as image:
                    if image.size != (300, 300):
                        raise ValueError("comparison requires 300x300 fixed-window crops")
                    sheet.paste(image.convert("RGB"), (x + 6, y + 40))
    return sheet


def compare_runs(before, after, output):
    """Compare completed runs of the same scene/optics, matched by blueprint ID."""
    from .artifacts import check_artifacts, digest, write_json
    from .config import parse_preview_config

    before, after, output = (Path(p).resolve() for p in (before, after, output))
    if before == after:
        raise ValueError("comparison requires two distinct capture directories")
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    if output.is_relative_to(before) or output.is_relative_to(after):
        raise ValueError("comparison output must be outside the source captures")
    configs, batches = [], []
    for source in (before, after):
        check_artifacts(source)
        configs.append(
            parse_preview_config(json.loads((source / "config.json").read_text(encoding="utf-8")))
        )
        batches.append(
            [
                json.loads(line)
                for line in (source / "nodes.jsonl").read_text(encoding="utf-8").splitlines()
            ]
        )
    a, b = configs
    if (
        a.map_name != b.map_name
        or a.camera != b.camera
        or a.target_pose != b.target_pose
        or a.center != b.center
        or a.ground_z != b.ground_z
        or set(a.blueprint_ids) != set(b.blueprint_ids)
    ):
        raise ValueError(
            "comparison requires matching map, camera, target pose, center and classes"
        )
    metadata = [
        json.loads((p / "metadata.json").read_text(encoding="utf-8")) for p in (before, after)
    ]
    if metadata[0].get("weather") != metadata[1].get("weather"):
        raise ValueError("comparison requires matching weather")
    output.mkdir(parents=True)
    matches = []
    for index, blueprint in enumerate(a.blueprint_ids):
        other_index = b.blueprint_ids.index(blueprint)
        old = render_crop_sheet(before, batches[0], index, a.heights_m)
        new = render_crop_sheet(after, batches[1], other_index, b.heights_m)
        combined = Image.new("RGB", (old.width, old.height + new.height + 64), "#202020")
        draw = ImageDraw.Draw(combined)
        draw.text((6, 8), f"BEFORE: {before.name} | {blueprint} | 1:1 crop pixels", fill="white")
        combined.paste(old, (0, 32))
        draw.text(
            (6, old.height + 40),
            f"AFTER: {after.name} | {blueprint} | 1:1 crop pixels",
            fill="white",
        )
        combined.paste(new, (0, old.height + 64))
        filename = f"compare_{index:02d}.png"
        combined.save(output / filename)
        matches.append(
            {
                "blueprint_id": blueprint,
                "before_index": index,
                "after_index": other_index,
                "image": filename,
            }
        )
    result = {
        "schema_version": 1,
        "before_path": str(before),
        "after_path": str(after),
        "before_config_sha256": digest(before / "config.json"),
        "after_config_sha256": digest(after / "config.json"),
        "before_grid": a.resolved["view_grid"],
        "after_grid": b.resolved["view_grid"],
        "before": scale_report(batches[0], a),
        "after": scale_report(batches[1], b),
        "matches": matches,
        "qualification": "Visual scale comparison only; not classification or policy performance",
        "control_note": "Same configured scene/optics; rendering differences may remain",
    }
    write_json(output / "comparison.json", result)
    return result
