"""Preview files are inspectable, incremental, and separate from training data."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .preview import crop_target, decode_rgb, pose_values


def write_json(path, value):
    Path(path).write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def save_images(root, node, pair, box):
    rgb, instance = decode_rgb(pair.rgb), decode_rgb(pair.instance)
    if rgb.shape != instance.shape:
        raise ValueError("RGB and instance dimensions differ")
    crop = crop_target(rgb, box)
    prefix = f"images/c{node['class_index']:02d}_{node['view']['key']}"
    images = {
        "rgb": Image.fromarray(rgb),
        "instance": Image.fromarray(instance),
        "crop": Image.fromarray(crop.image),
    }
    overlay = images["rgb"].copy()
    bounds = [box.x_min, box.y_min, box.x_max, box.y_max]
    finite = box.in_front and all(math.isfinite(v) for v in bounds)
    if finite:
        w, h = overlay.size
        clipped = [
            max(0, bounds[0]),
            max(0, bounds[1]),
            min(w - 1, bounds[2]),
            min(h - 1, bounds[3]),
        ]
        if clipped[0] <= clipped[2] and clipped[1] <= clipped[3]:
            ImageDraw.Draw(overlay).rectangle(clipped, outline="red", width=2)
    images["overlay"] = overlay
    paths = {}
    for kind, image in images.items():
        relative = f"{prefix}_{kind}.png"
        path = Path(root) / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        image.save(path)
        paths[kind] = relative
    node.update(
        paths=paths,
        frame=int(pair.frame),
        timestamp=float(pair.rgb.timestamp),
        rgb_pose=list(pose_values(pair.rgb.transform)),
        instance_pose=list(pose_values(pair.instance.transform)),
        instance_timestamp=float(pair.instance.timestamp),
        projected_box=bounds if finite else None,
        crop_bbox=list(crop.bbox),
        crop_box_valid=crop.box_valid,
        rgb_mean=float(np.mean(rgb)),
        image_size=[rgb.shape[1], rgb.shape[0]],
    )


def contact_sheets(root, nodes, class_count):
    for index in range(class_count):
        sheet = Image.new("RGB", (8 * 240, 3 * 165), "#202020")
        draw = ImageDraw.Draw(sheet)
        indexed = {
            (n["view"]["row"], n["view"]["column"]): n for n in nodes if n["class_index"] == index
        }
        for row in range(3):
            for col in range(8):
                x, y = col * 240, row * 165
                node = indexed.get((row, col))
                status = node["status"] if node else "not captured"
                if node and "paths" in node:
                    with Image.open(Path(root) / node["paths"]["overlay"]) as source:
                        thumbnail = source.copy()
                    thumbnail.thumbnail((236, 136))
                    sheet.paste(thumbnail, (x, y + 24))
                draw.text((x + 4, y + 4), f"H{20 + row * 10} P{col + 1}: {status}", fill="white")
        sheet.save(Path(root) / f"contact_sheet_{index:02d}.png")


def finish_artifacts(root, nodes, summary):
    """Also runs for failed captures; incomplete status remains explicit."""
    write_json(Path(root) / "summary.json", summary)
    contact_sheets(root, nodes, summary["class_count"])
    files = sorted(p for p in Path(root).rglob("*") if p.is_file() and p.name != "checksums.json")
    write_json(
        Path(root) / "checksums.json", {p.relative_to(root).as_posix(): digest(p) for p in files}
    )


def check_artifacts(root):
    root = Path(root).resolve()
    try:
        summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
        hashes = json.loads((root / "checksums.json").read_text(encoding="utf-8"))
        mandatory = {"summary.json", "nodes.jsonl", "config.json", "metadata.json"}
        if not mandatory <= hashes.keys():
            raise ValueError("missing required manifest entries")
        for name, expected in hashes.items():
            path = (root / name).resolve()
            if not path.is_relative_to(root):
                raise ValueError("manifest path escapes output directory")
            if not path.is_file() or digest(path) != expected:
                raise ValueError(f"missing file or checksum mismatch: {name}")
        nodes = [
            json.loads(line)
            for line in (root / "nodes.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        captured = [n for n in nodes if n["status"] == "captured"]
        rejected = [n for n in nodes if n["status"] == "rejected"]
        keys = {(n["class_index"], n["view"]["key"]) for n in nodes}
        if len(keys) != len(nodes):
            raise ValueError("duplicate node identity")
        if len(captured) != summary["captured"] or len(rejected) != summary["rejected"]:
            raise ValueError("node counts disagree with summary")
        if summary["status"] != "complete" or len(nodes) != summary["planned"]:
            raise ValueError("capture incomplete; inspect summary.json")
        if not captured:
            raise ValueError("no usable captured nodes")
        for node in captured:
            if set(node["paths"]) != {"rgb", "instance", "crop", "overlay"}:
                raise ValueError("node missing an image modality")
            for kind, name in node["paths"].items():
                if name not in hashes:
                    raise ValueError(f"image missing from manifest: {name}")
                with Image.open(root / name) as image:
                    image.load()
                    expected_size = (300, 300) if kind == "crop" else tuple(node["image_size"])
                    if image.size != expected_size or image.mode != "RGB":
                        raise ValueError(f"wrong image shape: {name}")
        return summary
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid preview artifacts: {error}") from error
