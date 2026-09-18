"""Deterministic road-based scouting views and paginated image overviews."""

import math
from pathlib import Path


def build_views(spawns, count, heights, pitch):
    if not spawns:
        raise ValueError("Map has no road spawn points")
    if count < 1 or not heights or any(not math.isfinite(h) or h <= 0 for h in heights):
        raise ValueError("Count and heights must be positive")
    if not math.isfinite(pitch) or not -90 <= pitch <= 0:
        raise ValueError("Pitch must be between -90 and 0")
    # Farthest-point sampling gives spatial coverage instead of consecutive road positions.
    selected = [0]
    remaining = set(range(1, len(spawns)))
    while remaining and len(selected) < count:

        def distance(i):
            p = spawns[i].location
            return min(
                (p.x - spawns[j].location.x) ** 2 + (p.y - spawns[j].location.y) ** 2
                for j in selected
            )

        chosen = max(sorted(remaining), key=distance)
        if distance(chosen) < 0.01:
            break
        selected.append(chosen)
        remaining.remove(chosen)
    views = []
    for i in selected:
        p = spawns[i].location
        for height in heights:
            for yaw in (0, 90, 180, 270):
                views.append(
                    {
                        "spawn_index": i,
                        "height_above_spawn": height,
                        "pose": [p.x, p.y, p.z + height, pitch, yaw, 0],
                    }
                )
    return views


def make_overviews(output):
    from PIL import Image, ImageDraw, ImageOps

    output = Path(output)
    files = sorted(p for p in output.glob("*.png") if p.stem.isdigit())
    paths = []
    for offset in range(0, len(files), 24):
        batch = files[offset : offset + 24]
        sheet = Image.new("RGB", (4 * 320, math.ceil(len(batch) / 4) * 206), "#202020")
        draw = ImageDraw.Draw(sheet)
        for i, path in enumerate(batch):
            x, y = (i % 4) * 320, (i // 4) * 206
            with Image.open(path) as original:
                thumb = ImageOps.contain(original.convert("RGB"), (320, 180))
            sheet.paste(thumb, (x, y))
            draw.text((x + 8, y + 185), path.name, fill="white")
        target = output / f"overview_{offset // 24 + 1:02d}.jpg"
        sheet.save(target, quality=90)
        paths.append(target)
    return paths
