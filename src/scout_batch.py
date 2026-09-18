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


def build_comparison(source, step_m):
    """Equal-length camera probes; each route starts at the saved observation."""
    from .scout import moved

    if not isinstance(source, dict):
        raise ValueError("Source must be a scout JSON object")
    if not isinstance(source.get("map"), str) or not source["map"]:
        raise ValueError("Source must include a map name")
    try:
        pose = list(map(float, source["pose"]))
        width, height, fov = source["width"], source["height"], float(source["fov"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Source requires pose, width, height and fov") from exc
    if len(pose) != 6 or not all(math.isfinite(v) for v in pose) or not -90 <= pose[3] <= 90:
        raise ValueError("Pose must have six finite values and pitch in [-90, 90]")
    if (
        type(width) is not int
        or type(height) is not int
        or width <= 0
        or height <= 0
        or not 1 < fov < 179
    ):
        raise ValueError("Invalid source camera dimensions or FOV")
    if not math.isfinite(step_m) or step_m <= 0:
        raise ValueError("Step must be finite and positive")
    routes = {
        "up": ["up", "up"],
        "down": ["down", "down"],
        "left": ["a", "a"],
        "right": ["d", "d"],
        "forward": ["w", "w"],
        "back": ["s", "s"],
        "left_up": ["a", "up"],
        "up_left": ["up", "a"],
        "right_up": ["d", "up"],
        "up_right": ["up", "d"],
    }
    views = []
    for name, actions in routes.items():
        current = pose[:]
        for step, action in enumerate(["start", *actions]):
            if step:
                current = moved(current, action, step_m)
            views.append(
                {
                    "route": name,
                    "step": step,
                    "action": action,
                    "pose": current[:],
                    "path_length_m": step * step_m,
                }
            )
    return views


def write_comparison_report(output, views, results):
    """Include missing captures explicitly; human annotation fields stay empty."""
    import csv
    from html import escape

    output = Path(output)
    by_view = {r["view"]: r for r in results}
    rows = []
    cards = []
    for number, view in enumerate(views, 1):
        result = by_view.get(number, {"status": "not_run"})
        image_name = result.get("image", "")
        measurements = result.get("measurements", {})
        capture_quality = result.get("capture_quality", {})
        row = {
            "view": number,
            "route": view["route"],
            "step": view["step"],
            "action": view["action"],
            "path_length_m": view["path_length_m"],
            "status": result["status"],
            "image": image_name,
            "error": result.get("error", ""),
            **dict(zip(("x", "y", "z", "pitch", "yaw", "roll"), view["pose"], strict=True)),
            "visible_vehicle_instance_count": measurements.get(
                "visible_vehicle_instance_count", ""
            ),
            "new_vehicle_instance_count": measurements.get("new_vehicle_instance_count", ""),
            "cumulative_vehicle_instance_count": measurements.get(
                "cumulative_vehicle_instance_count", ""
            ),
            "mean_rgb": str(capture_quality.get("mean_rgb", "")),
            "depth_m": measurements.get("depth_m", ""),
            "instance_raw": measurements.get("instance_raw", ""),
            "new_target_count": "",
            "new_visible_ground_m2": "",
            "notes": "",
        }
        rows.append(row)
        label = f"{view['route']} / step {view['step']} / {view['path_length_m']:g} m"
        picture = (
            (
                f'<a href="{escape(image_name, quote=True)}">'
                f'<img src="{escape(image_name, quote=True)}" loading="lazy" '
                f'alt="{escape(label, quote=True)}"></a>'
            )
            if image_name
            else '<div class="missing">No image</div>'
        )
        links = ""
        if measurements:
            links = (
                f'<p><a href="{escape(measurements["depth_preview"], quote=True)}">深度预览</a> · '
                f'<a href="{escape(measurements["instance_raw"], quote=True)}">实例原图</a></p>'
                f"<p>车辆类实例：{measurements['visible_vehicle_instance_count']}；"
                f"本步新增实例：{measurements['new_vehicle_instance_count']}；"
                f"本路线累计：{measurements['cumulative_vehicle_instance_count']}</p>"
            )
        if capture_quality:
            mean_label = str([round(v, 1) for v in capture_quality["mean_rgb"]])
            links += f"<p>平均 RGB：{escape(mean_label)}</p>"
        cards.append(
            f"<article><h2>{escape(label)}</h2>{picture}"
            f"<p>{escape(view['action'])} | {escape(result['status'])} "
            f"{escape(result.get('error', ''))}</p>"
            f"{links}<small>pose: {escape(str(view['pose']))}</small></article>"
        )
    with (output / "comparison.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["view"])
        writer.writeheader()
        writer.writerows(rows)
    html = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>遮挡路线对照</title><style>
body{font-family:system-ui,sans-serif;margin:24px;background:#f5f5f3;color:#222}
main{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}
article{background:white;padding:12px;border:1px solid #ddd;border-radius:8px;
overflow-wrap:anywhere}
h2{font-size:16px}img{width:100%;display:block}
.missing{background:#ddd;padding:60px 0;text-align:center}
p{line-height:1.6}small{color:#555}@media(max-width:850px){main{grid-template-columns:1fr}}
</style><h1>同一起点 · 遮挡路线对照</h1>
<p>每行一条路线：起点 → 第一步 → 第二步。左右、前后相对起点相机朝向，俯仰角固定。
距离为请求位姿的累计位移，不包含路线之间的重置；不是已验证的飞行距离。
相机直接设置位置，不检查碰撞，不生成连续视频，不重置场景中的动态物体。</p>
<p>起点画面是停止对照。比较中途新露出的地面与车辆，不只比较终点。
相同终点的组合路线在静态场景中应得到近似相同画面；中途观察可能不同。
自动统计的是车辆语义类别的分割实例编号，编号不等于 Python Actor ID，
也不保证一个实例就是一辆完整车辆；首次起点的“新增实例”为 0，累计数含起点。
路线前序缺图时，新增及累计实例统计记为 null/None，不能当成 0。
真实目标发现数和新增可见面积仍留空，尚未限定搜索区域。</p>
<p><a href="comparison.csv">下载记录表</a> · <a href="plan.json">采集计划</a> ·
<a href="summary.json">运行状态</a> · <a href="quality.json">重复位姿亮度检查</a></p><main>"""
    (output / "comparison.html").write_text(
        html + "".join(cards) + "</main></html>", encoding="utf-8"
    )
