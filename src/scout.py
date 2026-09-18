"""Terminal-controlled CARLA RGB scout. Python 3.10+, CARLA client required."""

import argparse
import json
import math
import time
import weakref
from datetime import datetime
from pathlib import Path

from .scout_quality import (
    FrameInbox,
    InstanceHistory,
    StabilityGate,
    repeatability_report,
    save_measurements,
)


def values(t):
    return [
        t.location.x,
        t.location.y,
        t.location.z,
        t.rotation.pitch,
        t.rotation.yaw,
        t.rotation.roll,
    ]


def matches(t, pose):
    actual = values(t)
    return all(abs(a - b) < 0.02 for a, b in zip(actual[:3], pose[:3], strict=True)) and all(
        abs((a - b + 180) % 360 - 180) < 0.1 for a, b in zip(actual[3:], pose[3:], strict=True)
    )


def moved(pose, direction, distance):
    if not math.isfinite(distance):
        raise ValueError("Distance must be finite")
    p = list(pose)
    yaw = math.radians(p[4])
    axes = {
        "w": (math.cos(yaw), math.sin(yaw), 0),
        "s": (-math.cos(yaw), -math.sin(yaw), 0),
        "d": (-math.sin(yaw), math.cos(yaw), 0),
        "a": (math.sin(yaw), -math.cos(yaw), 0),
        "up": (0, 0, 1),
        "down": (0, 0, -1),
    }
    for i, v in enumerate(axes[direction]):
        p[i] += distance * v
    return p


HELP = """Commands (press Enter after each command):
  w / s / a / d [meters]   horizontal forward/back/left/right, default 5 m
  up / down [meters]      vertical movement, default 5 m
  yaw DEGREES             relative turn (yaw 15 = turn right)
  pitch DEGREES           absolute pitch (pitch -45 = look down 45 degrees)
  goto X Y Z PITCH YAW    absolute CARLA pose, meters/degrees
  spawns                 list road spawn locations
  spawn INDEX [height]   go above road spawn, default 40 m above spawn
  home                   return to starting pose
  load PATH.json         return to a saved capture pose (spaces allowed)
  save                   save another image
  pose                   print current requested pose
  help / quit
Every move automatically saves PNG + JSON. Camera movement has NO collision checks.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--output", default="out/scout")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fov", type=float)
    parser.add_argument("--timeout", type=float, default=20)
    parser.add_argument(
        "--tick",
        action="store_true",
        help="Drive an already synchronous world; no other tick client allowed",
    )
    parser.add_argument("--auto", action="store_true", help="Batch scout then exit")
    parser.add_argument("--locations", type=int, default=12)
    parser.add_argument(
        "--heights", type=float, nargs="+", default=[40], help="Heights above road spawn, meters"
    )
    parser.add_argument("--pitch", type=float, default=-45)
    parser.add_argument("--compare-from", type=Path, help="Compare routes from a scout JSON pose")
    parser.add_argument("--step-m", type=float, default=10, help="Comparison movement per step")
    parser.add_argument(
        "--dry-run", action="store_true", help="Write comparison plan without CARLA"
    )
    parser.add_argument("--rgb-only", action="store_true", help="Skip depth and instance sensors")
    parser.add_argument("--exposure-mode", choices=("manual", "histogram"), default="manual")
    parser.add_argument("--exposure-compensation", type=float, default=0.0)
    parser.add_argument("--warmup-seconds", type=float, default=2.0)
    parser.add_argument("--stability-threshold", type=float, default=1.0)
    args = parser.parse_args()
    if (
        not math.isfinite(args.warmup_seconds)
        or args.warmup_seconds < 0
        or not math.isfinite(args.stability_threshold)
        or args.stability_threshold <= 0
        or not math.isfinite(args.exposure_compensation)
        or args.timeout <= args.warmup_seconds + 1
    ):
        parser.error("Invalid quality settings; timeout must exceed warmup by more than 1 second")
    measure = args.compare_from is not None and not args.rgb_only
    quality_settings = {
        "exposure_mode": args.exposure_mode,
        "exposure_compensation": args.exposure_compensation,
        "shutter_speed": 200,
        "iso": 100,
        "fstop": 2.8,
        "motion_blur_intensity": 0,
        "warmup_seconds": args.warmup_seconds,
        "stability_threshold": args.stability_threshold,
    }

    source = None
    views = []
    if args.auto and args.compare_from:
        parser.error("--auto and --compare-from are mutually exclusive")
    if args.dry_run and not args.compare_from:
        parser.error("--dry-run requires --compare-from")
    if args.compare_from:
        from .scout_batch import build_comparison, make_overviews, write_comparison_report

        try:
            source = json.loads(args.compare_from.read_text(encoding="utf-8-sig"))
            views = build_comparison(source, args.step_m)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
    for key, default in (("width", 1280), ("height", 720), ("fov", 90)):
        if getattr(args, key) is None:
            setattr(args, key, source[key] if source else default)
    batch = args.auto or source is not None
    if args.auto:
        from .scout_batch import build_views, make_overviews

        if (
            args.locations < 1
            or any(not math.isfinite(h) or h <= 0 for h in args.heights)
            or not math.isfinite(args.pitch)
            or not -90 <= args.pitch <= 0
        ):
            parser.error("Invalid locations, heights, or pitch")
    if (
        args.width <= 0
        or args.height <= 0
        or not 1 < args.fov < 179
        or not math.isfinite(args.timeout)
        or args.timeout <= 0
    ):
        parser.error("Invalid camera dimensions, FOV, or timeout")

    def comparison_plan():
        return {
            "map": source["map"],
            "source": str(args.compare_from),
            "camera": {"width": args.width, "height": args.height, "fov": args.fov},
            "step_m": args.step_m,
            "camera_only_no_collision_check": True,
            "reset_scope": "camera_only",
            "quality_settings": quality_settings,
            "measurements": measure,
            "views": views,
        }

    if args.dry_run:
        output = Path(args.output) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output.mkdir(parents=True, exist_ok=False)
        (output / "plan.json").write_text(json.dumps(comparison_plan(), indent=2), encoding="utf-8")
        print(
            f"Dry run: {len(views)} observations, no CARLA connection. Plan: {output / 'plan.json'}"
        )
        return
    import carla

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    settings = world.get_settings()
    if settings.no_rendering_mode:
        raise RuntimeError("World has no_rendering_mode enabled; RGB rendering is required.")
    if settings.synchronous_mode and not args.tick:
        raise RuntimeError("World is synchronous. Stop other scripts, then add --tick.")
    if args.tick and not settings.synchronous_mode:
        parser.error("--tick is only for an already synchronous world")
    map_name = world.get_map().name
    if source and source["map"] != map_name:
        raise ValueError(f"Source map {source['map']} does not match server map {map_name}")
    spectator = world.get_spectator()
    original = spectator.get_transform()
    home = values(original)
    pose = home[:]
    output = Path(args.output) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    spawns = world.get_map().get_spawn_points()
    if args.auto:
        views = build_views(spawns, args.locations, args.heights, args.pitch)
    if source:
        (output / "plan.json").write_text(json.dumps(comparison_plan(), indent=2), encoding="utf-8")
    if args.auto:
        (output / "plan.json").write_text(
            json.dumps({"map": map_name, "views": views}, indent=2), encoding="utf-8"
        )
    channels = {"rgb": "sensor.camera.rgb"}
    if measure:
        channels.update(depth="sensor.camera.depth", instance="sensor.camera.instance_segmentation")
    inbox = FrameInbox(channels)
    sensors = {}
    index = 0
    last_capture = {}
    instance_history = InstanceHistory()
    vehicle_tags = {}
    if measure:
        for name in ("Vehicles", "Car", "Truck", "Bus", "Train", "Motorcycle", "Bicycle"):
            label = getattr(carla.CityObjectLabel, name, None)
            if label is not None:
                vehicle_tags[name] = int(label)
        if not vehicle_tags:
            raise RuntimeError(
                "No supported vehicle semantic labels; use --rgb-only for RGB capture"
            )
    (output / "capture_settings.json").write_text(
        json.dumps(
            {
                "quality_settings": quality_settings,
                "vehicle_semantic_tags": vehicle_tags,
                "channels": list(channels),
                "map": map_name,
                "instance_identity": "semantic:green:blue, NOT Python actor ID",
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    def transform(p):
        return carla.Transform(
            carla.Location(x=p[0], y=p[1], z=p[2]), carla.Rotation(pitch=p[3], yaw=p[4], roll=p[5])
        )

    def capture(command, view=None):
        nonlocal index, last_capture
        target = transform(pose)
        inbox.clear()
        for sensor in sensors.values():
            sensor.set_transform(target)
        spectator.set_transform(target)
        min_frame = world.get_snapshot().frame + 5
        deadline = time.monotonic() + args.timeout
        gate = StabilityGate(args.warmup_seconds, args.stability_threshold)
        image, accepted = None, None
        while time.monotonic() < deadline:
            if args.tick:
                world.tick()
            time.sleep(0.05)
            for bundle in inbox.ready(min_frame, lambda t: matches(t, pose)):
                dimensions = {(im.width, im.height) for im in bundle.values()}
                if dimensions != {(args.width, args.height)}:
                    raise RuntimeError("Sensor image dimensions differ from the configured camera")
                if (
                    max(im.timestamp for im in bundle.values())
                    - min(im.timestamp for im in bundle.values())
                    > 1e-4
                ):
                    raise RuntimeError("Sensor timestamps differ despite a common frame ID")
                if gate.add(bundle["rgb"]):
                    image, accepted = bundle["rgb"], bundle
                    break
            if image is not None:
                break
        if image is None:
            raise TimeoutError(
                "No stable, aligned camera frame at requested pose. "
                f"Last quality: {gate.diagnostics}"
            )
        index += 1
        stem = output / f"{index:04d}"
        image.save_to_disk(str(stem.with_suffix(".png")))
        metadata = {
            "map": map_name,
            "frame": image.frame,
            "timestamp": image.timestamp,
            "pose_order": ["x", "y", "z", "pitch", "yaw", "roll"],
            "pose": values(image.transform),
            "requested_pose": pose[:],
            "width": image.width,
            "height": image.height,
            "fov": args.fov,
            "command": command,
            "camera_only_no_collision_check": True,
            "capture_quality": gate.diagnostics,
            "sensor_frames": {name: im.frame for name, im in accepted.items()},
            "quality_settings": quality_settings,
        }
        if measure:
            metadata["measurements"] = save_measurements(
                output, stem.name, accepted, list(vehicle_tags.values())
            )
            identities = {item["key"] for item in metadata["measurements"]["vehicle_instances"]}
            route = view["route"] if view else "interactive"
            metadata["measurements"].update(
                instance_history.update(route, view["step"] if view else 0, identities)
            )
        if view is not None:
            metadata["comparison"] = view
        stem.with_suffix(".json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        print(f"Saved {stem}.png | pose: " + " ".join(f"{v:.2f}" for v in metadata["pose"]))

        last_capture = {
            "capture_quality": gate.diagnostics,
            "measurements": metadata.get("measurements", {}),
        }
        return stem.with_suffix(".png").name

    try:
        library = world.get_blueprint_library()
        inbox_reference = weakref.ref(inbox)
        for name, blueprint_id in channels.items():
            bp = library.find(blueprint_id)
            attributes = {
                "image_size_x": args.width,
                "image_size_y": args.height,
                "fov": args.fov,
                "sensor_tick": 0,
            }
            if name == "rgb":
                attributes.update(
                    {
                        key: quality_settings[key]
                        for key in (
                            "exposure_mode",
                            "exposure_compensation",
                            "shutter_speed",
                            "iso",
                            "fstop",
                            "motion_blur_intensity",
                        )
                    }
                )
            for key, value in attributes.items():
                if not bp.has_attribute(key):
                    raise RuntimeError(f"{blueprint_id} does not support required attribute {key}")
                bp.set_attribute(key, str(value))
            sensors[name] = world.spawn_actor(bp, original)

            def receive(image, channel=name, reference=inbox_reference):
                live = reference()
                if live is not None:
                    live.put(channel, image)

            sensors[name].listen(receive)
        print(f"Map: {map_name}\nOutput: {output.resolve()}\n{HELP}")
        if batch:
            failures = []
            results = []
            completed = 0
            try:
                for number, view in enumerate(views, 1):
                    pose = view["pose"][:]
                    description = (
                        f"route={view['route']} step={view['step']}"
                        if source
                        else f"spawn={view['spawn_index']}"
                    )
                    print(f"[{number}/{len(views)}] {description}", flush=True)
                    try:
                        filename = capture(
                            f"batch view={number} {description}", view if source else None
                        )
                        completed += 1
                        results.append(
                            {"view": number, "status": "ok", "image": filename, **last_capture}
                        )
                    except TimeoutError as exc:
                        failures.append({"view": number, "error": str(exc)})
                        results.append({"view": number, "status": "failed", "error": str(exc)})
                        print(f"Skipped: {exc}", flush=True)
                        if len(failures) >= 3 and completed == 0:
                            raise RuntimeError(
                                "Three captures failed; check CARLA rendering"
                            ) from exc
            finally:
                quality_report = repeatability_report(output)
                (output / "quality.json").write_text(
                    json.dumps(quality_report, indent=2), encoding="utf-8"
                )
                (output / "summary.json").write_text(
                    json.dumps(
                        {
                            "planned": len(views),
                            "completed": completed,
                            "failures": failures,
                            "not_run": len(views) - len(results),
                            "results": results,
                            "complete": completed == len(views),
                            "quality_passed": quality_report["passed"],
                        },
                        indent=2,
                    ),
                    encoding="utf-8",
                )
                if source:
                    write_comparison_report(output, views, results)
                    print(f"Comparison: {output / 'comparison.html'}")
                for overview in make_overviews(output):
                    print(f"Overview: {overview}")
            print(f"Finished: {completed}/{len(views)} images saved to {output}")
            if quality_report["passed"] is False:
                raise RuntimeError("Same-pose brightness changed across routes; see quality.json")
            if failures:
                raise RuntimeError("Batch incomplete; see summary.json")
            return
        capture("start")
        while True:
            try:
                raw = input("scout> ").strip()
                if not raw:
                    continue
                parts = raw.split()
                cmd = parts[0].lower()
                new_pose = pose[:]
                if cmd in ("quit", "exit"):
                    break
                if cmd == "help":
                    print(HELP)
                    continue
                if cmd == "pose":
                    print(pose)
                    continue
                if cmd == "spawns":
                    for i, point in enumerate(spawns):
                        print(i, [round(v, 2) for v in values(point)[:3]])
                    continue
                if cmd in ("w", "s", "a", "d", "up", "down") and len(parts) <= 2:
                    new_pose = moved(pose, cmd, float(parts[1]) if len(parts) == 2 else 5)
                elif cmd == "yaw" and len(parts) == 2:
                    new_pose[4] += float(parts[1])
                elif cmd == "pitch" and len(parts) == 2:
                    new_pose[3] = float(parts[1])
                elif cmd == "goto" and len(parts) == 6:
                    new_pose = [*map(float, parts[1:]), 0]
                elif cmd == "spawn" and len(parts) in (2, 3):
                    i = int(parts[1])
                    if not 0 <= i < len(spawns):
                        raise ValueError("Spawn index out of range")
                    new_pose = values(spawns[i])
                    new_pose[2] += float(parts[2]) if len(parts) == 3 else 40
                    new_pose[3], new_pose[5] = -45, 0
                elif cmd == "home" and len(parts) == 1:
                    new_pose = home[:]
                elif cmd == "load" and len(parts) >= 2:
                    record = json.loads(Path(raw.split(maxsplit=1)[1].strip('"')).read_text())
                    if record["map"] != map_name:
                        raise ValueError("Saved pose belongs to a different map")
                    new_pose = list(map(float, record["pose"]))
                elif cmd != "save" or len(parts) != 1:
                    raise ValueError("Unknown command or arguments; type help")
                if len(new_pose) != 6 or not all(math.isfinite(v) for v in new_pose):
                    raise ValueError("Pose must contain six finite numbers")
                if not -90 <= new_pose[3] <= 90:
                    raise ValueError("Pitch must be between -90 and 90")
                pose = new_pose
                capture(raw)
            except (ValueError, OSError, KeyError, TypeError, TimeoutError) as exc:
                print(f"Error: {exc}")
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
        if batch:
            raise SystemExit(130) from None
    finally:
        # Attempt cleanup of every owned sensor even if one sensor RPC fails.
        for name, sensor in reversed(list(sensors.items())):
            for action in ("stop", "destroy"):
                try:
                    getattr(sensor, action)()
                except Exception as exc:
                    print(f"Cleanup error: {name}.{action}: {exc}")
        spectator.set_transform(original)


if __name__ == "__main__":
    main()
