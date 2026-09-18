"""Frame alignment, settling diagnostics and lossless scout sensor measurements."""

import json
import threading
from collections import deque
from pathlib import Path

import numpy as np


def bgra(image):
    return np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)


class StabilityGate:
    """Require fresh frames, simulation-time warmup and a stable spatial RGB sample."""

    def __init__(self, warmup_seconds=2.0, threshold=1.0):
        self.warmup_seconds = warmup_seconds
        self.threshold = threshold
        self.history = deque(maxlen=8)
        self.first_timestamp = None
        self.last_frame = -1
        self.diagnostics = {}

    def add(self, image):
        if image.frame <= self.last_frame:
            return False
        self.last_frame = image.frame
        if self.first_timestamp is None:
            self.first_timestamp = image.timestamp
        if self.history and image.timestamp - self.history[-1][0] < 0.099:
            return False
        rgb = bgra(image)[:, :, 2::-1]
        # A fixed spatial sample also catches equal-mean patterns that are still changing.
        sample = rgb[:: max(1, image.height // 36), :: max(1, image.width // 64)].astype(float)
        mean = rgb.mean(axis=(0, 1))
        self.history.append((image.timestamp, sample, mean))
        means = np.array([entry[2] for entry in self.history])
        span = self.history[-1][0] - self.history[0][0]
        brightness_range = float(np.ptp(means, axis=0).max())
        spatial_mae = max(
            float(np.median(np.abs(entry[1] - sample).mean(axis=2))) for entry in self.history
        )
        self.diagnostics = {
            "mean_rgb": mean.tolist(),
            "window_frames": len(self.history),
            "window_span_seconds": span,
            "max_channel_range": brightness_range,
            "max_spatial_median_error": spatial_mae,
            "elapsed_sim_seconds": image.timestamp - self.first_timestamp,
            "threshold_0_255": self.threshold,
        }
        return (
            image.timestamp - self.first_timestamp >= self.warmup_seconds
            and len(self.history) == 8
            and span >= 0.5
            and brightness_range <= self.threshold
            and spatial_mae <= self.threshold
        )


class FrameInbox:
    """Match exact frame IDs and pose across sensors, despite callback ordering."""

    def __init__(self, names):
        self.buffers = {name: {} for name in names}
        self.lock = threading.Lock()

    def put(self, name, image):
        with self.lock:
            buffer = self.buffers[name]
            buffer[image.frame] = image
            while len(buffer) > 32:
                del buffer[min(buffer)]

    def clear(self):
        with self.lock:
            for buffer in self.buffers.values():
                buffer.clear()

    def ready(self, min_frame, pose_matches):
        with self.lock:
            common = set.intersection(*(set(b) for b in self.buffers.values()))
            bundles = []
            for number in sorted(common):
                bundle = {name: buffer.pop(number) for name, buffer in self.buffers.items()}
                if number >= min_frame and all(
                    pose_matches(im.transform) for im in bundle.values()
                ):
                    bundles.append(bundle)
            if common:
                for buffer in self.buffers.values():
                    for number in list(buffer):
                        if number <= max(common):
                            del buffer[number]
            return bundles


def decode_depth(image):
    pixels = bgra(image).astype(np.uint32)
    value = pixels[:, :, 2] + 256 * pixels[:, :, 1] + 65536 * pixels[:, :, 0]
    return (value.astype(np.float64) * (1000.0 / 16777215)).astype(np.float32)


def instance_records(image, vehicle_tags, min_pixels=20):
    pixels = bgra(image)
    mask = np.isin(pixels[:, :, 2], vehicle_tags)
    colors, counts = np.unique(pixels[mask, :3], axis=0, return_counts=True)
    result = []
    for color, count in zip(colors, counts, strict=True):
        if count < min_pixels:
            continue
        blue, green, semantic = map(int, color)
        ys, xs = np.where(np.all(pixels[:, :, :3] == color, axis=2))
        result.append(
            {
                "key": f"{semantic}:{green}:{blue}",
                "semantic_tag": semantic,
                "green": green,
                "blue": blue,
                "pixels": int(count),
                "bbox_xyxy": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
            }
        )
    return result


def save_measurements(output, stem, bundle, vehicle_tags):
    from PIL import Image

    output = Path(output)
    for name in ("depth", "instance"):
        (output / name).mkdir(exist_ok=True)
        bundle[name].save_to_disk(str(output / name / f"{stem}.png"))
    depth = decode_depth(bundle["depth"])
    np.save(output / "depth" / f"{stem}.npy", depth, allow_pickle=False)
    preview = (np.clip(depth / 150, 0, 1) * 255).astype(np.uint8)
    Image.fromarray(preview).save(output / "depth" / f"{stem}_preview.png")
    return {
        "depth_m": f"depth/{stem}.npy",
        "depth_raw": f"depth/{stem}.png",
        "depth_preview": f"depth/{stem}_preview.png",
        "instance_raw": f"instance/{stem}.png",
        "vehicle_instances": instance_records(bundle["instance"], vehicle_tags),
        "instance_identity_scope": "semantic:green:blue within this run, NOT Python actor ID",
        "min_instance_pixels": 20,
    }


def repeatability_report(output, threshold=3.0):
    groups = {}
    for path in sorted(Path(output).glob("*.json")):
        if not path.stem.isdigit():
            continue
        record = json.loads(path.read_text(encoding="utf-8"))
        if "capture_quality" not in record:
            continue
        key = tuple(round(v, 4) for v in record["requested_pose"])
        groups.setdefault(key, []).append((path.name, record["capture_quality"]["mean_rgb"]))
    reports = []
    for pose, entries in groups.items():
        if len(entries) < 2:
            continue
        spread = float(np.ptp([entry[1] for entry in entries], axis=0).max())
        reports.append(
            {
                "pose": list(pose),
                "files": [entry[0] for entry in entries],
                "max_channel_range": spread,
                "passed": spread <= threshold,
            }
        )
    return {
        "threshold_0_255": threshold,
        "groups": reports,
        "passed": all(item["passed"] for item in reports) if reports else None,
        "scope": "same-pose mean RGB repeatability, not a guarantee of static geometry",
    }


class InstanceHistory:
    """Per-route instance discovery diagnostics, only over a complete prefix."""

    def __init__(self):
        self.routes = {}

    def update(self, route, step, identities):
        if step == 0:
            self.routes[route] = {"seen": set(), "last_step": -1, "complete": True}
        state = self.routes.setdefault(route, {"seen": set(), "last_step": -1, "complete": False})
        state["complete"] &= step == state["last_step"] + 1
        new_ids = identities - state["seen"] if step > 0 else set()
        state["seen"].update(identities)
        state["last_step"] = step
        complete = state["complete"]
        return {
            "visible_vehicle_instance_count": len(identities),
            "new_vehicle_instance_count": len(new_ids) if complete else None,
            "cumulative_vehicle_instance_count": len(state["seen"]) if complete else None,
            "new_instance_keys": sorted(new_ids) if complete else None,
            "route_prefix_complete": complete,
        }
