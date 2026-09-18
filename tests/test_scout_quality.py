import importlib
import importlib.util
from types import SimpleNamespace as NS

import numpy as np
import pytest


def quality():
    assert importlib.util.find_spec("src.scout_quality"), "quality capture module is missing"
    return importlib.import_module("src.scout_quality")


def frame(number, value, timestamp=None):
    pixels = np.full((4, 8, 4), value, dtype=np.uint8)
    return NS(
        frame=number,
        timestamp=number * 0.1 if timestamp is None else timestamp,
        width=8,
        height=4,
        raw_data=pixels.tobytes(),
    )


def test_warmup_rejects_brightness_ramp_then_accepts_stable_fresh_frames():
    gate = quality().StabilityGate(warmup_seconds=2, threshold=1)
    for i in range(40):
        assert not gate.add(frame(i, 50 + i))
    accepted = [gate.add(frame(i, 100)) for i in range(40, 60)]
    assert any(accepted)
    assert gate.diagnostics["mean_rgb"] == [100, 100, 100]
    assert gate.diagnostics["window_span_seconds"] >= 0.5


def test_repeated_frame_does_not_count_as_stability():
    gate = quality().StabilityGate(warmup_seconds=0, threshold=1)
    for _ in range(100):
        assert not gate.add(frame(1, 100))


def test_constant_mean_but_changing_spatial_pattern_is_not_stable():
    gate = quality().StabilityGate(warmup_seconds=0, threshold=1)
    for i in range(20):
        im = frame(i, 0)
        pixels = np.zeros((4, 8, 4), dtype=np.uint8)
        pixels[:, (i % 2) :: 2, :3] = 200
        im.raw_data = pixels.tobytes()
        assert not gate.add(im)


def test_depth_uses_bgra_byte_order_and_preserves_metric_precision():
    im = NS(width=3, height=1, raw_data=bytes([0, 0, 255, 255, 0, 1, 0, 255, 255, 255, 255, 255]))
    actual = quality().decode_depth(im)
    assert actual.dtype == np.float32
    assert actual[0].tolist() == pytest.approx([255 / 16777215 * 1000, 256 / 16777215 * 1000, 1000])


def test_instances_use_semantic_and_color_identity_not_actor_id():
    im = NS(
        width=4,
        height=1,
        raw_data=bytes([5, 2, 14, 255, 5, 2, 14, 255, 5, 2, 4, 255, 6, 2, 14, 255]),
    )
    instances = quality().instance_records(im, [14], min_pixels=2)
    assert instances == [
        {
            "key": "14:2:5",
            "semantic_tag": 14,
            "green": 2,
            "blue": 5,
            "pixels": 2,
            "bbox_xyxy": [0, 0, 2, 1],
        }
    ]


def test_same_pose_brightness_drift_fails_repeatability(tmp_path):
    import json

    for i, mean in enumerate([60, 105], 1):
        (tmp_path / f"{i:04}.json").write_text(
            json.dumps(
                {
                    "requested_pose": [0, 0, 50, -45, 90, 0],
                    "capture_quality": {"mean_rgb": [mean] * 3},
                }
            ),
            encoding="utf-8",
        )
    report = quality().repeatability_report(tmp_path)
    assert report["passed"] is False
    assert report["groups"][0]["max_channel_range"] == 45


def test_repeatability_without_duplicate_pose_is_not_claimed_passed(tmp_path):
    report = quality().repeatability_report(tmp_path)
    assert report["passed"] is None


def test_common_frames_require_all_sensor_poses():
    inbox = quality().FrameInbox(["rgb", "depth", "instance"])
    for name, number, pose in [("rgb", 5, "new"), ("depth", 6, "new"), ("instance", 5, "new")]:
        inbox.put(name, NS(frame=number, transform=pose))
    assert inbox.ready(0, lambda pose: pose == "new") == []
    inbox.put("depth", NS(frame=5, transform="old"))
    assert inbox.ready(0, lambda pose: pose == "new") == []
    for name in ["rgb", "depth", "instance"]:
        inbox.put(name, NS(frame=7, transform="new"))
    bundles = inbox.ready(0, lambda pose: pose == "new")
    assert len(bundles) == 1
    assert {v.frame for v in bundles[0].values()} == {7}
    assert inbox.ready(0, lambda _: True) == []


def test_high_fps_can_eventually_pass_stability():
    gate = quality().StabilityGate(warmup_seconds=2, threshold=1)
    assert any(gate.add(frame(i, 100, timestamp=i / 60)) for i in range(240))


def test_local_animation_does_not_block_otherwise_stable_exposure():
    gate = quality().StabilityGate(warmup_seconds=2, threshold=1)
    passed = False
    for i in range(40):
        im = frame(i, 100)
        pixels = np.full((4, 8, 4), 100, dtype=np.uint8)
        pixels[:, 0, :3] = 0 if i % 2 else 200
        pixels[:, 1, :3] = 200 if i % 2 else 0
        im.raw_data = pixels.tobytes()
        passed |= gate.add(im)
    assert passed


def test_instance_history_is_per_route_and_missing_steps_invalidate_new_counts():
    assert hasattr(quality(), "InstanceHistory")
    history = quality().InstanceHistory()
    assert history.update("left", 0, {"a"})["new_vehicle_instance_count"] == 0
    one = history.update("left", 1, {"a", "b"})
    assert one["new_vehicle_instance_count"] == 1
    assert one["cumulative_vehicle_instance_count"] == 2
    assert history.update("right", 0, {"b"})["cumulative_vehicle_instance_count"] == 1
    assert history.update("right", 2, {"c"})["new_vehicle_instance_count"] is None
    assert history.update("missing_start", 1, {"b"})["cumulative_vehicle_instance_count"] is None
