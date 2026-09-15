import importlib
from types import SimpleNamespace

import numpy as np
import pytest

from src.carla_experiments.metrics import ProjectedBox
from src.carla_experiments.sensors import FramePair


def api():
    return importlib.import_module("src.carla_experiments.aod.preview")


def transform(x=0, y=0, z=20, yaw=0, pitch=-45, roll=0):
    return SimpleNamespace(
        location=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(pitch=pitch, yaw=yaw, roll=roll),
    )


def pair(frame, *, rgb_pose=None, instance_pose=None):
    return FramePair(
        frame,
        SimpleNamespace(frame=frame, timestamp=frame * 0.05, transform=rgb_pose or transform()),
        SimpleNamespace(
            frame=frame, timestamp=frame * 0.05, transform=instance_pose or transform()
        ),
    )


def test_preview_positions_use_ground_reference_not_target_height():
    views = api().preview_views((100, 200, 5), ground_z=3)
    assert len(views) == 24
    assert (views[0].x, views[0].y, views[0].z) == (110, 200, 23)
    assert (views[7].x, views[7].y, views[7].z) == (90, 190, 23)
    assert (views[8].x, views[8].y, views[8].z) == (110, 200, 33)
    assert (views[16].x, views[16].y, views[16].z) == (110, 200, 43)
    assert views[0].yaw == pytest.approx(180)
    assert views[0].pitch < 0


def test_crop_pads_edges_and_preserves_original_scale():
    rgb = np.full((4, 4, 3), 80, dtype=np.uint8)
    crop = api().crop_target(rgb, ProjectedBox(0, 0, 2, 2, True), size=4)
    assert crop.image.shape == (4, 4, 3)
    assert np.all(crop.image[0] == 0)
    assert np.all(crop.image[:, 0] == 0)
    assert np.all(crop.image[1:, 1:] == 80)
    assert crop.bbox == pytest.approx((0.25, 0.25, 0.75, 0.75))
    assert crop.box_valid


def test_bad_projection_is_explicit_empty_observation():
    crop = api().crop_target(
        np.full((4, 4, 3), 80, dtype=np.uint8),
        ProjectedBox(float("nan"), 0, 0, 0, False),
        size=4,
    )
    assert not crop.box_valid
    assert not crop.image.any()


def test_old_frame_is_rejected_even_if_pose_matches():
    assert not api().matches_view(pair(10), transform(), min_frame=10)


def test_both_sensors_must_match_requested_pose():
    assert not api().matches_view(
        pair(12, instance_pose=transform(x=5)),
        transform(),
        min_frame=10,
    )


def test_camera_rotation_wrap_does_not_reject_same_pose():
    candidate = pair(12, rgb_pose=transform(yaw=359), instance_pose=transform(yaw=-1))
    assert api().matches_view(candidate, transform(yaw=-1), min_frame=10)


def test_geometry_check_accounts_for_clearance():
    boxes = [((-1, -1, 19), (1, 1, 21))]
    assert api().blocked_by((1.5, 0, 20), boxes, clearance_m=1)
    assert not api().blocked_by((3, 0, 20), boxes, clearance_m=1)


def test_malformed_raw_frame_is_not_treated_as_occlusion():
    with pytest.raises(ValueError, match="bytes"):
        api().decode_rgb(SimpleNamespace(width=4, height=4, raw_data=b"broken"))
