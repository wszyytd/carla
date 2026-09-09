import numpy as np
import pytest

from src.carla_experiments.config import ObservationConfig
from src.carla_experiments.metrics import (
    ProjectedBox,
    build_projection_matrix,
    count_dominant_vehicle_instance_pixels,
    derive_motion_metrics,
    evaluate_observation,
    path_length,
    project_bounding_box,
)
from src.carla_experiments.trajectories.baselines import UavLimits, Vec3

OBSERVATION = ObservationConfig(
    edge_margin_fraction=0.05,
    min_bbox_short_side_px=32.0,
    max_center_error_fraction=0.2,
    max_distance_m=120.0,
    min_instance_pixels=500,
    min_valid_fraction=0.995,
    max_continuous_invalid_seconds=0.2,
)
LIMITS = UavLimits(12.0, 4.0, 8.0, 90.0)


def test_projection_matrix_and_carla_axis_conversion_put_forward_point_at_center() -> None:
    matrix = build_projection_matrix(width=1920, height=1080, fov_deg=60.0)
    expected_focal = 1920 / (2.0 * np.tan(np.deg2rad(60.0) / 2.0))
    vertices = np.array([[10.0, 0.0, 0.0], [10.0, 1.0, 1.0]])

    projected = project_bounding_box(vertices, np.eye(4), matrix)

    assert matrix[0, 0] == pytest.approx(expected_focal)
    assert projected.in_front is True
    assert projected.x_min == pytest.approx(960.0)
    assert projected.y_max == pytest.approx(540.0)


def test_projection_marks_box_with_nonpositive_depth_as_not_in_front() -> None:
    vertices = np.array([[10.0, 0.0, 0.0], [0.0, 1.0, 1.0]])

    projected = project_bounding_box(
        vertices,
        np.eye(4),
        build_projection_matrix(width=100, height=100, fov_deg=60.0),
    )

    assert projected.in_front is False


def test_count_dominant_vehicle_instance_pixels_uses_largest_vehicle_instance_in_box() -> None:
    raw = bytes(
        [
            0x01,
            0x02,
            10,
            255,
            0x01,
            0x02,
            10,
            255,
            0x03,
            0x04,
            10,
            255,
            0x01,
            0x02,
            10,
            255,
        ]
    )

    assert count_dominant_vehicle_instance_pixels(
        raw,
        width=2,
        height=2,
        projected_box=ProjectedBox(0.0, 0.0, 2.0, 2.0, True),
    ) == 3


def test_dominant_vehicle_instance_count_filters_semantics_and_clips_box() -> None:
    raw = bytes(
        [
            0x01, 0x02, 10, 255,
            0x01, 0x02, 5, 255,
            0x03, 0x04, 10, 255,
            0x01, 0x02, 10, 255,
        ]
    )

    assert count_dominant_vehicle_instance_pixels(
        raw,
        width=2,
        height=2,
        projected_box=ProjectedBox(-1.2, -0.1, 1.1, 2.0, True),
    ) == 2


@pytest.mark.parametrize(
    "projected_box",
    [
        ProjectedBox(0.0, 0.0, 2.0, 2.0, False),
        ProjectedBox(3.0, 3.0, 4.0, 4.0, True),
    ],
)
def test_dominant_vehicle_instance_count_returns_zero_when_box_has_no_visible_crop(
    projected_box: ProjectedBox,
) -> None:
    raw = bytes([0x01, 0x02, 10, 255] * 4)

    assert count_dominant_vehicle_instance_pixels(
        raw, width=2, height=2, projected_box=projected_box
    ) == 0


def test_dominant_vehicle_instance_count_validates_raw_length() -> None:
    raw = bytes([0x01, 0x02, 10, 255] * 4)

    with pytest.raises(ValueError, match="raw image length"):
        count_dominant_vehicle_instance_pixels(
            raw[:-1],
            width=2,
            height=2,
            projected_box=ProjectedBox(0.0, 0.0, 2.0, 2.0, True),
        )


def valid_observation_inputs() -> dict[str, object]:
    return {
        "projected_box": ProjectedBox(30.0, 30.0, 70.0, 70.0, True),
        "instance_pixels": 1000,
        "distance_m": 50.0,
        "speed_mps": 5.0,
        "acceleration_mps2": 2.0,
        "jerk_mps3": 3.0,
        "gimbal_rate_deg_s": 20.0,
        "width": 100,
        "height": 100,
        "config": OBSERVATION,
        "limits": LIMITS,
    }


@pytest.mark.parametrize(
    ("field", "invalid_value", "reason"),
    [
        ("projected_box", ProjectedBox(30.0, 30.0, 70.0, 70.0, False), "behind_camera"),
        ("projected_box", ProjectedBox(4.0, 30.0, 70.0, 70.0, True), "edge_margin"),
        ("projected_box", ProjectedBox(40.0, 40.0, 60.0, 60.0, True), "short_side"),
        ("projected_box", ProjectedBox(5.0, 34.0, 37.0, 66.0, True), "center_error"),
        ("distance_m", 121.0, "distance"),
        ("instance_pixels", 499, "instance_pixels"),
        ("speed_mps", 12.1, "speed"),
        ("acceleration_mps2", 4.1, "acceleration"),
        ("jerk_mps3", 8.1, "jerk"),
        ("gimbal_rate_deg_s", 90.1, "gimbal_rate"),
    ],
)
def test_evaluate_observation_rejects_each_failed_constraint(
    field: str, invalid_value: object, reason: str
) -> None:
    inputs = valid_observation_inputs()
    inputs[field] = invalid_value

    metrics = evaluate_observation(**inputs)

    assert metrics.valid is False
    assert reason in metrics.invalid_reasons


def test_evaluate_observation_accepts_all_constraints_at_once() -> None:
    metrics = evaluate_observation(**valid_observation_inputs())

    assert metrics.valid is True
    assert metrics.center_error_fraction == pytest.approx(0.0)
    assert metrics.invalid_reasons == ()


def test_path_length_and_motion_derivatives_keep_sample_count() -> None:
    positions = (Vec3(0.0, 0.0, 0.0), Vec3(3.0, 0.0, 0.0), Vec3(3.0, 4.0, 0.0))

    motion = derive_motion_metrics(positions, dt=1.0)

    assert path_length(positions) == pytest.approx(7.0)
    assert len(motion.velocities) == len(positions)
    assert len(motion.accelerations) == len(positions)
    assert len(motion.jerks) == len(positions)
    assert motion.velocities[0] == Vec3(0.0, 0.0, 0.0)
    assert motion.accelerations[0] == Vec3(0.0, 0.0, 0.0)
    assert motion.jerks[0] == Vec3(0.0, 0.0, 0.0)
    assert motion.speeds[-1] == pytest.approx(4.0)
