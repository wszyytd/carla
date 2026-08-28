from copy import deepcopy
from pathlib import Path

import pytest

from src.carla_experiments.config import parse_path_cost_config


def valid_mapping() -> dict[str, object]:
    return {
        "random_seed": 20260826,
        "client": {"host": "localhost", "port": 2000, "timeout_seconds": 10},
        "world": {"map": "Town10HD_Opt", "fixed_delta_seconds": 0.05},
        "traffic_manager": {
            "port": 8000,
            "random_seed": 20260826,
            "synchronous_mode": True,
        },
        "route": {
            "waypoint_spacing_m": 2.0,
            "window_length_m": 120.0,
            "min_total_turn_deg": 60.0,
            "candidate_rank": 0,
            "target_speed_mps": 8.0,
            "completion_tolerance_m": 3.0,
            "max_cross_track_error_m": 2.0,
            "speed_tolerance_fraction": 0.15,
            "max_duration_seconds": 40.0,
        },
        "target": {"blueprint_filter": "vehicle.*"},
        "camera": {
            "width": 1920,
            "height": 1080,
            "fov_deg": 60.0,
            "sensor_tick_seconds": 0.1,
        },
        "uav": {
            "altitude_m": 40.0,
            "initial_offset_x_m": 0.0,
            "initial_offset_y_m": 0.0,
            "max_speed_mps": 12.0,
            "max_acceleration_mps2": 4.0,
            "max_jerk_mps3": 8.0,
            "max_gimbal_rate_deg_s": 90.0,
        },
        "observation": {
            "edge_margin_fraction": 0.05,
            "min_bbox_short_side_px": 32.0,
            "max_center_error_fraction": 0.2,
            "max_distance_m": 120.0,
            "min_instance_pixels": 500,
            "min_valid_fraction": 0.995,
            "max_continuous_invalid_seconds": 0.2,
        },
        "output": {
            "root": "out/path_cost",
            "warmup_frames": 4,
            "sample_every_frames": 10,
        },
    }


def test_parse_path_cost_config_returns_typed_values() -> None:
    parsed = parse_path_cost_config(valid_mapping())

    assert parsed.random_seed == 20260826
    assert parsed.world.fixed_delta_seconds == 0.05
    assert parsed.traffic_manager.synchronous_mode is True
    assert parsed.route.target_speed_mps == 8.0
    assert parsed.camera.resolution == (1920, 1080)
    assert parsed.observation.min_valid_fraction == 0.995
    assert parsed.output.root == Path("out/path_cost")


@pytest.mark.parametrize(
    ("field_path", "invalid_value"),
    [
        ("random_seed", True),
        ("world.fixed_delta_seconds", 0),
        ("traffic_manager.port", True),
        ("traffic_manager.synchronous_mode", 1),
        ("route.candidate_rank", -1),
        ("route.target_speed_mps", float("inf")),
        ("route.max_cross_track_error_m", 0),
        ("route.speed_tolerance_fraction", 1.1),
        ("camera.width", 0),
        ("camera.sensor_tick_seconds", 0.075),
        ("uav.max_jerk_mps3", 0),
        ("observation.edge_margin_fraction", 0.5),
        ("observation.min_valid_fraction", 1.1),
        ("output.warmup_frames", -1),
    ],
)
def test_parse_path_cost_config_rejects_invalid_boundaries(
    field_path: str, invalid_value: object
) -> None:
    mapping = deepcopy(valid_mapping())
    section, separator, field = field_path.partition(".")
    if separator:
        section_mapping = mapping[section]
        assert isinstance(section_mapping, dict)
        section_mapping[field] = invalid_value
    else:
        mapping[field_path] = invalid_value

    with pytest.raises(ValueError, match=field_path.replace(".", r"\.")):
        parse_path_cost_config(mapping)


def test_parse_path_cost_config_rejects_empty_required_sections() -> None:
    mapping = valid_mapping()
    mapping["camera"] = None

    with pytest.raises(ValueError, match=r"camera must be a mapping"):
        parse_path_cost_config(mapping)


def test_parse_path_cost_config_requires_min_total_turn_deg() -> None:
    mapping = valid_mapping()
    route = mapping["route"]
    assert isinstance(route, dict)
    route.pop("min_total_turn_deg")

    with pytest.raises(ValueError, match=r"route\.min_total_turn_deg"):
        parse_path_cost_config(mapping)
