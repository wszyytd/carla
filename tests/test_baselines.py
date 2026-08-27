import pytest

from src.carla_experiments.trajectories.baselines import (
    Euler,
    MotionState,
    UavLimits,
    Vec3,
    advance_jerk_limited,
    angle_distance,
    command_baseline,
    look_at,
    norm,
    step_angle,
)

LIMITS = UavLimits(
    max_speed_mps=12.0,
    max_acceleration_mps2=4.0,
    max_jerk_mps3=8.0,
    max_gimbal_rate_deg_s=90.0,
)


def stationary(position: Vec3 | None = None) -> MotionState:
    if position is None:
        position = Vec3(0.0, 0.0, 40.0)
    return MotionState(
        position=position,
        velocity=Vec3(0.0, 0.0, 0.0),
        acceleration=Vec3(0.0, 0.0, 0.0),
        gimbal=Euler(-90.0, 0.0),
    )


def test_look_at_returns_carla_pitch_and_yaw() -> None:
    assert look_at(Vec3(0, 0, 40), Vec3(0, 0, 0)) == Euler(pitch=-90, yaw=0, roll=0)
    assert look_at(Vec3(0, 0, 40), Vec3(10, 0, 40)) == Euler(pitch=0, yaw=0, roll=0)
    assert look_at(Vec3(0, 0, 40), Vec3(0, 10, 40)) == Euler(pitch=0, yaw=90, roll=0)


def test_step_angle_crosses_wrap_boundary_by_shortest_path() -> None:
    assert step_angle(179.0, -179.0, max_delta_deg=1.0) == pytest.approx(180.0)
    assert angle_distance(179.0, -179.0) == pytest.approx(2.0)


def test_advance_jerk_limited_enforces_every_motion_limit() -> None:
    state = stationary()
    dt = 0.05

    next_state = advance_jerk_limited(
        state,
        desired_position=Vec3(100.0, 100.0, 100.0),
        desired_gimbal=Euler(0.0, 180.0),
        limits=LIMITS,
        dt=dt,
    )

    acceleration_delta = next_state.acceleration - state.acceleration
    assert norm(next_state.velocity) <= LIMITS.max_speed_mps
    assert norm(next_state.acceleration) <= LIMITS.max_acceleration_mps2
    assert norm(acceleration_delta) / dt <= LIMITS.max_jerk_mps3 + 1e-12
    assert (
        angle_distance(next_state.gimbal.yaw, state.gimbal.yaw) / dt
        <= LIMITS.max_gimbal_rate_deg_s + 1e-12
    )


def test_hover_keeps_anchor_as_desired_position() -> None:
    anchor = Vec3(3.0, 4.0, 40.0)

    command = command_baseline(
        "hover",
        stationary(anchor),
        target_position=Vec3(25.0, 30.0, 0.0),
        hover_position=anchor,
        altitude_m=40.0,
        limits=LIMITS,
        dt=0.05,
    )

    assert command.policy == "hover"
    assert command.state.position == anchor


def test_vertical_follow_uses_target_horizontal_position_and_relative_altitude() -> None:
    target = Vec3(25.0, -10.0, 2.0)
    permissive_limits = UavLimits(
        max_speed_mps=100.0,
        max_acceleration_mps2=100.0,
        max_jerk_mps3=100.0,
        max_gimbal_rate_deg_s=360.0,
    )

    command = command_baseline(
        "vertical_follow",
        stationary(),
        target_position=target,
        hover_position=Vec3(0.0, 0.0, 40.0),
        altitude_m=40.0,
        limits=permissive_limits,
        dt=1.0,
    )

    assert command.policy == "vertical_follow"
    assert command.state.position == Vec3(25.0, -10.0, 42.0)


def test_vertical_follow_gimbal_tracks_from_reachable_position_not_unreached_goal() -> None:
    command = command_baseline(
        "vertical_follow",
        stationary(),
        target_position=Vec3(25.0, -10.0, 2.0),
        hover_position=Vec3(0.0, 0.0, 40.0),
        altitude_m=40.0,
        limits=LIMITS,
        dt=0.05,
    )

    assert command.state.position.x < 25.0
    assert command.state.gimbal.pitch > -90.0


def test_command_baseline_rejects_unknown_policy() -> None:
    with pytest.raises(ValueError, match="unsupported baseline policy: orbit"):
        command_baseline(
            "orbit",
            stationary(),
            target_position=Vec3(0.0, 0.0, 0.0),
            hover_position=Vec3(0.0, 0.0, 40.0),
            altitude_m=40.0,
            limits=LIMITS,
            dt=0.05,
        )
