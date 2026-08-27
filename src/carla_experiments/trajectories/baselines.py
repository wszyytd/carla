from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: Vec3) -> Vec3:
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vec3) -> Vec3:
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> Vec3:
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__


@dataclass(frozen=True)
class Euler:
    pitch: float
    yaw: float
    roll: float = 0.0


@dataclass(frozen=True)
class MotionState:
    position: Vec3
    velocity: Vec3
    acceleration: Vec3
    gimbal: Euler


@dataclass(frozen=True)
class UavLimits:
    max_speed_mps: float
    max_acceleration_mps2: float
    max_jerk_mps3: float
    max_gimbal_rate_deg_s: float


@dataclass(frozen=True)
class CameraCommand:
    state: MotionState
    policy: str


def norm(vector: Vec3) -> float:
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


def _clip_norm(vector: Vec3, maximum: float) -> Vec3:
    magnitude = norm(vector)
    if magnitude <= maximum or magnitude == 0.0:
        return vector
    return vector * (maximum / magnitude)


def angle_distance(first_deg: float, second_deg: float) -> float:
    return abs((second_deg - first_deg + 180.0) % 360.0 - 180.0)


def _signed_angle_delta(current_deg: float, desired_deg: float) -> float:
    return (desired_deg - current_deg + 180.0) % 360.0 - 180.0


def step_angle(current_deg: float, desired_deg: float, *, max_delta_deg: float) -> float:
    delta = _signed_angle_delta(current_deg, desired_deg)
    limited = max(-max_delta_deg, min(max_delta_deg, delta))
    return current_deg + limited


def look_at(camera: Vec3, target: Vec3) -> Euler:
    displacement = target - camera
    horizontal = math.hypot(displacement.x, displacement.y)
    pitch = math.degrees(math.atan2(displacement.z, horizontal))
    yaw = 0.0 if horizontal == 0.0 else math.degrees(math.atan2(displacement.y, displacement.x))
    return Euler(pitch=pitch, yaw=yaw)


def _step_gimbal(current: Euler, desired: Euler, maximum_delta_deg: float) -> Euler:
    pitch_delta = desired.pitch - current.pitch
    yaw_delta = _signed_angle_delta(current.yaw, desired.yaw)
    magnitude = math.hypot(pitch_delta, yaw_delta)
    if magnitude > maximum_delta_deg and magnitude > 0.0:
        scale = maximum_delta_deg / magnitude
        pitch_delta *= scale
        yaw_delta *= scale
    return Euler(
        pitch=current.pitch + pitch_delta,
        yaw=current.yaw + yaw_delta,
        roll=0.0,
    )


def advance_jerk_limited(
    state: MotionState,
    *,
    desired_position: Vec3,
    desired_gimbal: Euler,
    limits: UavLimits,
    dt: float,
) -> MotionState:
    """Advance one fixed step while enforcing translational and gimbal limits."""

    desired_velocity = _clip_norm(
        (desired_position - state.position) * (1.0 / dt),
        limits.max_speed_mps,
    )
    desired_acceleration = _clip_norm(
        (desired_velocity - state.velocity) * (1.0 / dt),
        limits.max_acceleration_mps2,
    )
    acceleration_delta = _clip_norm(
        desired_acceleration - state.acceleration,
        limits.max_jerk_mps3 * dt,
    )
    acceleration = _clip_norm(
        state.acceleration + acceleration_delta,
        limits.max_acceleration_mps2,
    )
    velocity = _clip_norm(state.velocity + acceleration * dt, limits.max_speed_mps)
    position = state.position + velocity * dt
    gimbal = _step_gimbal(
        state.gimbal,
        desired_gimbal,
        limits.max_gimbal_rate_deg_s * dt,
    )
    return MotionState(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        gimbal=gimbal,
    )


def command_baseline(
    policy: str,
    state: MotionState,
    *,
    target_position: Vec3,
    hover_position: Vec3,
    altitude_m: float,
    limits: UavLimits,
    dt: float,
) -> CameraCommand:
    """Compute one Hover or Vertical Follow camera command."""

    if policy == "hover":
        desired_position = hover_position
    elif policy == "vertical_follow":
        desired_position = Vec3(
            target_position.x,
            target_position.y,
            target_position.z + altitude_m,
        )
    else:
        raise ValueError(f"unsupported baseline policy: {policy}")

    translated_state = advance_jerk_limited(
        state,
        desired_position=desired_position,
        desired_gimbal=state.gimbal,
        limits=limits,
        dt=dt,
    )
    next_state = MotionState(
        position=translated_state.position,
        velocity=translated_state.velocity,
        acceleration=translated_state.acceleration,
        gimbal=_step_gimbal(
            state.gimbal,
            look_at(translated_state.position, target_position),
            limits.max_gimbal_rate_deg_s * dt,
        ),
    )
    return CameraCommand(state=next_state, policy=policy)
