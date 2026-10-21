"""Unreal math for offline checks: FRotator / FQuat as UE 5.7 UnrealMath.cpp computes them (double
precision), the gimbal composition of NavPyGimbalCameraActor and the capture slot grid."""
from __future__ import annotations

import math
from typing import Sequence

SINGULARITY_THRESHOLD = 0.4999995     # FQuat4d::Rotator (UnrealMath.cpp)

Quat = tuple[float, float, float, float]          # x, y, z, w (Unreal FQuat order)
Vec = tuple[float, float, float]


# --- Unreal math (FRotator / FQuat, UE 5.7 UnrealMath.cpp, double precision) -----------------------------------------
def rotator_to_quat(pitch: float, yaw: float, roll: float) -> Quat:
    """FRotator::Quaternion()."""
    sp, cp = math.sin(math.radians(pitch) / 2), math.cos(math.radians(pitch) / 2)
    sy, cy = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
    sr, cr = math.sin(math.radians(roll) / 2), math.cos(math.radians(roll) / 2)
    return (cr * sp * sy - sr * cp * cy, -cr * sp * cy - sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_to_rotator(q: Quat) -> tuple[float, float, float]:
    """FQuat4d::Rotator(): (pitch, yaw, roll) in degrees; at a pole the roll is 0 and the yaw takes the rest."""
    x, y, z, w = q
    singularity = z * x - w * y
    if singularity < -SINGULARITY_THRESHOLD:
        return -90.0, _normalize_axis(-2.0 * math.degrees(math.atan2(x, w))), 0.0
    if singularity > SINGULARITY_THRESHOLD:
        return 90.0, _normalize_axis(2.0 * math.degrees(math.atan2(x, w))), 0.0
    return (math.degrees(math.asin(2 * singularity)), math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
            math.degrees(math.atan2(-2 * (w * x + y * z), 1 - 2 * (x * x + y * y))))


def _normalize_axis(angle: float) -> float:
    """FRotator::NormalizeAxis: (-180, 180]."""
    angle = math.fmod(angle, 360.0)
    angle = angle + 360.0 if angle < 0 else angle
    return angle - 360.0 if angle > 180.0 else angle


def quat_mul(a: Quat, b: Quat) -> Quat:
    """a * b: rotation b, then a (FQuat::operator*)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q: Quat, v: Sequence[float]) -> Vec:
    x, y, z, _w = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (x, y, z)


def compose_gimbal(vehicle: Quat, yaw: float, pitch: float, roll: float, zero_world_roll: bool) -> Quat:
    """NavPyComposeGimbalWorldRotation, then FRotator -> FQuat as SetWorldRotation does."""
    p, y, r = quat_to_rotator(quat_mul(vehicle, rotator_to_quat(pitch, yaw, roll)))
    return rotator_to_quat(p, y, 0.0 if zero_world_roll else r)


def slot_time_us(origin_us: int, slot: int, fps: float) -> int:
    """FNavPyBoardTimeInterpolator::SlotTimeUs: origin + RoundToDouble(k * 1e6 / F) (round half up)."""
    return origin_us + math.floor(slot * 1.0e6 / fps + 0.5)
