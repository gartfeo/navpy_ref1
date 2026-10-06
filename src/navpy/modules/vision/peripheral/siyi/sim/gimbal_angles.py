"""Angular dynamics for the simulated SIYI gimbal."""

from __future__ import annotations

from dataclasses import dataclass, field

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.siyi.siyi_sdk import ZR10


MAX_SLEW_RATE = 90.0
ANGLE_SEEK_DEADBAND = 0.1
MODE_LOCK = 0
MODE_FOLLOW = 1
MODE_FPV = 2


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap180(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class GimbalAngularSnapshot:
    yaw: float
    pitch: float
    roll: float
    yaw_speed: float
    pitch_speed: float
    roll_speed: float
    motion_mode: int
    vehicle_attitude: Attitude
    yaw_command: float
    pitch_command: float
    target_yaw: float | None
    target_pitch: float | None


@dataclass
class _AngularState:
    yaw: float
    pitch: float
    roll: float = 0.0
    yaw_speed: float = 0.0
    pitch_speed: float = 0.0
    roll_speed: float = 0.0
    yaw_command: float = 0.0
    pitch_command: float = 0.0
    target_yaw: float | None = None
    target_pitch: float | None = None
    motion_mode: int = MODE_LOCK
    vehicle_attitude: Attitude = field(
        default_factory=lambda: Attitude(0.0, 0.0, 0.0)
    )


class GimbalAngularPlant:
    """Own motor commands, stabilized angles, and mechanical limits."""

    def __init__(self, initial_pitch: float, initial_yaw: float) -> None:
        self._state = _AngularState(
            yaw=float(initial_yaw),
            pitch=float(initial_pitch),
        )

    def snapshot(self) -> GimbalAngularSnapshot:
        state = self._state
        return GimbalAngularSnapshot(
            state.yaw,
            state.pitch,
            state.roll,
            state.yaw_speed,
            state.pitch_speed,
            state.roll_speed,
            state.motion_mode,
            state.vehicle_attitude,
            state.yaw_command,
            state.pitch_command,
            state.target_yaw,
            state.target_pitch,
        )

    def set_speed(self, yaw_command: float, pitch_command: float) -> None:
        state = self._state
        state.yaw_command = clamp(float(yaw_command), -100.0, 100.0)
        state.pitch_command = clamp(float(pitch_command), -100.0, 100.0)
        state.target_yaw = None
        state.target_pitch = None

    def set_target_angles(self, yaw_deg: float, pitch_deg: float) -> None:
        state = self._state
        state.target_yaw = (
            wrap180(float(yaw_deg))
            if state.motion_mode == MODE_LOCK
            else clamp(float(yaw_deg), ZR10.MIN_YAW_DEG, ZR10.MAX_YAW_DEG)
        )
        state.target_pitch = clamp(
            float(pitch_deg),
            ZR10.MIN_PITCH_DEG,
            ZR10.MAX_PITCH_DEG,
        )
        state.yaw_command = 0.0
        state.pitch_command = 0.0

    def set_motion_mode(self, mode: int) -> None:
        if mode not in (MODE_LOCK, MODE_FOLLOW, MODE_FPV):
            return
        state = self._state
        previous = state.motion_mode
        if previous == mode:
            return
        if previous != MODE_LOCK and mode == MODE_LOCK:
            state.yaw = wrap180(state.yaw + state.vehicle_attitude.yaw)
        elif previous == MODE_LOCK and mode != MODE_LOCK:
            state.yaw = wrap180(state.yaw - state.vehicle_attitude.yaw)
        if state.target_yaw is not None:
            state.target_yaw = None
            state.target_pitch = None
            state.yaw_command = 0.0
            state.pitch_command = 0.0
        state.motion_mode = mode

    def set_vehicle_attitude(self, attitude: Attitude) -> None:
        self._state.vehicle_attitude = attitude

    def advance(self, dt_s: float) -> None:
        if dt_s <= 0.0:
            return
        state = self._state
        self._enforce_limits()
        old_yaw, old_pitch = state.yaw, state.pitch
        if state.target_yaw is not None and state.target_pitch is not None:
            self._advance_target(dt_s)
        else:
            state.yaw += state.yaw_command / 100.0 * MAX_SLEW_RATE * dt_s
            state.pitch += state.pitch_command / 100.0 * MAX_SLEW_RATE * dt_s
        if state.motion_mode == MODE_LOCK:
            state.yaw = wrap180(state.yaw)
        self._enforce_limits()
        yaw_delta = state.yaw - old_yaw
        if state.motion_mode == MODE_LOCK:
            yaw_delta = wrap180(yaw_delta)
        state.yaw_speed = yaw_delta / dt_s
        state.pitch_speed = (state.pitch - old_pitch) / dt_s
        state.roll_speed = 0.0

    def _advance_target(self, dt_s: float) -> None:
        state = self._state
        assert state.target_yaw is not None
        assert state.target_pitch is not None
        yaw_error = state.target_yaw - state.yaw
        if state.motion_mode == MODE_LOCK:
            yaw_error = wrap180(yaw_error)
        pitch_error = state.target_pitch - state.pitch
        maximum_change = MAX_SLEW_RATE * dt_s
        yaw_change = clamp(yaw_error, -maximum_change, maximum_change)
        pitch_change = clamp(pitch_error, -maximum_change, maximum_change)
        state.yaw += yaw_change
        state.pitch += pitch_change
        state.yaw_command = 100.0 * yaw_change / maximum_change
        state.pitch_command = 100.0 * pitch_change / maximum_change
        if abs(yaw_error) <= maximum_change and abs(pitch_error) <= maximum_change:
            state.target_yaw = None
            state.target_pitch = None
            state.yaw_command = 0.0
            state.pitch_command = 0.0

    def _enforce_limits(self) -> None:
        state = self._state
        attitude = state.vehicle_attitude
        if state.motion_mode == MODE_LOCK:
            body_yaw = wrap180(state.yaw - attitude.yaw)
            state.yaw = wrap180(
                clamp(body_yaw, ZR10.MIN_YAW_DEG, ZR10.MAX_YAW_DEG)
                + attitude.yaw
            )
        else:
            state.yaw = clamp(
                state.yaw,
                ZR10.MIN_YAW_DEG,
                ZR10.MAX_YAW_DEG,
            )
        body_pitch = state.pitch - attitude.pitch
        state.pitch = clamp(
            body_pitch,
            ZR10.MIN_PITCH_DEG,
            ZR10.MAX_PITCH_DEG,
        ) + attitude.pitch


__all__ = [
    "ANGLE_SEEK_DEADBAND",
    "GimbalAngularPlant",
    "GimbalAngularSnapshot",
    "MAX_SLEW_RATE",
    "MODE_FOLLOW",
    "MODE_FPV",
    "MODE_LOCK",
    "clamp",
    "wrap180",
]
