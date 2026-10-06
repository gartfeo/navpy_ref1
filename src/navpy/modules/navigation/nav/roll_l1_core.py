"""Core Roll-L1 and pitch policies independent of legacy composition."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from navpy.args.pid_args import PIDArgs
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.nav.pid.pid import Pid
from navpy.modules.navigation.nav.pn.pitch_pn import PitchPn
from navpy.modules.navigation.nav.roll_l1 import RollL1Control
from navpy.modules.navigation.nav.roll_l1_inputs import (
    PitchLockSettings,
    RollL1AirframeLimits,
    RollL1AttitudeSnapshot,
)


class PitchController(ABC):
    @abstractmethod
    def calc(
        self,
        poi_ned: np.ndarray,
        pitch_error: float,
        current_pitch: float,
    ) -> float:
        raise NotImplementedError

    @abstractmethod
    def reset(self, min_pitch: float, max_pitch: float) -> None:
        raise NotImplementedError


class PitchPidController(PitchController):
    def __init__(
        self,
        args: PIDArgs,
        min_pitch: float,
        max_pitch: float,
    ) -> None:
        self._pid = Pid(args, min_pitch, max_pitch)

    def calc(
        self,
        poi_ned: np.ndarray,
        pitch_error: float,
        current_pitch: float,
    ) -> float:
        del current_pitch
        n, e, d = poi_ned
        los_rad = math.atan2(d, math.hypot(n, e))
        feedforward = -math.degrees(los_rad)
        correction = self._pid.calc(pitch_error)
        return feedforward + correction

    def reset(self, min_pitch: float, max_pitch: float) -> None:
        self._pid.reset(min_pitch, max_pitch)


class PitchPnController(PitchController):
    def __init__(
        self,
        kp: Callable[[], float],
        min_pitch: float,
        max_pitch: float,
    ) -> None:
        self._kp = kp
        self._pitch_pn = PitchPn(
            kp=self._kp(),
            out_min=min_pitch,
            out_max=max_pitch,
        )

    def calc(
        self,
        poi_ned: np.ndarray,
        pitch_error: float,
        current_pitch: float,
    ) -> float:
        return self._pitch_pn.calc(poi_ned, pitch_error, current_pitch)

    def reset(self, min_pitch: float, max_pitch: float) -> None:
        self._pitch_pn.reset(
            kp=self._kp(),
            out_min=min_pitch,
            out_max=max_pitch,
        )


def create_pitch_controller(
    controller_name: str,
    pitch_args: PIDArgs,
    kp: Callable[[], float],
    min_pitch: float,
    max_pitch: float,
) -> PitchController:
    if controller_name == "pn":
        return PitchPnController(kp, min_pitch, max_pitch)
    if controller_name == "pid":
        return PitchPidController(pitch_args, min_pitch, max_pitch)
    raise ValueError(f"Unknown pitch controller: {controller_name!r}")


class PitchLockPolicy:
    def __init__(
        self,
        settings: Callable[[], PitchLockSettings],
    ) -> None:
        self._settings = settings
        self._lock_pitch = True
        self._pitch_lock_released = False

    def reset(self) -> None:
        self._lock_pitch = True
        self._pitch_lock_released = False

    def should_lock(
        self,
        cmd_pitch: float,
        distance: float | None,
        roll_error_unlocked: float,
        roll_error_locked: float,
    ) -> bool:
        if self._pitch_lock_released:
            return False
        settings = self._settings()
        roll_error = (
            max(roll_error_locked, roll_error_unlocked)
            if self._lock_pitch
            else roll_error_unlocked
        )
        is_rolling = (
            settings.roll_difference_deg < 0
            or roll_error > settings.roll_difference_deg
        )
        should_lock = (
            cmd_pitch > settings.final_approach_angle_deg and is_rolling
        )
        far_enough = (
            settings.lock_distance_m < 0
            or bool(distance and distance > settings.lock_distance_m)
        )
        if should_lock and far_enough:
            self._lock_pitch = True
        else:
            if self._lock_pitch:
                self._pitch_lock_released = True
            self._lock_pitch = False
        return self._lock_pitch


class RollL1PitchNav:
    """L1 roll with explicit aircraft, controller, and policy ports."""

    def __init__(
        self,
        roll_l1: RollL1Control,
        pitch_controller: PitchController,
        pitch_lock: PitchLockPolicy,
        attitude: Callable[[], RollL1AttitudeSnapshot],
        limits: Callable[[], RollL1AirframeLimits],
        final_approach_throttle: Callable[[], float | None],
    ) -> None:
        self._roll_l1 = roll_l1
        self._pitch_controller = pitch_controller
        self._pitch_lock = pitch_lock
        self._attitude = attitude
        self._limits = limits
        self._final_approach_throttle = final_approach_throttle
        self._apply_limits(self._limits())

    def calc(
        self,
        prev_loc: Location,
        current_loc: Location,
        next_loc: Location,
        target_bearing_cd: float,
        poi_ned: np.ndarray,
        pitch_error: float,
        distance: float | None,
    ) -> tuple[float, float, float | None]:
        attitude = self._attitude()
        cmd_pitch = self._pitch_controller.calc(
            poi_ned,
            pitch_error,
            attitude.pitch_deg,
        )
        cmd_pitch = float(np.clip(cmd_pitch, self._pitch_min, self._pitch_max))
        self._roll_l1.update(
            prev_loc,
            next_loc,
            current_loc,
            target_bearing_cd,
        )
        predicted_roll = self._roll_l1.nav_roll(cmd_pitch)
        predicted_roll_lock = self._roll_l1.nav_roll(0)
        lock_pitch = self._pitch_lock.should_lock(
            cmd_pitch,
            distance,
            abs(predicted_roll - attitude.roll_deg),
            abs(predicted_roll_lock - attitude.roll_deg),
        )
        return (
            predicted_roll_lock if lock_pitch else predicted_roll,
            0.0 if lock_pitch else cmd_pitch,
            self._calc_cmd_thr(lock_pitch),
        )

    def reset(self) -> None:
        self._roll_l1.reset()
        self._apply_limits(self._limits())
        self._pitch_controller.reset(self._pitch_min, self._pitch_max)
        self._pitch_lock.reset()

    def _apply_limits(self, limits: RollL1AirframeLimits) -> None:
        self._pitch_min = limits.pitch_min_deg
        self._pitch_max = limits.pitch_max_deg
        self._throttle_min = limits.trim_throttle_percent

    def _calc_cmd_thr(self, lock_pitch: bool) -> float | None:
        configured = self._final_approach_throttle()
        if lock_pitch or configured is None:
            return None
        return max(self._throttle_min, configured)


__all__ = [
    "PitchController",
    "PitchLockPolicy",
    "PitchPidController",
    "PitchPnController",
    "RollL1PitchNav",
    "create_pitch_controller",
]
