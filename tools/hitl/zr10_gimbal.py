"""The emulated ZR10's motion: navpy's gimbal plant on a 20 ms timeline, what Unreal renders and what the wire reports.

The plant is navpy's deterministic sim plant (gimbal_angles.py, gimbal_zoom_plant.py,
gimbal_frame_transform.body_readback), so the emulator and navpy's sim share their dynamics. Added here: 0x07 dead
time and deadband, report lag (0x0D and 0x18 apart), report scale (hitl_final_approach.md section 4).

Angles. The plant holds the camera's rotation; body_readback gives it relative to the aircraft. It is decomposed
here as a yaw-outer gimbal: yaw about the body yaw axis, then pitch, then roll (body = Rz(yaw) Ry(pitch) Rx(roll),
``body_zyx_angles``). The rig's camera model is the same chain (gimbal-controller geometry.py
commands_from_pixels: a yaw turn after a pitch turn), and Unreal composes FRotator angles the same way with the
vehicle attitude. navpy's own readback decomposes roll, then pitch, then yaw (matrix_to_xyz, g_seq "XYZ"), which
differs once yaw and pitch are both non-zero: that is navpy's SIYI conversion to fix (design 3, "Signs"), not
hidden here.

The wire reports yaw left-positive: report yaw = -scale * body yaw (design 3, "Signs"). Pitch is up-positive.
Roll keeps the plant's sign (TBC: the real unit reported roll 0.00 throughout). What Unreal renders
(``true_state``) is the same body rotation without report lag or scale (yaw right, pitch up, roll right wing down:
the vehicle_state convention), and the zoom reading.

Ordering: with a 0x07 dead time each rate command (a stop included) waits its own dead time in arrival order;
``cancel_rates`` drops the ones still waiting.

Nothing moves before the first aircraft attitude, because the plant's lock stabilisation and body limits need it.
The plant then starts at the body centre.
"""
from __future__ import annotations

import math
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass

from navpy.core.kernel.attitude import Attitude
from navpy.sim.gimbal.siyi.gimbal_angles import (
    MODE_FPV,
    GimbalAngularLimits,
    GimbalAngularPlant,
    GimbalAngularSnapshot,
    wrap180,
)
from navpy.sim.gimbal.siyi.gimbal_frame_transform import body_readback, zyx_to_matrix
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import GimbalZoomLimits, GimbalZoomPlant
from tools.hitl.zr10_config import STEP_S, EmulatorConfig

# Float noise allowed when the body angles are checked against the rig limits.
BODY_LIMIT_TOLERANCE_DEG = 0.05
_EPS_S = 1e-9


@dataclass(frozen=True)
class Report:
    """What the wire reports for one instant (after scale; yaw left-positive)."""

    t_s: float
    yaw: float
    pitch: float
    roll: float
    yaw_rate: float
    pitch_rate: float
    roll_rate: float
    zoom: float


@dataclass(frozen=True)
class TrueState:
    """What Unreal renders: the body-relative camera rotation as FRotator angles, and the zoom reading."""

    yaw: float
    pitch: float
    roll: float
    zoom: float
    motion_mode: int


def body_zyx_angles(body: Attitude) -> tuple[float, float, float]:
    """navpy's roll-pitch-yaw body readback -> the same rotation as yaw, then pitch, then roll (yaw, pitch, roll)."""
    matrix = (zyx_to_matrix(0.0, 0.0, body.roll) @ zyx_to_matrix(0.0, body.pitch, 0.0)
              @ zyx_to_matrix(body.yaw, 0.0, 0.0))
    yaw = math.degrees(math.atan2(matrix[1, 0], matrix[0, 0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(matrix[2, 0])))))
    roll = math.degrees(math.atan2(matrix[2, 1], matrix[2, 2]))
    return yaw, pitch, roll


def plant_body_angles(snapshot: GimbalAngularSnapshot) -> tuple[float, float, float]:
    """The plant's camera rotation relative to the aircraft as yaw-outer angles (yaw, pitch, roll).

    Lock and follow only: the device refuses FPV, whose raw plant angles body_readback does not convert.
    """
    if snapshot.motion_mode == MODE_FPV:
        raise ValueError("FPV is not modelled")
    return body_zyx_angles(body_readback(snapshot))


def reading(level: float) -> float:
    """The 0x18 reading: the lens level in exact 0.1 steps (the wire carries tenths, siyi_sdk.py:687)."""
    return round(level * 10.0) / 10.0


class Zr10Gimbal:
    def __init__(self, config: EmulatorConfig, zoom_readings: Mapping[int, float]) -> None:
        self.config = config
        readings = zoom_readings.values()
        self._zoom_limits = GimbalZoomLimits(
            min_level=min(readings),
            max_level=max(readings),
            seek_speed=config.absolute_zoom_speed_per_s,
            incremental_speed=config.manual_zoom_speed_per_s,
        )
        self.angular_limits = GimbalAngularLimits(
            max_slew_rate=config.slew_rate_deg_s,
            min_yaw=config.yaw_min_deg,
            max_yaw=config.yaw_max_deg,
            min_pitch=config.pitch_min_deg,
            max_pitch=config.pitch_max_deg,
        )
        self._angular: GimbalAngularPlant | None = None
        # The reading of navpy's start-up "zoom 1" command, else the lowest reading.
        self.zoom = GimbalZoomPlant(zoom_readings.get(10, min(readings)), self._zoom_limits)
        self._steps = 0
        self._time_s = 0.0
        self._pending_rates: deque[tuple[float, int, int]] = deque()
        self._history: deque[Report] = deque()

    # -- time and aircraft ------------------------------------------------------------------------------
    @property
    def now_s(self) -> float:
        return self._time_s

    @property
    def has_attitude(self) -> bool:
        return self._angular is not None

    @property
    def plant(self) -> GimbalAngularPlant:
        assert self._angular is not None
        return self._angular

    def set_vehicle_attitude(self, attitude: Attitude) -> None:
        """The aircraft attitude (NED degrees) from the bridge's vehicle_state; the first one starts the plant."""
        if self._angular is None:
            # Lock mode holds world angles: the body centre is the aircraft's own yaw and pitch.
            self._angular = GimbalAngularPlant(attitude.pitch, attitude.yaw, self.angular_limits)
            self._angular.set_vehicle_attitude(attitude)
            self._record_history()
            return
        self._angular.set_vehicle_attitude(attitude)

    def step(self) -> None:
        """Advance to the next 20 ms step boundary."""
        self._steps += 1
        self._integrate_to(self._steps * STEP_S)

    def advance_to(self, t_s: float) -> None:
        """Advance to ``t_s`` inside the current step, before a command handled at that time."""
        self._integrate_to(min(t_s, (self._steps + 1) * STEP_S - _EPS_S))

    def _integrate_to(self, t_s: float) -> None:
        if t_s <= self._time_s:
            return
        angular = self._angular
        if angular is None:
            self._time_s = t_s
            return
        while self._pending_rates and self._pending_rates[0][0] <= t_s:
            due_s, yaw_speed, pitch_speed = self._pending_rates.popleft()
            self._advance_plants(due_s)
            angular.set_speed(yaw_speed, pitch_speed)
        self._advance_plants(t_s)
        self._record_history()

    def _advance_plants(self, t_s: float) -> None:
        dt_s = t_s - self._time_s
        if dt_s > 0.0:
            self.plant.advance(dt_s)
            self.zoom.advance(dt_s)
            self._time_s = t_s

    def _record_history(self) -> None:
        self._history.append(self._report_now(self._history[-1] if self._history else None))
        keep_s = max(self.config.attitude_report_lag_s, self.config.zoom_report_lag_s) + 2 * STEP_S
        while len(self._history) > 1 and self._history[1].t_s <= self.now_s - keep_s:
            self._history.popleft()

    # -- rate commands -----------------------------------------------------------------------------------
    def set_rate(self, yaw_speed: int, pitch_speed: int) -> None:
        """A 0x07 rate (-100..100 each) after the deadband, now or after the dead time."""
        yaw_speed, pitch_speed = (self._deadband(max(-100, min(100, s))) for s in (yaw_speed, pitch_speed))
        if self.config.rate_dead_time_s > 0.0:
            self._pending_rates.append((self.now_s + self.config.rate_dead_time_s, yaw_speed, pitch_speed))
        else:
            self.plant.set_speed(yaw_speed, pitch_speed)

    def cancel_rates(self) -> None:
        self._pending_rates.clear()

    def _deadband(self, speed: int) -> int:
        return 0 if abs(speed) <= self.config.rate_deadband else speed

    # -- what is reported and what is rendered --------------------------------------------------------------
    def within_body_limits(self) -> bool:
        """False when the camera is outside the rig's body limits.

        navpy's plant clamps world-angle differences, which equal the body angles only for a level aircraft;
        banked, it can point where the real gimbal hits its stop.
        """
        yaw, pitch, _roll = plant_body_angles(self.plant.snapshot())
        limits, tol = self.angular_limits, BODY_LIMIT_TOLERANCE_DEG
        return (limits.min_yaw - tol <= yaw <= limits.max_yaw + tol
                and limits.min_pitch - tol <= pitch <= limits.max_pitch + tol)

    def true_state(self) -> TrueState | None:
        if self._angular is None:
            return None
        snapshot = self._angular.snapshot()
        yaw, pitch, roll = plant_body_angles(snapshot)
        return TrueState(yaw, pitch, roll, reading(self.zoom.level), snapshot.motion_mode)

    def reported(self) -> Report | None:
        """The 0x0D angles and the 0x18 reading the wire carries now, each with its own report lag."""
        if not self._history:
            return None
        attitude = self._lagged(self.config.attitude_report_lag_s)
        zoom = self._lagged(self.config.zoom_report_lag_s)
        return Report(attitude.t_s, attitude.yaw, attitude.pitch, attitude.roll, attitude.yaw_rate,
                      attitude.pitch_rate, attitude.roll_rate, zoom.zoom)

    def _lagged(self, lag_s: float) -> Report:
        wanted_s = self.now_s - lag_s + _EPS_S
        chosen = self._history[0]
        for report in self._history:
            if report.t_s > wanted_s:
                break
            chosen = report
        return chosen

    def _report_now(self, previous: Report | None) -> Report:
        body_yaw, body_pitch, roll = plant_body_angles(self.plant.snapshot())
        yaw = -self.config.yaw_report_scale * body_yaw
        pitch = self.config.pitch_report_scale * body_pitch
        dt_s = 0.0 if previous is None else self.now_s - previous.t_s
        if previous is None or dt_s <= 0.0:
            rates = (0.0, 0.0, 0.0) if previous is None else (previous.yaw_rate, previous.pitch_rate,
                                                              previous.roll_rate)
        else:
            rates = (wrap180(yaw - previous.yaw) / dt_s, (pitch - previous.pitch) / dt_s,
                     wrap180(roll - previous.roll) / dt_s)
        return Report(self.now_s, yaw, pitch, roll, *rates, reading(self.zoom.level))


__all__ = ["Report", "TrueState", "Zr10Gimbal", "body_zyx_angles", "plant_body_angles", "reading"]
