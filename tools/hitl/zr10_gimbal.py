"""The emulated ZR10's motion: navpy's gimbal plant on a 20 ms timeline, what Unreal renders and what the wire reports.

The plant is navpy's deterministic sim plant (gimbal_angles.py, gimbal_zoom_plant.py,
gimbal_frame_transform.body_readback), so the emulator and navpy's sim share their dynamics. Added here: 0x07 dead
time and deadband, report lag (0x0D and 0x18 apart), report scale (hitl_final_approach.md section 4).

Angles. The plant holds the camera's rotation; body_readback gives it relative to the aircraft as a yaw-outer
gimbal (body = Rz(yaw) Ry(pitch) Rx(roll), navpy's SIYI convention, siyi_convention.py), the order Unreal composes
FRotator angles in with the vehicle attitude.

The wire reports yaw left-positive: report yaw = -scale * body yaw (design 3, "Signs"). Pitch is up-positive.
Roll keeps the plant's sign (TBC: the real unit reported roll 0.00 throughout). What Unreal renders
(``true_state``) is the same body rotation without report lag or scale (yaw right, pitch up, roll right wing down:
the vehicle_state convention), the zoom reading and the lens setting: how the lens got there (the last accepted 0x0F
command, none after a manual zoom: no lens row exists for an arbitrary stop) and whether it has settled. Unreal picks
its lens row by them (hitl_final_approach.md section 9, step 5); the wire carries neither. The 1.0 wide stop is not
modelled: the plant's floor is the table's lowest reading (1.1).

Ordering: with a 0x07 dead time each rate command (a stop included) waits its own dead time in arrival order;
``cancel_rates`` drops the ones still waiting.

Mechanical stops (owner, 2026-10-16: as the real gimbal). The camera cannot pass the body limits: what Unreal
renders and what the wire reports are the plant's body angles with yaw and pitch clamped to the limits (roll is not
clamped: its stops are unmeasured). navpy's plant clamps world-angle differences, equal to body angles only for a
level aircraft, so banked the plant can ask for a pose past a stop; the camera then sits at the stop and returns
when the aircraft does, the plant's target unchanged. Whether the real unit holds or slides its lock target at a
stop is unmeasured. Steps at a stop are counted, not treated as a fault.

Nothing moves before the first aircraft attitude, because the plant's lock stabilisation and body limits need it.
The plant then starts at the body centre.
"""
from __future__ import annotations

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
from navpy.sim.gimbal.siyi.gimbal_frame_transform import body_readback
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import GimbalZoomLimits, GimbalZoomPlant
from tools.hitl.zr10_config import STEP_S, EmulatorConfig

# Float noise allowed before a step counts as at a stop.
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
    """What Unreal renders: the body-relative camera rotation as FRotator angles, the zoom reading and the lens
    setting (the absolute zoom command it was set by, None for none, and whether travel has finished)."""

    yaw: float
    pitch: float
    roll: float
    zoom: float
    motion_mode: int
    lens_command: float | None
    lens_settled: bool


def plant_body_angles(snapshot: GimbalAngularSnapshot) -> tuple[float, float, float]:
    """The plant's camera rotation relative to the aircraft as yaw-outer angles (yaw, pitch, roll).

    Lock and follow only: the device refuses FPV, whose raw plant angles body_readback does not convert.
    """
    if snapshot.motion_mode == MODE_FPV:
        raise ValueError("FPV is not modelled")
    body = body_readback(snapshot)
    return body.yaw, body.pitch, body.roll


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
        # The reading of navpy's start-up "zoom 1" command, else the lowest reading (set by nothing known).
        self.zoom = GimbalZoomPlant(zoom_readings.get(10, min(readings)), self._zoom_limits)
        self._lens_command_tenths: int | None = 10 if 10 in zoom_readings else None
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

    # -- zoom commands -----------------------------------------------------------------------------------------
    def seek_zoom(self, command_tenths: int, level: float) -> None:
        """An accepted 0x0F absolute zoom: the lens travels to ``level`` and is then set by that command."""
        self.zoom.set_target(level)
        self._lens_command_tenths = command_tenths

    def manual_zoom(self, direction: int) -> None:
        """A 0x05 manual zoom start (+1 in, -1 out) or stop (0): the lens is set by no known setting from now on."""
        if direction:
            self.zoom.start(direction)
        else:
            self.zoom.stop()
        self._lens_command_tenths = None

    # -- what is reported and what is rendered --------------------------------------------------------------
    def camera_angles(self, snapshot: GimbalAngularSnapshot) -> tuple[float, float, float]:
        """The camera's body angles (yaw, pitch, roll): the plant's, yaw and pitch held at the stops."""
        yaw, pitch, roll = plant_body_angles(snapshot)
        limits = self.angular_limits
        return (min(max(yaw, limits.min_yaw), limits.max_yaw), min(max(pitch, limits.min_pitch), limits.max_pitch),
                roll)

    def at_stop(self) -> bool:
        """True when the plant asks for a pose past a stop, so the camera is held there."""
        yaw, pitch, _roll = plant_body_angles(self.plant.snapshot())
        limits, tol = self.angular_limits, BODY_LIMIT_TOLERANCE_DEG
        return not (limits.min_yaw - tol <= yaw <= limits.max_yaw + tol
                    and limits.min_pitch - tol <= pitch <= limits.max_pitch + tol)

    def true_state(self) -> TrueState | None:
        if self._angular is None:
            return None
        snapshot = self._angular.snapshot()
        yaw, pitch, roll = self.camera_angles(snapshot)
        lens = self.zoom.snapshot()
        command = None if self._lens_command_tenths is None else self._lens_command_tenths / 10.0
        return TrueState(yaw, pitch, roll, reading(lens.level), snapshot.motion_mode, command,
                         lens.target is None and lens.direction == 0)

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
        body_yaw, body_pitch, roll = self.camera_angles(self.plant.snapshot())
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


__all__ = ["Report", "TrueState", "Zr10Gimbal", "plant_body_angles", "reading"]
