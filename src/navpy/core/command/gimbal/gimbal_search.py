"""The search sweep: a profile-defined gimbal scan that runs only while nothing is detected (HITL step 21).

Inputs are camera state only: the newest SIYI attitude readback (body-relative joint angles, gimbal capture
history on the capture clock), the frame tick and the detector's own detections. No dock, aircraft pose, heading,
altitude or ground speed is read, so the scan is the same in every scenario.

It writes through the mount's gimbal actuator, the same fenced SIYI writer as rate tracking (the result handoff's
SiyiCommandFence, and the stage-3 envelope that will wrap that writer). Each tick runs under the gimbal command
gate, so it never interleaves with a detection, geo or loss-recovery transaction.

The scan stops (one zero rate, only if its own last command moved the gimbal) on: a fresh detection this tick or
within ``clear_s``; a session that tracks or a geo session; the guidance zoom freeze; frames older than
``frame_max_age_s``; readback older than ``readback_max_age_s`` or missing; and revoke (the command fence closed),
which latches and writes nothing more: the fence's own stop covers the gimbal. A detection or geo session start
yields first (``yield_to``), so its commands never follow a moving scan. Each (re)start first commands the widest
zoom step, as step 18's loss policy does, and moves only once that command went out (a failed start is retried every
``START_RETRY_S``); then it selects FOLLOW mode, so the readback is body-relative.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.core.command.gimbal.gimbal_search_pattern import GimbalSearchPattern
from navpy.core.command.gimbal.gimbal_tracking_constants import GIMBAL_COMMAND_ERRORS, MODE_FOLLOW
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory
from navpy.core.ports.gimbal_hardware_ports import GimbalActuatorPort

# The SIYI speed command is whole percent of the slew rate; a smaller nonzero rate would round to a stop.
MIN_COMMAND_UNITS = 1.0
# A start whose widest-zoom or FOLLOW command failed is retried at this interval, not on every frame.
START_RETRY_S = 1.0


@dataclass(frozen=True)
class GimbalSearchSetup:
    """The profile's pattern and the run's revoke source (the SIYI command fence's ``closed``)."""

    pattern: GimbalSearchPattern
    revoked: Callable[[], bool]


@dataclass
class _ScanState:
    scanning: bool = False
    target: int = 0
    command: tuple[float, float] = (0.0, 0.0)    # the scan's own last rate; (0, 0) when it holds nothing
    last_frame_s: float | None = None
    last_seen_s: float | None = None
    last_start_s: float | None = None
    revoked: bool = False
    reason: str = "idle"
    writes: int = 0
    stops: list[str] = field(default_factory=list)


class GimbalSearch:
    """One mount's scan. ``tick`` and ``yield_to`` run under the gimbal command gate."""

    def __init__(
        self,
        setup: GimbalSearchSetup,
        gimbal: GimbalActuatorPort,
        history: Callable[[], GimbalCaptureHistory | None],
        widest_zoom: Callable[[], bool],
        max_slew_dps: float,
        logger: ILogger,
    ) -> None:
        self._pattern = setup.pattern
        self._targets = setup.pattern.targets
        self._revoked = setup.revoked
        self._gimbal = gimbal
        self._history = history
        self._widest_zoom = widest_zoom
        self._max_slew_dps = float(max_slew_dps)
        self._logger = logger
        self._state = _ScanState()

    @property
    def scanning(self) -> bool:
        return self._state.scanning

    @property
    def reason(self) -> str:
        """Why the scan is not moving ("scanning" while it is)."""
        return self._state.reason

    @property
    def writes(self) -> int:
        return self._state.writes

    def tick(self, blocked: str | None, seen: bool, new_frame: bool) -> None:
        """One tracking tick. ``blocked`` names a session that owns the gimbal; ``seen`` is any fresh detection."""
        state, now = self._state, rt.perf_counter()
        if new_frame:
            state.last_frame_s = now
        if seen:
            state.last_seen_s = now
        if state.revoked or self._revoked():
            self._revoke()
            return
        reason = blocked or self._idle_reason(now, seen)
        if reason is not None:
            self._halt(reason)
            return
        sample = self._readback(now)
        if sample is None:
            self._halt("readback_stale")
            return
        if not state.scanning and not self._start(now):
            return
        if new_frame:
            self._steer(sample)

    def yield_to(self, owner: str) -> None:
        """A detection or geo session takes the gimbal: stop the scan before its first command."""
        if self._state.revoked or self._revoked():
            self._revoke()
            return
        self._halt(owner)

    def _idle_reason(self, now: float, seen: bool) -> str | None:
        state, timing = self._state, self._pattern.timing
        if seen or (state.last_seen_s is not None and now - state.last_seen_s < timing.clear_s):
            return "detected"
        if state.last_frame_s is None or now - state.last_frame_s > timing.frame_max_age_s:
            return "frames_stale"
        return None

    def _readback(self, now: float) -> tuple[float, float] | None:
        history = self._history()
        if history is None or not history.samples:
            return None
        sample = history.samples[-1]
        age = now - sample.receipt_s
        if not 0.0 <= age <= self._pattern.timing.readback_max_age_s:
            return None
        yaw, pitch = float(sample.att.yaw), float(sample.att.pitch)
        return (yaw, pitch) if math.isfinite(yaw) and math.isfinite(pitch) else None

    def _start(self, now: float) -> bool:
        state = self._state
        if state.last_start_s is not None and now - state.last_start_s < START_RETRY_S:
            return False
        state.last_start_s = now
        if not self._widest_zoom():
            self._halt("zoom_failed")
            return False
        try:
            self._gimbal.set_motion_mode(MODE_FOLLOW)
        except GIMBAL_COMMAND_ERRORS as exc:
            self._warn(f"search FOLLOW failed: {exc}")
            return False
        state.last_start_s = None
        state.scanning, state.target, state.reason = True, 0, "scanning"
        self._info(f"search start ({len(self._targets)} targets)")
        return True

    def _steer(self, readback: tuple[float, float]) -> None:
        state, tolerance = self._state, self._pattern.tolerance_deg
        error = (0.0, 0.0)
        for _ in range(len(self._targets)):
            target = self._targets[state.target]
            error = (target[0] - readback[0], target[1] - readback[1])
            if max(abs(error[0]), abs(error[1])) > tolerance:
                break
            state.target = (state.target + 1) % len(self._targets)
        distance = math.hypot(*error)
        speed = min(self._pattern.rate_dps, distance / self._pattern.approach_s)
        units = (0.0, 0.0) if distance <= 0 else tuple(
            _axis_units(speed * axis / distance, axis, self._max_slew_dps, tolerance) for axis in error)
        self._send(units[0], units[1])

    def _halt(self, reason: str) -> None:
        state = self._state
        if state.scanning or state.reason != reason:
            self._info(f"search stop ({reason})")
            state.stops.append(reason)
        state.scanning, state.reason = False, reason
        if state.command != (0.0, 0.0):
            self._send(0.0, 0.0)

    def _revoke(self) -> None:
        state = self._state
        if not state.revoked:
            self._info("search revoked: no further search writes")
            state.stops.append("revoked")
        state.revoked, state.scanning, state.reason = True, False, "revoked"

    def _send(self, yaw_units: float, pitch_units: float) -> None:
        state = self._state
        try:
            self._gimbal.set_rate(yaw_units, pitch_units)
        except GIMBAL_COMMAND_ERRORS as exc:
            state.command = (math.nan, math.nan)     # unknown: the next stop resends zero
            self._warn(f"search rate failed: {exc}")
            return
        state.writes += 1
        state.command = (float(yaw_units), float(pitch_units))

    def _info(self, message: str) -> None:
        self._logger.info(f"GimbalSearch: {message}")

    def _warn(self, message: str) -> None:
        self._logger.warning(f"GimbalSearch: {message}")


def _axis_units(dps: float, error: float, max_slew_dps: float, tolerance_deg: float) -> float:
    """One axis's rate command in SIYI units (percent of the max slew), at least one unit outside the tolerance."""
    units = max(-100.0, min(100.0, dps / max_slew_dps * 100.0))
    if abs(error) > tolerance_deg and abs(units) < MIN_COMMAND_UNITS:
        return math.copysign(MIN_COMMAND_UNITS, error)
    return units


__all__ = ["GimbalSearch", "GimbalSearchSetup", "MIN_COMMAND_UNITS", "START_RETRY_S"]
