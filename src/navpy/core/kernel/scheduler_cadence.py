"""Adapt NavPy scheduler frequency to accelerated ArduPilot SITL."""

from __future__ import annotations

import ctypes
import math
import os
import sys
import threading
import time
import weakref
from numbers import Real
from typing import Callable, Optional

_TIMER_PERIOD_MS = 1
_timer_resolution_lock = threading.RLock()
_timer_resolution_refcount = 0
_interpreter_cadence_lock = threading.RLock()
_interpreter_cadence_baseline_s: float | None = None
_interpreter_cadence_requests: dict[object, float] = {}
# The fastest demo target is a 50 Hz ArduPilot loop at 10x SITL, leaving a
# 2 ms wall slot.  A 1 ms handoff is the live-proven balance on Windows: the
# prior ``baseline / speedup`` rule produced 0.5 ms at 10x and made the three
# companion processes lose MAVLink/vision source progress as soon as navigation
# began.  This remains a scheduler implementation detail, not a user config.
_ACCELERATED_INTERPRETER_SWITCH_S = 0.001


class SchedulerCadence:
    """Convert an ArduPilot scheduler period into a wall-clock sleep period.

    ``SIM_SPEEDUP`` describes how frequently NavPy must run to stay aligned
    with accelerated SITL. This service deliberately exposes no clock: source
    timestamps, observation ages, and control ``dt`` remain owned by their
    respective producers and are never scaled here.
    """

    SPEED_POLL_WALL_S = 1.0
    _SYNC_EPS = 1e-9

    def __init__(
        self,
        speedup_reader: Callable[[], Optional[float]],
        *,
        wall_clock: Callable[[], float] = time.time,
        speed_poll_wall_s: float = SPEED_POLL_WALL_S,
        high_resolution_timer: bool = True,
    ) -> None:
        self._speedup_reader = speedup_reader
        self._wall_clock = wall_clock
        self._speed_poll_wall_s = (
            _positive_float(speed_poll_wall_s) or self.SPEED_POLL_WALL_S
        )
        self._speedup = 1.0
        self._next_speed_sync_wall_s = float(wall_clock())
        self._lock = threading.RLock()
        self._high_resolution_timer_enabled = bool(high_resolution_timer)
        self._high_resolution_timer_active = False
        self._high_resolution_timer_finalizer: Optional[weakref.finalize] = None
        self._interpreter_cadence_token = object()
        self._command_cadence_active = False
        self._interpreter_cadence_finalizer = weakref.finalize(
            self,
            _release_interpreter_cadence,
            self._interpreter_cadence_token,
        )
        self.sync_speed()

    @property
    def current_speedup(self) -> float:
        with self._lock:
            self._sync_speed_if_due_locked()
            return self._speedup

    def wall_period_for_scheduler_period(
        self,
        scheduler_period_s: float,
    ) -> float:
        """Return the wall sleep needed for one scheduler period."""
        period = _positive_float(scheduler_period_s)
        if period is None:
            return 0.0
        with self._lock:
            self._sync_speed_if_due_locked()
            self._sync_nonblocking_speed_locked()
            return period / self._speedup

    def sync_speed_if_due(self) -> None:
        with self._lock:
            self._sync_speed_if_due_locked()

    def sync_speed(self) -> float:
        """Refresh the scheduler multiplier without changing any clock."""
        with self._lock:
            value = self._read_speedup()
            if value is not None:
                self._speedup = value
            self._sync_process_cadence_locked()
            self._next_speed_sync_wall_s = (
                float(self._wall_clock()) + self._speed_poll_wall_s
            )
            return self._speedup

    def set_command_cadence_active(self, active: bool) -> None:
        """Tighten the process thread quantum only while commands are issued.

        Accelerated detector and MAVLink ingestion run before final approach.
        Applying the command-loop quantum to the whole process during that
        phase can overload those threads and bury confirmation responses behind
        telemetry.  The frequency adapter remains active at all times; only the
        process-wide interpreter hint is scoped to actual command work.
        """
        with self._lock:
            requested = bool(active)
            if requested == self._command_cadence_active:
                return
            self._command_cadence_active = requested
            self._sync_interpreter_cadence_locked()

    def close(self) -> None:
        """Release process timer-resolution state held by this adapter."""
        with self._lock:
            self._set_high_resolution_timer_locked(False)
            self._command_cadence_active = False
            finalizer = self._interpreter_cadence_finalizer
            if finalizer.alive:
                finalizer()

    def _sync_speed_if_due_locked(self) -> None:
        if (
            float(self._wall_clock()) + self._SYNC_EPS
            >= self._next_speed_sync_wall_s
        ):
            self.sync_speed()

    def _sync_nonblocking_speed_locked(self) -> None:
        value = self._read_speedup()
        if value is None or abs(value - self._speedup) <= self._SYNC_EPS:
            return
        self._speedup = value
        self._sync_process_cadence_locked()

    def _read_speedup(self) -> Optional[float]:
        try:
            return _positive_float(self._speedup_reader())
        except Exception:
            return None

    def _sync_process_cadence_locked(self) -> None:
        self._set_high_resolution_timer_locked(self._speedup > 1.01)
        self._sync_interpreter_cadence_locked()

    def _sync_interpreter_cadence_locked(self) -> None:
        if self._command_cadence_active:
            _set_interpreter_cadence(
                self._interpreter_cadence_token,
                self._speedup,
            )
        else:
            _release_interpreter_cadence(self._interpreter_cadence_token)

    def _set_high_resolution_timer_locked(self, enabled: bool) -> None:
        if not self._high_resolution_timer_enabled:
            enabled = False
        if enabled and not self._high_resolution_timer_active:
            if _request_high_resolution_timer():
                self._high_resolution_timer_active = True
                self._high_resolution_timer_finalizer = weakref.finalize(
                    self,
                    _release_high_resolution_timer,
                )
            return
        if not enabled and self._high_resolution_timer_active:
            finalizer = self._high_resolution_timer_finalizer
            self._high_resolution_timer_finalizer = None
            self._high_resolution_timer_active = False
            if finalizer is not None and finalizer.alive:
                finalizer()
            else:
                _release_high_resolution_timer()


def _positive_float(value: object) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        return None
    return number


def _request_high_resolution_timer() -> bool:
    """Request 1 ms Windows sleeps while accelerated SITL is active."""
    if os.name != "nt":
        return False
    global _timer_resolution_refcount
    with _timer_resolution_lock:
        if _timer_resolution_refcount == 0:
            try:
                result = ctypes.windll.winmm.timeBeginPeriod(  # type: ignore[name-defined, attr-defined]
                    _TIMER_PERIOD_MS
                )
            except Exception:
                return False
            if result != 0:
                return False
        _timer_resolution_refcount += 1
        return True


def _release_high_resolution_timer() -> None:
    if os.name != "nt":
        return
    global _timer_resolution_refcount
    with _timer_resolution_lock:
        if _timer_resolution_refcount <= 0:
            return
        _timer_resolution_refcount -= 1
        if _timer_resolution_refcount == 0:
            try:
                ctypes.windll.winmm.timeEndPeriod(  # type: ignore[name-defined, attr-defined]
                    _TIMER_PERIOD_MS
                )
            except Exception:
                pass


def _set_interpreter_cadence(token: object, speedup: float) -> None:
    """Use the live-proven accelerated thread handoff during command work."""
    global _interpreter_cadence_baseline_s
    with _interpreter_cadence_lock:
        if speedup <= 1.01:
            _interpreter_cadence_requests.pop(token, None)
        else:
            if _interpreter_cadence_baseline_s is None:
                _interpreter_cadence_baseline_s = sys.getswitchinterval()
            _interpreter_cadence_requests[token] = min(
                _interpreter_cadence_baseline_s,
                _ACCELERATED_INTERPRETER_SWITCH_S,
            )
        _apply_interpreter_cadence_locked()


def _release_interpreter_cadence(token: object) -> None:
    with _interpreter_cadence_lock:
        _interpreter_cadence_requests.pop(token, None)
        _apply_interpreter_cadence_locked()


def _apply_interpreter_cadence_locked() -> None:
    global _interpreter_cadence_baseline_s
    baseline = _interpreter_cadence_baseline_s
    if baseline is None:
        return
    target = (
        min(_interpreter_cadence_requests.values())
        if _interpreter_cadence_requests
        else baseline
    )
    if not math.isclose(sys.getswitchinterval(), target, abs_tol=1e-12):
        sys.setswitchinterval(target)
    if not _interpreter_cadence_requests:
        _interpreter_cadence_baseline_s = None


__all__ = ["SchedulerCadence"]
