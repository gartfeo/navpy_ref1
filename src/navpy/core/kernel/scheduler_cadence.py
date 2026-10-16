"""Adapt NavPy scheduler frequency to accelerated ArduPilot SITL."""

from __future__ import annotations

import math
import sys
import weakref
from numbers import Real
from typing import Callable, Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.reader_budget_feed import ReaderBudgetFeed
from navpy.core.kernel.windows_timer_resolution import (
    release_high_resolution_timer as _release_high_resolution_timer,
    request_high_resolution_timer as _request_high_resolution_timer,
)

_interpreter_cadence_lock = rt.RLock()
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
        wall_clock: Callable[[], float] = rt.time,
        speed_poll_wall_s: float = SPEED_POLL_WALL_S,
        high_resolution_timer: bool = True,
        reader_budget: Optional[ReaderBudgetFeed] = None,
    ) -> None:
        self._speedup_reader = speedup_reader
        self._reader_budget = reader_budget
        self._wall_clock = wall_clock
        self._speed_poll_wall_s = (
            _positive_float(speed_poll_wall_s) or self.SPEED_POLL_WALL_S
        )
        self._speedup = 1.0
        self._next_speed_sync_wall_s = float(wall_clock())
        self._lock = rt.RLock()
        self._high_resolution_timer_enabled = bool(high_resolution_timer)
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
            feed, self._reader_budget = self._reader_budget, None
            if feed is not None:
                feed.sink.release()
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
        self._sync_reader_budget_locked()

    def _sync_reader_budget_locked(self) -> None:
        feed = self._reader_budget
        if feed is not None:
            feed.sink.set(feed.scheduler_period_s / self._speedup)

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
        # The finalizer is the held request: one exists while it is active.
        finalizer = self._high_resolution_timer_finalizer
        if enabled and finalizer is None:
            if _request_high_resolution_timer():
                self._high_resolution_timer_finalizer = weakref.finalize(
                    self,
                    _release_high_resolution_timer,
                )
            return
        if not enabled and finalizer is not None:
            self._high_resolution_timer_finalizer = None
            if finalizer.alive:
                finalizer()
            else:
                _release_high_resolution_timer()


class IdentitySchedulerCadence(SchedulerCadence):
    """Lockstep's cadence: sim time is the scheduler's time, so the speedup is 1.

    It never reads ``SIM_SPEEDUP`` (nothing under lockstep writes it), and
    with a speedup of 1 it requests no timer resolution and no interpreter
    switch interval: every wall period equals its scheduler period.
    """

    def __init__(self) -> None:
        super().__init__(lambda: 1.0, high_resolution_timer=False)


def _positive_float(value: object) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        return None
    return number


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


__all__ = ["IdentitySchedulerCadence", "SchedulerCadence"]
