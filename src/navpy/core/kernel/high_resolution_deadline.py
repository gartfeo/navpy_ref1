"""Interruptible high-resolution wall-deadline waiting."""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Protocol


HIGH_RESOLUTION_TAIL_S = 0.05

# The longest wall period a scheduler deadline loop may wait for one slot
# (todo 291). The cadence re-reads SIM_SPEEDUP every second
# (``SchedulerCadence.SPEED_POLL_WALL_S``); a longer slot could not follow a
# corrected speedup, and no scheduler rate NavPy starts with needs one at a
# speedup above 1/25 (SCHED_LOOP_RATE must be 50 to 2000 Hz or NavPy refuses
# to start, see pose_stream_rate; the pre-start NAV default is 25 Hz). A tiny
# speedup therefore makes the loop run sooner, never stall it for years.
MAX_SCHEDULER_WALL_PERIOD_S = 1.0

# The longest single ``Event.wait`` slice. ``threading.TIMEOUT_MAX`` is about
# 49.7 days on Windows and larger elsewhere; a wait above it raises
# OverflowError. Each slice re-reads the clock, so one second costs nothing.
MAX_WAIT_SLICE_S = 1.0


class DeadlineStopEvent(Protocol):
    def is_set(self) -> bool: ...

    def wait(self, timeout: float) -> bool: ...


def bounded_scheduler_wall_period_s(
    wall_period_s: object,
    *,
    fallback_s: float,
) -> float:
    """Return a wall period in ``[0, MAX_SCHEDULER_WALL_PERIOD_S]``.

    A NaN, negative (including -inf) or unreadable period is no period: use
    ``fallback_s`` (the unscaled scheduler period), itself bounded, with 0
    when it is no period either. Zero stays zero, as before.
    """
    value = _float_or_nan(wall_period_s)
    if not value >= 0.0:
        value = _float_or_nan(fallback_s)
        if not value >= 0.0:
            value = 0.0
    return min(value, MAX_SCHEDULER_WALL_PERIOD_S)


def wait_until_deadline(
    deadline_s: float,
    *,
    stop_event: DeadlineStopEvent,
    monotonic_s: Callable[[], float],
    sleep_s: Callable[[float], None],
    max_wait_s: float = math.inf,
) -> bool:
    """Return at the deadline, or ``False`` when stop interrupts the wait.

    A non-finite deadline or clock reading is not a deadline: wait out what
    is left of a finite ``max_wait_s`` (else nothing) and return, instead of
    raising or waiting forever. ``max_wait_s`` caps the total wait. It is
    spent by the clock's measured progress until the clock steps back or
    freezes across an Event wait, then by the time asked of each wait, so
    such a clock cannot stretch the wait, while an early sleep return still
    waits the rest. Scheduler loops use ``wait_for_slot``.
    """
    budget_s = max_wait_s
    previous_s: float | None = None
    wait_s = 0.0
    waited_on_event = False
    stalled = False
    clock_trusted = True
    while not stop_event.is_set():
        now_s = monotonic_s()
        if previous_s is not None:
            progress_s = now_s - previous_s
            # A clock that stepped back, or did not move across an Event
            # wait (which ends early only on stop) or across two waits in a
            # row, is not measuring this wait. One sleep may end early.
            frozen = progress_s == 0.0 and (waited_on_event or stalled)
            if not progress_s >= 0.0 or frozen:
                clock_trusted = False
            stalled = progress_s == 0.0
            budget_s -= progress_s if clock_trusted else wait_s
        previous_s = now_s
        remaining_s = min(deadline_s - now_s, budget_s)
        if not math.isfinite(remaining_s):
            # The clock or deadline stopped measuring: wait out a finite
            # budget (one slice at most) so the caller is still paced.
            if 0.0 < budget_s < math.inf:
                stop_event.wait(min(budget_s, MAX_WAIT_SLICE_S))
            return not stop_event.is_set()
        if remaining_s <= 0.0:
            return True
        if remaining_s > HIGH_RESOLUTION_TAIL_S:
            wait_s = min(remaining_s - HIGH_RESOLUTION_TAIL_S, MAX_WAIT_SLICE_S)
            waited_on_event = True
            stop_event.wait(wait_s)
        else:
            # Windows Event timeouts quantize short waits; time.sleep uses the
            # high-resolution waitable timer on the supported runtime.
            wait_s = remaining_s
            waited_on_event = False
            sleep_s(wait_s)
    return False


def wait_for_slot(
    deadline_s: float,
    period_s: float,
    *,
    stop_event: DeadlineStopEvent,
    monotonic_s: Callable[[], float],
    sleep_s: Callable[[float], None],
) -> bool:
    """Wait for a scheduler slot of ``period_s`` ending at ``deadline_s``.

    Return ``False`` when stop interrupts the wait. The wait never asks for
    more than one period in total, so a clock stepping back, before or
    during the wait, cannot strand the loop. When the clock cannot measure
    the slot (a non-finite reading or deadline, or a value so large that
    adding the period does not change it), pace by the period itself
    instead of spinning.
    """
    period_s = bounded_scheduler_wall_period_s(period_s, fallback_s=0.0)
    now_s = _float_or_nan(monotonic_s())
    if (
        not math.isfinite(now_s)
        or not math.isfinite(deadline_s)
        or (period_s > 0.0 and now_s + period_s <= now_s)
    ):
        if period_s > 0.0:
            stop_event.wait(period_s)
        return not stop_event.is_set()
    return wait_until_deadline(
        deadline_s,
        stop_event=stop_event,
        monotonic_s=monotonic_s,
        sleep_s=sleep_s,
        max_wait_s=period_s,
    )


def next_fixed_deadline(
    scheduled_s: float,
    completed_s: float,
    period_s: float,
) -> float:
    """Return the first unmissed deadline on the existing AP wall grid."""
    if period_s <= 0.0:
        return completed_s
    if completed_s <= math.nextafter(scheduled_s, math.inf):
        return scheduled_s
    late_periods = (completed_s - scheduled_s) / period_s
    if not math.isfinite(late_periods):
        # A non-finite or huge clock value has no grid to keep: re-anchor one
        # period after completion; the waiter paces a clock that cannot
        # measure it (todo 291).
        return completed_s + period_s
    slots_late = math.ceil(late_periods)
    candidate = scheduled_s + slots_late * period_s
    # Protect against a downward-rounded product. Never replay a missed slot.
    return candidate + period_s if candidate < completed_s else candidate



def _float_or_nan(value: object) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError):
        return math.nan


__all__ = [
    "DeadlineStopEvent",
    "HIGH_RESOLUTION_TAIL_S",
    "MAX_SCHEDULER_WALL_PERIOD_S",
    "MAX_WAIT_SLICE_S",
    "bounded_scheduler_wall_period_s",
    "next_fixed_deadline",
    "wait_for_slot",
    "wait_until_deadline",
]
