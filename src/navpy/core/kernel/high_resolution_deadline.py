"""Interruptible high-resolution wall-deadline waiting."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol


HIGH_RESOLUTION_TAIL_S = 0.05


class DeadlineStopEvent(Protocol):
    def is_set(self) -> bool: ...

    def wait(self, timeout: float) -> bool: ...


def wait_until_deadline(
    deadline_s: float,
    *,
    stop_event: DeadlineStopEvent,
    monotonic_s: Callable[[], float],
    sleep_s: Callable[[float], None],
) -> bool:
    """Return at the deadline, or ``False`` when stop interrupts the wait."""
    while not stop_event.is_set():
        remaining_s = deadline_s - monotonic_s()
        if remaining_s <= 0.0:
            return True
        if remaining_s > HIGH_RESOLUTION_TAIL_S:
            stop_event.wait(remaining_s - HIGH_RESOLUTION_TAIL_S)
        else:
            # Windows Event timeouts quantize short waits; time.sleep uses the
            # high-resolution waitable timer on the supported runtime.
            sleep_s(remaining_s)
    return False


__all__ = [
    "DeadlineStopEvent",
    "HIGH_RESOLUTION_TAIL_S",
    "wait_until_deadline",
]
