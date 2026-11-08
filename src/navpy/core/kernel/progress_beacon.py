"""A loop's progress, as the in-process watchdog sees it (docs/design/navpy_recovery.md, step 6).

The loop marks each pass busy while it works (``with beacon.busy():``) and idle while it waits for its next slot.
A watchdog thread reads ``reading()``: a pass that stays busy with the same number is stuck. Waiting is never a
hang, so a loop that sleeps between passes, or has nothing to do, is not reported.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt


@dataclass(frozen=True)
class BeaconReading:
    passes: int     # passes entered so far
    busy: bool      # inside a pass now


class ProgressBeacon:
    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._passes = 0
        self._busy = False

    @contextmanager
    def busy(self) -> Iterator[None]:
        with self._lock:
            self._passes += 1
            self._busy = True
        try:
            yield
        finally:
            with self._lock:
                self._busy = False

    def reading(self) -> BeaconReading:
        with self._lock:
            return BeaconReading(self._passes, self._busy)


__all__ = ["BeaconReading", "ProgressBeacon"]
