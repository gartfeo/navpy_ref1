"""A loop's progress, as the in-process watchdog sees it (docs/design/navpy_recovery.md, step 6).

The loop marks each pass busy while it works (``with beacon.busy():``) and idle while it waits for its next slot.
A watchdog thread reads ``reading()``: a pass that stays busy with the same number is stuck. Waiting for the next
slot is never a hang.

A pass may also make a wait that is bounded by design, such as a parameter exchange with the autopilot (its reply
timeout times its attempts). Such a wait declares its bound (``with bounded_wait(bound_s):``) on the thread that
makes it; the pass running on that thread gets that much more time before it counts as stuck. Code not running in a
watched pass declares into nothing.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt


@dataclass(frozen=True)
class BeaconReading:
    passes: int          # passes entered so far
    busy: bool           # inside a pass now
    allowance_s: float   # bounded waits this pass declared so far


class ProgressBeacon:
    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._passes = 0
        self._busy = False
        self._allowance_s = 0.0

    @contextmanager
    def busy(self) -> Iterator[None]:
        thread = rt.get_ident()
        with self._lock:
            self._passes += 1
            self._busy = True
            self._allowance_s = 0.0
        with _REGISTRY_LOCK:
            previous = _PASS_OF_THREAD.get(thread)
            _PASS_OF_THREAD[thread] = self
        try:
            yield
        finally:
            with _REGISTRY_LOCK:
                if previous is None:
                    _PASS_OF_THREAD.pop(thread, None)
                else:
                    _PASS_OF_THREAD[thread] = previous
            with self._lock:
                self._busy = False

    def reading(self) -> BeaconReading:
        with self._lock:
            return BeaconReading(self._passes, self._busy, self._allowance_s)

    def _allow(self, bound_s: float) -> None:
        with self._lock:
            if self._busy:
                self._allowance_s += bound_s


_REGISTRY_LOCK = rt.Lock()
_PASS_OF_THREAD: dict[int, ProgressBeacon] = {}   # thread ident -> the beacon of the pass it runs


@contextmanager
def bounded_wait(bound_s: float) -> Iterator[None]:
    """A wait of at most ``bound_s`` (wall seconds) on this thread, by design: not a hang of its pass."""
    with _REGISTRY_LOCK:
        beacon = _PASS_OF_THREAD.get(rt.get_ident())
    if beacon is not None and bound_s > 0:
        beacon._allow(float(bound_s))
    yield


__all__ = ["BeaconReading", "ProgressBeacon", "bounded_wait"]
