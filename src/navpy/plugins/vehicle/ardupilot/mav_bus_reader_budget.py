"""Wall-time budget for one MAVLink reader batch on a shared bus.

The reader drains every packet that is already decodable before it yields,
but a batch ends once it has run for one scheduler wall slot, so continuous
ingress cannot starve the detector or the command worker
(docs/design/reader_batch.md). Each scheduler that shares the bus requests a
budget; the reader uses the smallest live request, so the fastest vehicle on
a shared bus wins.
"""
from __future__ import annotations

import math

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.pose_stream_rate import SCHED_LOOP_RATE_FALLBACK_HZ

#: With no live request (the GCS backend has no scheduler cadence, and NavPy
#: before its cadence exists) a batch may run for one 1x slot of a 50 Hz
#: ArduPilot loop: 20 ms.
DEFAULT_READER_BATCH_BUDGET_S = 1.0 / SCHED_LOOP_RATE_FALLBACK_HZ


class ReaderBudget:
    """One owner's request; ``set`` replaces it, ``release`` withdraws it."""

    def __init__(self, registry: "ReaderBudgetRegistry", owner: str) -> None:
        self._registry = registry
        self.owner = owner

    def set(self, wall_slot_s: float) -> None:
        self._registry._set(self, wall_slot_s)

    def release(self) -> None:
        self._registry._release(self)


class ReaderBudgetRegistry:
    """Locked set of live budget requests; the minimum is the batch budget."""

    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._requests: dict[ReaderBudget, float | None] = {}

    def request(self, owner: str) -> ReaderBudget:
        handle = ReaderBudget(self, owner)
        with self._lock:
            self._requests[handle] = None
        return handle

    def current_s(self) -> float:
        with self._lock:
            slots = [slot for slot in self._requests.values() if slot is not None]
        return min(slots) if slots else DEFAULT_READER_BATCH_BUDGET_S

    def _set(self, handle: ReaderBudget, wall_slot_s: float) -> None:
        slot = float(wall_slot_s)
        if not math.isfinite(slot) or slot <= 0.0:
            raise ValueError(f"reader budget must be positive: {wall_slot_s!r}")
        with self._lock:
            if handle not in self._requests:
                raise RuntimeError(
                    f"reader budget for {handle.owner} set after release"
                )
            self._requests[handle] = slot

    def _release(self, handle: ReaderBudget) -> None:
        with self._lock:
            self._requests.pop(handle, None)


__all__ = [
    "DEFAULT_READER_BATCH_BUDGET_S",
    "ReaderBudget",
    "ReaderBudgetRegistry",
]
