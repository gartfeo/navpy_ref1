"""Generation-fenced confirmation worker leases."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.review.confirmation_round_transaction import (
    ConfirmationWorkerLease,
)


class ConfirmationWorkerState:
    def __init__(self, lock: rt.RLock) -> None:
        self._lock = lock
        self._generation = 0
        # A dict keeps insertion order; a lease holds an Event, so a set of
        # leases would order by object identity.
        self._workers: dict[ConfirmationWorkerLease, None] = {}
        self._current_by_poi: dict[int, ConfirmationWorkerLease] = {}

    def start(
        self,
        poi_id: Optional[int],
        event_factory: Callable[[], rt.Event],
    ) -> ConfirmationWorkerLease:
        with self._lock:
            previous = (
                None if poi_id is None else self._current_by_poi.get(poi_id)
            )
            if previous is not None:
                previous.cancel_event.set()
                self._workers.pop(previous, None)
            lease = ConfirmationWorkerLease(
                poi_id,
                self._generation,
                event_factory(),
            )
            self._workers[lease] = None
            if poi_id is not None:
                self._current_by_poi[poi_id] = lease
            return lease

    def is_current(self, lease: ConfirmationWorkerLease) -> bool:
        with self._lock:
            return (
                lease.generation == self._generation
                and lease in self._workers
                and not lease.cancel_event.is_set()
                and (
                    lease.poi_id is None
                    or self._current_by_poi.get(lease.poi_id) is lease
                )
            )

    def finish(self, lease: ConfirmationWorkerLease) -> None:
        with self._lock:
            self._workers.pop(lease, None)
            if (
                lease.poi_id is not None
                and self._current_by_poi.get(lease.poi_id) is lease
            ):
                self._current_by_poi.pop(lease.poi_id, None)

    def reset(self) -> None:
        with self._lock:
            self._generation += 1
            workers = tuple(self._workers)
            self._workers.clear()
            self._current_by_poi.clear()
            for worker in workers:
                worker.cancel_event.set()

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._workers)


__all__ = ["ConfirmationWorkerState"]
