"""Atomic simulator detection snapshots and source-event publications."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Sequence

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_event_lease import DetectionEventLease
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.sim.detection_event_stream import DetectionEventStream
from navpy.modules.vision.sim.detection_snapshot_state import DetectionSnapshot
from navpy.modules.vision.poi_priority import prioritize_pois


@dataclass(eq=False)
class PublicationSlot:
    """Identity reservation; validity is owned by its publication store."""


class DetectionPublicationStore:
    """Commit detection snapshots and source events in one transaction."""

    def __init__(self, *, source_driven: bool, capacity: int) -> None:
        self._source_driven = bool(source_driven)
        self._capacity = max(1, int(capacity))
        self._condition = threading.Condition()
        self._reservations: set[PublicationSlot] = set()
        self._snapshot = DetectionSnapshot((), (), None)
        self._events = DetectionEventStream(self._condition)

    @property
    def source_driven(self) -> bool:
        return self._source_driven

    def reserve(
        self,
        invalidated: threading.Event,
        stopped: threading.Event,
    ) -> Optional[PublicationSlot]:
        while not (invalidated.is_set() or stopped.is_set()):
            with self._condition:
                if invalidated.is_set() or stopped.is_set():
                    return None
                if self._events.stopped_locked:
                    return None
                if self._events.clearing_locked:
                    self._condition.wait()
                    continue
                occupied = self._events.pending_count_locked() + len(
                    self._reservations
                )
                if not self._source_driven or occupied < self._capacity:
                    slot = PublicationSlot()
                    self._reservations.add(slot)
                    return slot
                self._condition.wait()
        return None

    def publish(
        self,
        slot: PublicationSlot,
        pois: Sequence[DetectedObject],
        *,
        primary_poi: Optional[DetectedObject],
        source_timestamp_s: float,
        source_receipt_timestamp_s: Optional[float],
        source_name: Optional[str],
        source_discontinuity: Optional[bool],
    ) -> bool:
        poi_tuple = tuple(pois)
        with self._condition:
            if (
                self._events.stopped_locked
                or self._events.clearing_locked
                or slot not in self._reservations
            ):
                return False
            self._reservations.remove(slot)
            self._snapshot = DetectionSnapshot(
                poi_tuple,
                poi_tuple,
                primary_poi,
            )
            if self._source_driven:
                publication = DetectionPublication(
                    tuple(prioritize_pois(poi_tuple, primary_poi)),
                    source_timestamp_s,
                    source_receipt_timestamp_s,
                    source_name,
                    bool(source_discontinuity),
                )
                self._events.publish_locked(publication)
            self._condition.notify_all()
            return True

    def abandon(self, slot: PublicationSlot) -> None:
        with self._condition:
            self._reservations.discard(slot)
            self._condition.notify_all()

    def drain(self) -> list[DetectionPublication]:
        if not self._source_driven:
            return []
        with self._condition:
            return self._events.drain_locked()

    def open_event_lease(
        self,
        reset_handler: Callable[[], None] | None = None,
    ) -> DetectionEventLease:
        if not self._source_driven:
            raise RuntimeError("polling publication stores cannot be leased")
        return self._events.open_lease(reset_handler)

    def snapshot(self) -> DetectionSnapshot:
        with self._condition:
            return self._snapshot

    def clear(
        self,
        *,
        mark_discontinuity: bool,
    ) -> list[DetectionPublication]:
        with self._condition:
            reset_handler, dropped = self._events.begin_clear_locked(
                mark_discontinuity=mark_discontinuity,
            )
            self._clear_store_state_locked()
        try:
            if reset_handler is not None:
                reset_handler()
        finally:
            with self._condition:
                self._events.finish_reset_locked()
        return dropped

    def stop(self) -> list[DetectionPublication]:
        """Permanently close publication admission and wake lease owners."""
        with self._condition:
            reset_handler, dropped = self._events.begin_stop_locked()
            self._clear_store_state_locked()
        try:
            if reset_handler is not None:
                reset_handler()
        finally:
            with self._condition:
                self._events.finish_reset_locked()
        return dropped

    def _clear_store_state_locked(self) -> None:
        self._reservations.clear()
        self._snapshot = DetectionSnapshot((), (), None)

    def wake(self) -> None:
        with self._condition:
            self._condition.notify_all()


__all__ = [
    "DetectionPublicationStore",
    "DetectionSnapshot",
    "PublicationSlot",
]
