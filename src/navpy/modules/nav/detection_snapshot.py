"""Thread-safe navigation inbox for detector snapshots and publications."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

from navpy.modules.nav.final_approach_source_contracts import DetectionInboxLease
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication


@dataclass(frozen=True)
class DetectionSelection:
    """One generation-consistent POI selection input."""

    pois: tuple[DetectedObject, ...]
    primary_poi: Optional[DetectedObject]


@dataclass
class DetectionSnapshot:
    """Latest detector response and source publications, guarded together."""

    _detected_pois: list[DetectedObject] = field(default_factory=list)
    _primary_poi: Optional[DetectedObject] = None
    _lock: threading.RLock = field(default_factory=threading.RLock)
    _pending_events: list[DetectionPublication] = field(default_factory=list)
    _event_generation: int = 0

    def replace(
        self,
        detected_pois: list[DetectedObject],
        primary_poi: Optional[DetectedObject],
        pending_events: list[DetectionPublication],
        *,
        append_events: bool,
    ) -> None:
        with self._lock:
            self._detected_pois = list(detected_pois)
            self._primary_poi = primary_poi
            if append_events:
                self._pending_events.extend(pending_events)
            else:
                self._pending_events = list(pending_events)
                self._event_generation += 1

    def clear(self) -> None:
        with self._lock:
            self._detected_pois = []
            self._primary_poi = None
            self._pending_events = []
            self._event_generation += 1

    def replace_selection(
        self,
        detected_pois: list[DetectedObject],
        primary_poi: Optional[DetectedObject] = None,
    ) -> None:
        """Replace only selection state while retaining queued publications."""
        with self._lock:
            self._detected_pois = list(detected_pois)
            self._primary_poi = primary_poi

    def discard_events(self) -> None:
        with self._lock:
            self._pending_events = []
            self._event_generation += 1

    def pois(self) -> list[DetectedObject]:
        return list(self.selection().pois)

    def selection(self) -> DetectionSelection:
        with self._lock:
            return DetectionSelection(
                tuple(self._detected_pois),
                self._primary_poi,
            )

    def events(self) -> list[DetectionPublication]:
        with self._lock:
            return list(self._pending_events)

    def lease_events(self) -> DetectionInboxLease:
        with self._lock:
            return DetectionInboxLease(
                generation=self._event_generation,
                events=tuple(self._pending_events),
            )

    def ack_events(self, lease: DetectionInboxLease) -> bool:
        """Remove only the exact leased prefix, retaining later appends."""
        with self._lock:
            if lease.generation != self._event_generation:
                return False
            prefix_length = len(lease.events)
            if len(self._pending_events) < prefix_length:
                return False
            if any(
                current is not leased
                for current, leased in zip(
                    self._pending_events[:prefix_length],
                    lease.events,
                )
            ):
                return False
            del self._pending_events[:prefix_length]
            self._event_generation += 1
            return True


__all__ = ["DetectionSelection", "DetectionSnapshot"]
