"""Thread-safe navigation inbox for detector snapshots and publications."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

from navpy.modules.nav.terminal_source_contracts import DetectionInboxLease
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication


@dataclass(frozen=True)
class DetectionSelection:
    """One generation-consistent target selection input."""

    targets: tuple[DetectedObject, ...]
    primary_target: Optional[DetectedObject]


@dataclass
class DetectionSnapshot:
    """Latest detector response and source publications, guarded together."""

    _detected_targets: list[DetectedObject] = field(default_factory=list)
    _primary_target: Optional[DetectedObject] = None
    _lock: threading.RLock = field(default_factory=threading.RLock)
    _pending_events: list[DetectionPublication] = field(default_factory=list)
    _event_generation: int = 0

    def replace(
        self,
        detected_targets: list[DetectedObject],
        primary_target: Optional[DetectedObject],
        pending_events: list[DetectionPublication],
        *,
        append_events: bool,
    ) -> None:
        with self._lock:
            self._detected_targets = list(detected_targets)
            self._primary_target = primary_target
            if append_events:
                self._pending_events.extend(pending_events)
            else:
                self._pending_events = list(pending_events)
                self._event_generation += 1

    def clear(self) -> None:
        with self._lock:
            self._detected_targets = []
            self._primary_target = None
            self._pending_events = []
            self._event_generation += 1

    def replace_selection(
        self,
        detected_targets: list[DetectedObject],
        primary_target: Optional[DetectedObject] = None,
    ) -> None:
        """Replace only selection state while retaining queued publications."""
        with self._lock:
            self._detected_targets = list(detected_targets)
            self._primary_target = primary_target

    def discard_events(self) -> None:
        with self._lock:
            self._pending_events = []
            self._event_generation += 1

    def targets(self) -> list[DetectedObject]:
        return list(self.selection().targets)

    def selection(self) -> DetectionSelection:
        with self._lock:
            return DetectionSelection(
                tuple(self._detected_targets),
                self._primary_target,
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
