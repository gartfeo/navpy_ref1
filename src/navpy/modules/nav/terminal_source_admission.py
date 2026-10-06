"""Source-event admission for terminal NAV."""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterable
from typing import Callable, Optional, Protocol

from navpy.modules.nav.terminal_source_contracts import (
    ActiveSourceEvents,
    DetectionInboxLease,
    NavSourceBatch,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.target_identity import get_target_task_id
from navpy.modules.vision.target_priority import find_target_by_task_id


class DetectionEventInbox(Protocol):
    def events(self) -> list[DetectionPublication]: ...

    def lease_events(self) -> DetectionInboxLease: ...

    def ack_events(self, lease: DetectionInboxLease) -> bool: ...

    def discard_events(self) -> None: ...


class TerminalSourceSessionPort(Protocol):
    def active_events(
        self,
        active_target: Optional[DetectedObject],
        pending_events: Iterable[DetectionPublication],
    ) -> ActiveSourceEvents: ...

    def target_uses_source_events(
        self,
        *targets: Optional[DetectedObject],
    ) -> bool: ...

    def detection_receipt_age_s(
        self,
        target: DetectedObject,
    ) -> Optional[float]: ...


class TerminalBatchAdmission(Protocol):
    def admit_batch(
        self,
        publication: Optional[DetectionPublication],
        active_target: DetectedObject,
        *,
        discontinuity_source_names: tuple[str, ...],
        has_unnamed_discontinuity: bool,
        commit: Callable[[], bool],
    ) -> bool: ...


@dataclass(frozen=True)
class TerminalSourcePorts:
    source_session: TerminalSourceSessionPort
    active_target: Callable[[], Optional[DetectedObject]]
    event_inbox: DetectionEventInbox
    detections: Callable[[], list[DetectedObject]]


class TerminalSourceAdmission:
    """Own source-event admission and active-target matching."""

    def __init__(
        self,
        ports: TerminalSourcePorts,
        publication_admission: TerminalBatchAdmission,
    ) -> None:
        self._ports = ports
        self._publication_admission = publication_admission

    def collect(
        self,
        fresh_target: Optional[DetectedObject],
        active_target: DetectedObject,
    ) -> NavSourceBatch:
        source_driven = self.target_uses_source_driven_events(
            fresh_target,
            active_target,
        )
        event_lease: DetectionInboxLease | None = None
        if source_driven:
            event_lease = self._ports.event_inbox.lease_events()
            active_events = self._ports.source_session.active_events(
                self._ports.active_target(),
                event_lease.events,
            )
        else:
            active_events = ActiveSourceEvents(())
            self.clear_pending_source_events()
        return NavSourceBatch(
            source_driven=source_driven,
            latest_event=active_events.latest,
            discontinuity_source_names=(
                active_events.discontinuity_source_names
            ),
            has_unnamed_discontinuity=(
                active_events.has_unnamed_discontinuity
            ),
            event_lease=event_lease,
        )

    def frame_ready(
        self,
        batch: NavSourceBatch,
        fresh_target: Optional[DetectedObject],
        active_target: DetectedObject,
    ) -> bool:
        if batch.source_driven and batch.latest_event is None:
            if batch.event_lease is not None:
                self._ports.event_inbox.ack_events(batch.event_lease)
            return False
        if fresh_target is None and not batch.source_driven:
            return False
        return True

    def consume(
        self,
        batch: NavSourceBatch,
        active_target: DetectedObject,
    ) -> bool:
        return self._publication_admission.admit_batch(
            batch.latest_event,
            active_target,
            discontinuity_source_names=batch.discontinuity_source_names,
            has_unnamed_discontinuity=batch.has_unnamed_discontinuity,
            commit=lambda: (
                batch.event_lease is not None
                and self._ports.event_inbox.ack_events(batch.event_lease)
            ),
        )

    @staticmethod
    def source_event_target(
        event: Optional[DetectionPublication],
        active_target: DetectedObject,
    ) -> Optional[DetectedObject]:
        if event is None:
            return None
        return find_target_by_task_id(
            event.detected_targets,
            get_target_task_id(active_target),
        )

    def clear_pending_source_events(self) -> None:
        self._ports.event_inbox.discard_events()

    def pending_active_target_events(self) -> list[DetectionPublication]:
        return list(self._ports.source_session.active_events(
            self._ports.active_target(),
            self._ports.event_inbox.events(),
        ).events)

    def target_uses_source_driven_events(
        self,
        *targets: Optional[DetectedObject],
    ) -> bool:
        return self._ports.source_session.target_uses_source_events(*targets)

    def source_receipt_age_s(self, target: DetectedObject) -> Optional[float]:
        return self._ports.source_session.detection_receipt_age_s(target)

    def find_active_target_detection(self) -> Optional[DetectedObject]:
        active = self._ports.active_target()
        if active is None:
            return None
        return find_target_by_task_id(
            self._ports.detections(),
            get_target_task_id(active),
        )


__all__ = [
    "DetectionEventInbox",
    "TerminalBatchAdmission",
    "TerminalSourceAdmission",
    "TerminalSourcePorts",
    "TerminalSourceSessionPort",
]
