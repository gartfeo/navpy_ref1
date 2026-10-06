"""Source-event admission for final-approach NAV."""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterable
from typing import Callable, Optional, Protocol

from navpy.modules.nav.final_approach_source_contracts import (
    ActiveSourceEvents,
    DetectionInboxLease,
    NavSourceBatch,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.poi_identity import get_poi_task_id
from navpy.modules.vision.poi_priority import find_poi_by_task_id


class DetectionEventInbox(Protocol):
    def events(self) -> list[DetectionPublication]: ...

    def lease_events(self) -> DetectionInboxLease: ...

    def ack_events(self, lease: DetectionInboxLease) -> bool: ...

    def discard_events(self) -> None: ...


class FinalApproachSourceSessionPort(Protocol):
    def active_events(
        self,
        active_poi: Optional[DetectedObject],
        pending_events: Iterable[DetectionPublication],
    ) -> ActiveSourceEvents: ...

    def poi_uses_source_events(
        self,
        *pois: Optional[DetectedObject],
    ) -> bool: ...

    def detection_receipt_age_s(
        self,
        poi: DetectedObject,
    ) -> Optional[float]: ...


class FinalApproachBatchAdmission(Protocol):
    def admit_batch(
        self,
        publication: Optional[DetectionPublication],
        active_poi: DetectedObject,
        *,
        discontinuity_source_names: tuple[str, ...],
        has_unnamed_discontinuity: bool,
        commit: Callable[[], bool],
    ) -> bool: ...


@dataclass(frozen=True)
class FinalApproachSourcePorts:
    source_session: FinalApproachSourceSessionPort
    active_poi: Callable[[], Optional[DetectedObject]]
    event_inbox: DetectionEventInbox
    detections: Callable[[], list[DetectedObject]]


class FinalApproachSourceAdmission:
    """Own source-event admission and active-POI matching."""

    def __init__(
        self,
        ports: FinalApproachSourcePorts,
        publication_admission: FinalApproachBatchAdmission,
    ) -> None:
        self._ports = ports
        self._publication_admission = publication_admission

    def collect(
        self,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> NavSourceBatch:
        source_driven = self.poi_uses_source_driven_events(
            fresh_poi,
            active_poi,
        )
        event_lease: DetectionInboxLease | None = None
        if source_driven:
            event_lease = self._ports.event_inbox.lease_events()
            active_events = self._ports.source_session.active_events(
                self._ports.active_poi(),
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
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> bool:
        if batch.source_driven and batch.latest_event is None:
            if batch.event_lease is not None:
                self._ports.event_inbox.ack_events(batch.event_lease)
            return False
        if fresh_poi is None and not batch.source_driven:
            return False
        return True

    def consume(
        self,
        batch: NavSourceBatch,
        active_poi: DetectedObject,
    ) -> bool:
        return self._publication_admission.admit_batch(
            batch.latest_event,
            active_poi,
            discontinuity_source_names=batch.discontinuity_source_names,
            has_unnamed_discontinuity=batch.has_unnamed_discontinuity,
            commit=lambda: (
                batch.event_lease is not None
                and self._ports.event_inbox.ack_events(batch.event_lease)
            ),
        )

    @staticmethod
    def source_event_poi(
        event: Optional[DetectionPublication],
        active_poi: DetectedObject,
    ) -> Optional[DetectedObject]:
        if event is None:
            return None
        return find_poi_by_task_id(
            event.detected_pois,
            get_poi_task_id(active_poi),
        )

    def clear_pending_source_events(self) -> None:
        self._ports.event_inbox.discard_events()

    def pending_active_poi_events(self) -> list[DetectionPublication]:
        return list(self._ports.source_session.active_events(
            self._ports.active_poi(),
            self._ports.event_inbox.events(),
        ).events)

    def poi_uses_source_driven_events(
        self,
        *pois: Optional[DetectedObject],
    ) -> bool:
        return self._ports.source_session.poi_uses_source_events(*pois)

    def source_receipt_age_s(self, poi: DetectedObject) -> Optional[float]:
        return self._ports.source_session.detection_receipt_age_s(poi)

    def find_active_poi_detection(self) -> Optional[DetectedObject]:
        active = self._ports.active_poi()
        if active is None:
            return None
        return find_poi_by_task_id(
            self._ports.detections(),
            get_poi_task_id(active),
        )


__all__ = [
    "DetectionEventInbox",
    "FinalApproachBatchAdmission",
    "FinalApproachSourceAdmission",
    "FinalApproachSourcePorts",
    "FinalApproachSourceSessionPort",
]
