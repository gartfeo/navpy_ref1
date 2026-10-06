"""Fresh-source release gate for approved final approaches."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.nav.confirmation_reporting import ConfirmDebugReporter
from navpy.modules.nav.detection_freshness import DetectionFreshnessPolicy
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.poi_identity import get_poi_task_id
from navpy.modules.vision.poi_priority import find_poi_by_task_id


class FinalApproachReleaseSourceQuery(Protocol):
    """Source queries required before releasing an approved navigation_task."""

    def find_active_poi_detection(self) -> DetectedObject | None: ...

    def poi_uses_source_driven_events(
        self,
        *pois: DetectedObject | None,
    ) -> bool: ...

    def pending_active_poi_events(self) -> list[DetectionPublication]: ...


class FinalApproachReleaseGate:
    """Require a fresh same-POI frame before operator approval releases."""

    def __init__(
        self,
        source: FinalApproachReleaseSourceQuery,
        freshness: DetectionFreshnessPolicy,
        debug: ConfirmDebugReporter,
    ) -> None:
        self._source = source
        self._freshness = freshness
        self._debug = debug

    def is_ready(self, active_poi: DetectedObject) -> bool:
        fresh_poi = self._source.find_active_poi_detection()
        if fresh_poi is None:
            self._debug.log(
                "operator_approved_waiting_detection "
                f"obj={active_poi.identity.obj_id}"
            )
            return False
        source_driven = self._source.poi_uses_source_driven_events(
            fresh_poi,
            active_poi,
        )
        task_id = get_poi_task_id(active_poi)
        if not source_driven:
            return self._freshness.is_poi_fresh_for_confirm(task_id)
        events = self._source.pending_active_poi_events()
        latest_event = events[-1] if events else None
        event_poi = (
            find_poi_by_task_id(latest_event.detected_pois, task_id)
            if latest_event is not None
            else None
        )
        if (
            event_poi is not None
            and not self._freshness.detection_is_fresh_for_confirm(
                event_poi,
                require_source_receipt=True,
            )
        ):
            event_poi = None
        if event_poi is None:
            self._debug.log(
                "operator_approved_waiting_source_event "
                f"obj={active_poi.identity.obj_id}"
            )
        return event_poi is not None


__all__ = ["FinalApproachReleaseGate", "FinalApproachReleaseSourceQuery"]
