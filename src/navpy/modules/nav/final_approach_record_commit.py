"""One-time final-approach confirmation record commit."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.modules.nav.final_approach_source_contracts import NavSourceBatch
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication


class FinalApproachRecordDeferral(Protocol):
    def defer_or_fail(self, poi: DetectedObject) -> None: ...


class FinalApproachRecordPoiResolver(Protocol):
    def source_event_poi(
        self,
        event: Optional[DetectionPublication],
        active_poi: DetectedObject,
    ) -> Optional[DetectedObject]: ...


@dataclass(frozen=True)
class FinalApproachRecordPorts:
    uses_vision_nav: Callable[[], bool]
    final_approach_recorded: Callable[[], bool]
    mark_final_approach_recorded: Callable[[], None]
    record_detection: Callable[[DetectedObject], bool]
    debug: Callable[[str], None]
    mark_no_detection: Callable[[DetectedObject], None]


class FinalApproachRecordCommit:
    """Commit the first accepted visual record before command dispatch."""

    def __init__(
        self,
        ports: FinalApproachRecordPorts,
        source: FinalApproachRecordPoiResolver,
        deadline: FinalApproachRecordDeferral,
    ) -> None:
        self._ports = ports
        self._source = source
        self._deadline = deadline

    def commit(
        self,
        batch: NavSourceBatch,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> bool:
        if (
            not self._ports.uses_vision_nav()
            or self._ports.final_approach_recorded()
        ):
            return True
        record_poi = fresh_poi
        if batch.source_driven:
            record_poi = self._source.source_event_poi(
                batch.latest_event,
                active_poi,
            )
            if record_poi is None:
                self._ports.mark_no_detection(active_poi)
                return False
        if record_poi is None:
            return False
        if not self._ports.record_detection(record_poi):
            if batch.source_driven:
                self._ports.debug(
                    "final_approach_record_rejected_source_event "
                    f"obj={record_poi.identity.obj_id}"
                )
            self._deadline.defer_or_fail(record_poi)
            return False
        self._ports.mark_final_approach_recorded()
        return True


__all__ = [
    "FinalApproachRecordCommit",
    "FinalApproachRecordDeferral",
    "FinalApproachRecordPorts",
    "FinalApproachRecordPoiResolver",
]
