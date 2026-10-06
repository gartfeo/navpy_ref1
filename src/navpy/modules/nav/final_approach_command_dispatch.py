"""Final-approach record commit and measured-frame command dispatch."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.final_approach_source_contracts import NavSourceBatch
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.poi_identity import get_poi_task_id


@dataclass(frozen=True)
class FinalApproachCommandPorts:
    nav: Callable[[Optional[DetectedObject]], bool]
    mark_failed: Callable[[], None]
    logger: ILogger


class FinalApproachCommandPoiResolver(Protocol):
    def source_event_poi(
        self,
        event: Optional[DetectionPublication],
        active_poi: DetectedObject,
    ) -> Optional[DetectedObject]: ...


class FinalApproachCommandDispatch:
    """Own measured-frame dispatch and final-approach failure reporting."""

    def __init__(
        self,
        ports: FinalApproachCommandPorts,
        source: FinalApproachCommandPoiResolver,
    ) -> None:
        self._ports = ports
        self._source = source

    def dispatch(
        self,
        batch: NavSourceBatch,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> bool:
        if not batch.source_driven:
            if not self._ports.nav(fresh_poi):
                self.mark_navigation_rejected(fresh_poi or active_poi)
                return False
            return True
        if batch.latest_event is None:
            self.mark_navigation_rejected(active_poi)
            return False
        return self.dispatch_source_publication(
            batch.latest_event,
            active_poi,
        )

    def dispatch_source_publication(
        self,
        publication: DetectionPublication,
        active_poi: DetectedObject,
    ) -> bool:
        nav_poi = self.source_publication_poi(
            publication,
            active_poi,
        )
        if nav_poi is None:
            self.mark_navigation_rejected(active_poi)
            return False
        return self.dispatch_poi(nav_poi)

    def source_publication_poi(
        self,
        publication: DetectionPublication,
        active_poi: DetectedObject,
    ) -> DetectedObject | None:
        return self._source.source_event_poi(publication, active_poi)

    def dispatch_poi(self, nav_poi: DetectedObject) -> bool:
        if not self._ports.nav(nav_poi):
            self.mark_navigation_rejected(nav_poi)
            return False
        return True

    def mark_no_detection(self, active_poi: DetectedObject) -> None:
        self._mark_failed_and_log(
            lambda: (
                f"NAV_FAIL: no_detection "
                f"task={get_poi_task_id(active_poi)} "
                f"obj={active_poi.identity.obj_id}"
            )
        )

    def mark_navigation_rejected(self, poi: DetectedObject) -> None:
        self._mark_failed_and_log(
            lambda: (
                f"NAV_FAIL: navigation_rejected "
                f"task={get_poi_task_id(poi)} "
                f"obj={poi.identity.obj_id}"
            )
        )

    def mark_source_liveness_expired(
        self,
        poi: DetectedObject | None,
    ) -> None:
        self._mark_failed_and_log(
            lambda: (
                "NAV_FAIL: source_command_stale "
                + self._poi_identity(poi)
            )
        )

    def _mark_failed_and_log(self, message: Callable[[], str]) -> None:
        self._ports.mark_failed()
        try:
            self._ports.logger.info(
                message(),
                key="nav",
                dest=LogStatusDest.DRONE,
            )
        except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
            return

    @staticmethod
    def _poi_identity(poi: DetectedObject | None) -> str:
        if poi is None:
            return "active_poi=missing"
        return (
            f"task={get_poi_task_id(poi)} "
            f"obj={poi.identity.obj_id}"
        )


__all__ = [
    "FinalApproachCommandDispatch",
    "FinalApproachCommandPorts",
    "FinalApproachCommandPoiResolver",
]
