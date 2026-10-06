"""Terminal record commit and measured-frame command dispatch."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.terminal_source_contracts import NavSourceBatch
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.target_identity import get_target_task_id


@dataclass(frozen=True)
class TerminalCommandPorts:
    nav: Callable[[Optional[DetectedObject]], bool]
    mark_failed: Callable[[], None]
    logger: ILogger


class TerminalCommandTargetResolver(Protocol):
    def source_event_target(
        self,
        event: Optional[DetectionPublication],
        active_target: DetectedObject,
    ) -> Optional[DetectedObject]: ...


class TerminalCommandDispatch:
    """Own measured-frame dispatch and terminal failure reporting."""

    def __init__(
        self,
        ports: TerminalCommandPorts,
        source: TerminalCommandTargetResolver,
    ) -> None:
        self._ports = ports
        self._source = source

    def dispatch(
        self,
        batch: NavSourceBatch,
        fresh_target: Optional[DetectedObject],
        active_target: DetectedObject,
    ) -> bool:
        if not batch.source_driven:
            if not self._ports.nav(fresh_target):
                self.mark_navigation_rejected(fresh_target or active_target)
                return False
            return True
        if batch.latest_event is None:
            self.mark_navigation_rejected(active_target)
            return False
        return self.dispatch_source_publication(
            batch.latest_event,
            active_target,
        )

    def dispatch_source_publication(
        self,
        publication: DetectionPublication,
        active_target: DetectedObject,
    ) -> bool:
        nav_target = self.source_publication_target(
            publication,
            active_target,
        )
        if nav_target is None:
            self.mark_navigation_rejected(active_target)
            return False
        return self.dispatch_target(nav_target)

    def source_publication_target(
        self,
        publication: DetectionPublication,
        active_target: DetectedObject,
    ) -> DetectedObject | None:
        return self._source.source_event_target(publication, active_target)

    def dispatch_target(self, nav_target: DetectedObject) -> bool:
        if not self._ports.nav(nav_target):
            self.mark_navigation_rejected(nav_target)
            return False
        return True

    def mark_no_detection(self, active_target: DetectedObject) -> None:
        self._mark_failed_and_log(
            lambda: (
                f"NAV_FAIL: no_detection "
                f"task={get_target_task_id(active_target)} "
                f"obj={active_target.identity.obj_id}"
            )
        )

    def mark_navigation_rejected(self, target: DetectedObject) -> None:
        self._mark_failed_and_log(
            lambda: (
                f"NAV_FAIL: navigation_rejected "
                f"task={get_target_task_id(target)} "
                f"obj={target.identity.obj_id}"
            )
        )

    def mark_source_liveness_expired(
        self,
        target: DetectedObject | None,
    ) -> None:
        self._mark_failed_and_log(
            lambda: (
                "NAV_FAIL: source_command_stale "
                + self._target_identity(target)
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
    def _target_identity(target: DetectedObject | None) -> str:
        if target is None:
            return "active_target=missing"
        return (
            f"task={get_target_task_id(target)} "
            f"obj={target.identity.obj_id}"
        )


__all__ = [
    "TerminalCommandDispatch",
    "TerminalCommandPorts",
    "TerminalCommandTargetResolver",
]
