"""Validate and route one exclusive final-approach source publication."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication


class FinalApproachSourceCommandRouter(Protocol):
    def source_publication_poi(
        self,
        publication: DetectionPublication,
        active_poi: DetectedObject,
    ) -> DetectedObject | None: ...

    def mark_navigation_rejected(self, poi: DetectedObject) -> None: ...

    def dispatch_poi(self, nav_poi: DetectedObject) -> bool: ...


class FinalApproachEventAdmission(Protocol):
    def admit_event(
        self,
        publication: DetectionPublication,
        active_poi: Optional[DetectedObject],
    ) -> bool: ...


@dataclass(frozen=True)
class FinalApproachSourceEventHandlerPorts:
    active_poi: Callable[[], Optional[DetectedObject]]
    session_active: Callable[[], bool]
    reset_source: Callable[[], None]
    admission: FinalApproachEventAdmission
    mark_failed: Callable[[], None]
    logger: ILogger
    commands: FinalApproachSourceCommandRouter


class FinalApproachSourceEventHandler:
    """Apply source continuity and dispatch one active-POI observation."""

    def __init__(self, ports: FinalApproachSourceEventHandlerPorts) -> None:
        self._ports = ports

    def handle(self, publication: DetectionPublication) -> bool:
        named_discontinuity = _has_named_discontinuity(publication)
        if named_discontinuity and not self._ports.admission.admit_event(
            publication,
            None,
        ):
            return self._reject()
        if not self._ports.session_active():
            # Navigation publishes a requested phase/mode change before the
            # NAV exit hook can fence this dispatch thread.  A publication
            # already inside the dispatch boundary may therefore observe the
            # intentionally closed session.  Reject and invalidate it, but do
            # not turn normal NAV teardown into a navigation failure.
            return self._reject()
        active_poi = self._ports.active_poi()
        if active_poi is None:
            self._mark_failed("NAV_FAIL: source_event_active_poi_missing")
            return self._reject()
        if (
            publication.source_discontinuity
            and not named_discontinuity
            and not self._ports.admission.admit_event(
                publication,
                active_poi,
            )
        ):
            return self._reject()
        nav_poi = self._ports.commands.source_publication_poi(
            publication,
            active_poi,
        )
        if nav_poi is None:
            if named_discontinuity:
                return True
            self._ports.commands.mark_navigation_rejected(active_poi)
            return self._reject()
        if (
            not publication.source_discontinuity
            and not self._ports.admission.admit_event(
                publication,
                active_poi,
            )
        ):
            return self._reject()
        if not self._ports.commands.dispatch_poi(nav_poi):
            return self._reject()
        return True

    def handle_reset(self) -> None:
        """Invalidate queued commands before detector reset can return."""
        self._ports.reset_source()

    def _reject(self) -> bool:
        self._ports.reset_source()
        return False

    def _mark_failed(self, message: str) -> None:
        self._ports.mark_failed()
        try:
            self._ports.logger.error(message)
        except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
            return


def _has_named_discontinuity(publication: DetectionPublication) -> bool:
    return (
        bool(publication.source_discontinuity)
        and isinstance(publication.source_name, str)
        and bool(publication.source_name)
    )

__all__ = [
    "FinalApproachEventAdmission",
    "FinalApproachSourceCommandRouter",
    "FinalApproachSourceEventHandler",
    "FinalApproachSourceEventHandlerPorts",
]
