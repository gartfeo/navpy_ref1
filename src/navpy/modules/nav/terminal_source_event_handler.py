"""Validate and route one exclusive terminal source publication."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication


class TerminalSourceCommandRouter(Protocol):
    def source_publication_target(
        self,
        publication: DetectionPublication,
        active_target: DetectedObject,
    ) -> DetectedObject | None: ...

    def mark_navigation_rejected(self, target: DetectedObject) -> None: ...

    def dispatch_target(self, nav_target: DetectedObject) -> bool: ...


class TerminalEventAdmission(Protocol):
    def admit_event(
        self,
        publication: DetectionPublication,
        active_target: Optional[DetectedObject],
    ) -> bool: ...


@dataclass(frozen=True)
class TerminalSourceEventHandlerPorts:
    active_target: Callable[[], Optional[DetectedObject]]
    session_active: Callable[[], bool]
    reset_source: Callable[[], None]
    admission: TerminalEventAdmission
    mark_failed: Callable[[], None]
    logger: ILogger
    commands: TerminalSourceCommandRouter


class TerminalSourceEventHandler:
    """Apply source continuity and dispatch one active-target observation."""

    def __init__(self, ports: TerminalSourceEventHandlerPorts) -> None:
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
        active_target = self._ports.active_target()
        if active_target is None:
            self._mark_failed("NAV_FAIL: source_event_active_target_missing")
            return self._reject()
        if (
            publication.source_discontinuity
            and not named_discontinuity
            and not self._ports.admission.admit_event(
                publication,
                active_target,
            )
        ):
            return self._reject()
        nav_target = self._ports.commands.source_publication_target(
            publication,
            active_target,
        )
        if nav_target is None:
            if named_discontinuity:
                return True
            self._ports.commands.mark_navigation_rejected(active_target)
            return self._reject()
        if (
            not publication.source_discontinuity
            and not self._ports.admission.admit_event(
                publication,
                active_target,
            )
        ):
            return self._reject()
        if not self._ports.commands.dispatch_target(nav_target):
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
    "TerminalEventAdmission",
    "TerminalSourceCommandRouter",
    "TerminalSourceEventHandler",
    "TerminalSourceEventHandlerPorts",
]
