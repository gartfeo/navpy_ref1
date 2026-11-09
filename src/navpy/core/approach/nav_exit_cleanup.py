"""Ordered NAV command-fence and resource teardown."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.core.approach.vision_nav.episode_latch import (
    GuidanceEpisodeResult,
)
from navpy.core.kernel.exception_groups import ExceptionGroup
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.dock_contact import DockContactEvent
from navpy.core.flow.navigation_speedup import NavigationSpeedupLease


class DockOutcomePort(Protocol):
    """How the armed episode ended (missed_dock.md); never a command input."""

    def result(self) -> Optional[GuidanceEpisodeResult]: ...

    def record_failed(self, result: GuidanceEpisodeResult, reason: str) -> None: ...


@dataclass(frozen=True)
class NavExitCleanupPorts:
    disarm_dock_contact: Callable[[], Optional[DockContactEvent]]
    close_guidance_source: Callable[[], None]
    navigation_reset: Callable[[], object]
    detector_stop: Callable[[], None]
    # Its result is read before the reset clears it; None: no vision runtime.
    dock_outcome: Optional[DockOutcomePort] = None
    #: The held episode's dock-contact end (GuidanceDockContact.disarm_held); None: ``disarm_dock_contact``.
    disarm_held_dock_contact: Optional[Callable[[], Optional[DockContactEvent]]] = None


@dataclass(frozen=True)
class NavCleanupResult:
    snap: object | None
    errors: tuple[Exception, ...]
    # The armed episode's own-dock contact, captured before the reset.
    own_dock_contact: DockContactEvent | None = None
    # The armed episode's first result (missed_dock.md), before the reset.
    dock_result: GuidanceEpisodeResult | None = None


class NavExitCleanup:
    """Require source quiescence before resetting command ownership."""

    def __init__(
        self,
        ports: NavExitCleanupPorts,
        speedup: NavigationSpeedupLease,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._speedup = speedup
        self._logger = logger

    def run(self) -> NavCleanupResult:
        errors: list[Exception] = []
        snap = None
        contact = None
        result = None

        try:
            contact = self._ports.disarm_dock_contact()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        try:
            outcome = self._ports.dock_outcome
            result = None if outcome is None else outcome.result()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        source_closed = False
        try:
            self._ports.close_guidance_source()
            source_closed = True
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        if source_closed:
            try:
                snap = self._ports.navigation_reset()
            except Exception as error:  # noqa: BLE001 - teardown transaction
                errors.append(error)
        try:
            self._ports.detector_stop()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        try:
            self._logger.defer_status_texts(False, flush=True)
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        try:
            if not self._speedup.restore(log=True):
                errors.append(
                    RuntimeError("SIM_SPEEDUP rollback remains unverified")
                )
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        return NavCleanupResult(
            snap=snap,
            errors=tuple(errors),
            own_dock_contact=contact,
            dock_result=result,
        )

    def end_episode(self) -> None:
        """Recovery's hold ends the guidance episode, without failure (nav_held_approach.py): the dock-contact
        episode, the source admission and the command session (``navigation_reset``: commands, liveness and the
        episode latch) end; tracking, zoom, the speedup and the task stay for the fresh episode."""
        errors: list[Exception] = []
        disarm = self._ports.disarm_held_dock_contact or self._ports.disarm_dock_contact
        for step in (disarm, self._ports.close_guidance_source, self._ports.navigation_reset):
            try:
                step()
            except Exception as error:  # noqa: BLE001 - every step runs; the NAV loop fails the cycle on the group
                errors.append(error)
        if errors:
            raise ExceptionGroup("held guidance episode did not end", errors)

    def record_dock_failed(
        self,
        result: GuidanceEpisodeResult,
        reason: str,
    ) -> None:
        """The DOCK_FAILED row, once the NAV exit succeeded."""
        if self._ports.dock_outcome is not None:
            self._ports.dock_outcome.record_failed(result, reason)


__all__ = [
    "DockOutcomePort",
    "NavCleanupResult",
    "NavExitCleanup",
    "NavExitCleanupPorts",
]
