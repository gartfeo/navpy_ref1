"""Ordered NAV command-fence and resource teardown."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.core.approach.vision_nav.episode_latch import (
    FinalApproachEpisodeTerminal,
)
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.dock_contact import DockContactEvent
from navpy.core.flow.navigation_speedup import NavigationSpeedupLease


class DockOutcomePort(Protocol):
    """How the armed episode ended (missed_dock.md); never a command input."""

    def terminal(self) -> Optional[FinalApproachEpisodeTerminal]: ...

    def record_failed(self, terminal: FinalApproachEpisodeTerminal, reason: str) -> None: ...


@dataclass(frozen=True)
class NavExitCleanupPorts:
    disarm_dock_contact: Callable[[], Optional[DockContactEvent]]
    close_final_approach_source: Callable[[], None]
    navigation_reset: Callable[[], object]
    detector_stop: Callable[[], None]
    # Its terminal is read before the reset clears it; None: no vision runtime.
    dock_outcome: Optional[DockOutcomePort] = None


@dataclass(frozen=True)
class NavCleanupResult:
    snap: object | None
    errors: tuple[Exception, ...]
    # The armed episode's own-dock contact, captured before the reset.
    own_dock_contact: DockContactEvent | None = None
    # The armed episode's first terminal (missed_dock.md), before the reset.
    dock_terminal: FinalApproachEpisodeTerminal | None = None


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
        terminal = None

        try:
            contact = self._ports.disarm_dock_contact()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        try:
            outcome = self._ports.dock_outcome
            terminal = None if outcome is None else outcome.terminal()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        source_closed = False
        try:
            self._ports.close_final_approach_source()
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
            dock_terminal=terminal,
        )

    def record_dock_failed(
        self,
        terminal: FinalApproachEpisodeTerminal,
        reason: str,
    ) -> None:
        """The DOCK_FAILED row, once the NAV exit succeeded."""
        if self._ports.dock_outcome is not None:
            self._ports.dock_outcome.record_failed(terminal, reason)


__all__ = [
    "DockOutcomePort",
    "NavCleanupResult",
    "NavExitCleanup",
    "NavExitCleanupPorts",
]
