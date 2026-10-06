"""Ordered NAV command-fence and resource teardown."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.navigation_speedup import NavigationSpeedupLease


@dataclass(frozen=True)
class NavExitCleanupPorts:
    close_terminal_source: Callable[[], None]
    navigation_reset: Callable[[], object]
    detector_stop: Callable[[], None]


@dataclass(frozen=True)
class NavCleanupResult:
    snap: object | None
    errors: tuple[Exception, ...]


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

        source_closed = False
        try:
            self._ports.close_terminal_source()
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
        return NavCleanupResult(snap=snap, errors=tuple(errors))


__all__ = [
    "NavCleanupResult",
    "NavExitCleanup",
    "NavExitCleanupPorts",
]
