"""Simulation-safe one-shot completion: a dock disarms in simulation only; a failed dock never disarms and asks
for RTL instead, in simulation and on real hardware (owner ruling 2026-11-09, navpy_recovery.md step 10, item 3)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.flow.navigation_task_reset import NavigationTaskResetTransaction


@dataclass(frozen=True)
class OneShotCompletionPorts:
    is_simulated_autopilot: Callable[[], bool]
    is_stopping: Callable[[], bool]
    disarm: Callable[[], None]
    #: RecoveryHold.request_sortie_rtl: RTL through recovery's ownership (nav_sortie_return.py).
    request_rtl: Callable[[], None]


class OneShotCompletion:
    """Latch one-shot completion without ever disarming real hardware."""

    def __init__(
        self,
        ports: OneShotCompletionPorts,
        reset: NavigationTaskResetTransaction,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._reset = reset
        self._logger = logger

    def run(self, failed: bool = False) -> tuple[Exception, ...]:
        """``failed``: the sortie ended in a failed dock (DOCK_FAILED), not a dock."""
        errors: list[Exception] = []
        if failed:
            self._return(errors)
        else:
            self._complete_dock(errors)
        try:
            self._reset.clear()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        return tuple(errors)

    def _return(self, errors: list[Exception]) -> None:
        try:
            self._ports.request_rtl()
            self._logger.info("One-shot dock failed: no disarm, RTL requested through recovery", key="nav")
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)

    def _complete_dock(self, errors: list[Exception]) -> None:
        simulated: bool | None = None
        try:
            simulated = self._ports.is_simulated_autopilot()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        if simulated:
            self._complete_simulation(errors)
        elif simulated is False:
            try:
                self._logger.warning(
                    "One-shot disarm SUPPRESSED: real autopilot "
                    "(no simulator SIM_SPEEDUP control). "
                    "NavPy will not force-disarm an airframe in flight; "
                    "one-shot stops new navigation tasks without motor-off.",
                    key="nav",
                )
            except Exception as error:  # noqa: BLE001 - teardown diagnostics
                errors.append(error)

    def _complete_simulation(self, errors: list[Exception]) -> None:
        stopping: bool | None = None
        try:
            stopping = self._ports.is_stopping()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        if stopping is False:
            try:
                self._ports.disarm()
            except Exception as error:  # noqa: BLE001 - teardown transaction
                errors.append(error)


__all__ = ["OneShotCompletion", "OneShotCompletionPorts"]
