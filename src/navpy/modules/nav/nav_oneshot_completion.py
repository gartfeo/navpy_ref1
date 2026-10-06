"""Simulation-safe one-shot completion."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.navigation_task_reset import NavigationTaskResetTransaction


@dataclass(frozen=True)
class OneShotCompletionPorts:
    is_simulated_autopilot: Callable[[], bool]
    is_stopping: Callable[[], bool]
    disarm: Callable[[], None]


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

    def run(self) -> tuple[Exception, ...]:
        errors: list[Exception] = []
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
        try:
            self._reset.clear()
        except Exception as error:  # noqa: BLE001 - teardown transaction
            errors.append(error)
        return tuple(errors)

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
