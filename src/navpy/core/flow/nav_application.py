"""Navigation application lifecycle boundary."""

from __future__ import annotations

from navpy.core.kernel.exception_groups import ExceptionGroup

from typing import Callable

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.flow.navigation_task_reset import NavigationTaskResetTransaction
from navpy.core.swarm.nav_network import NavNetworkRuntime
from navpy.core.flow.nav_state import ConfirmOverrideInbox
from navpy.core.flow.navigation_loop import NavigationLoop


def _run_shutdown_steps(
    steps: tuple[Callable[[], object], ...],
) -> None:
    errors: list[Exception] = []
    for action in steps:
        try:
            action()
        except Exception as error:  # noqa: BLE001 - preserve teardown failures
            errors.append(error)
    if errors:
        raise ExceptionGroup("navigation shutdown failed", errors)


class NavShutdown:
    """Fence navigation and release every navigation lifecycle resource."""

    def __init__(
        self,
        close_final_approach_source: Callable[[], None],
        navigation_reset: Callable[[], object],
        navigation_task_reset: NavigationTaskResetTransaction,
        network: NavNetworkRuntime,
        logger: ILogger,
    ) -> None:
        self._close_final_approach_source = close_final_approach_source
        self._navigation_reset = navigation_reset
        self._navigation_task_reset = navigation_task_reset
        self._network = network
        self._logger = logger

    def run(self, loop_stop: Callable[[], object]) -> None:
        # Quiescence is a hard gate: live cycle code may still touch navigation,
        # navigation task, and network resources until loop_stop returns.
        loop_stop()
        try:
            self._close_final_approach_source()
        except Exception as source_error:
            errors = [source_error]
            try:
                _run_shutdown_steps((self._network.stop, self._log_stopped))
            except ExceptionGroup as independent_errors:
                errors.extend(independent_errors.exceptions)
            raise ExceptionGroup("navigation shutdown failed", errors)
        _run_shutdown_steps((
            self._navigation_reset,
            self._navigation_task_reset.clear,
            self._network.stop,
            self._log_stopped,
        ))

    def _log_stopped(self) -> None:
        self._logger.info(
            "NavController thread stopped.",
            key="nav",
            status="NavController thread stopped",
            dest=LogStatusDest.DRONE,
        )


class NavApplication:
    """Own the four process-level navigation lifecycle capabilities."""

    def __init__(
        self,
        loop: NavigationLoop,
        network: NavNetworkRuntime,
        shutdown: NavShutdown,
        overrides: ConfirmOverrideInbox,
    ) -> None:
        self._loop = loop
        self._network = network
        self._shutdown = shutdown
        self._overrides = overrides

    def set_network(self, network: NetworkAbc | None) -> None:
        self._network.set_network(network)

    def start(self, loop_rate_hz: float) -> None:
        self._loop.start(loop_rate_hz)

    def stop(self) -> None:
        self._shutdown.run(self._loop.stop)

    def raise_if_failed(self) -> None:
        self._loop.raise_if_failed()
        self._network.raise_if_failed()

    def force_confirm_override(self, task_id: int) -> None:
        self._overrides.request(task_id)


__all__ = ["NavApplication", "NavShutdown"]
