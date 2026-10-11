"""NAV entry transaction."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.flow.nav_state import NavigationTaskState
from navpy.core.flow.navigation_zoom import ZoomController


@dataclass(frozen=True)
class NavEntryPorts:
    request_guided: Callable[[], None]
    navigation_init: Callable[[], None]
    final_approach_active: Callable[[], bool]


class NavEntry:
    """Initialize one navigation phase and its operator-status lease."""

    def __init__(
        self,
        ports: NavEntryPorts,
        navigation_task: NavigationTaskState,
        zoom: ZoomController,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._zoom = zoom
        self._logger = logger

    def run(self) -> None:
        self._navigation_task.nav_mode_observed = False
        self._navigation_task.final_approach_navigation_active = False
        self._navigation_task.final_approach_nav_completed = False
        if (
            self._ports.final_approach_active()
            and not self._zoom.freeze_final_approach_wide()
        ):
            raise RuntimeError("final approach requires minimum zoom")
        self._ports.request_guided()
        self._ports.navigation_init()
        if not self._ports.final_approach_active():
            self._zoom.set_recognition_demand(False)
        self._logger.info(
            "INIT: NAV MODE",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        self._logger.defer_status_texts(True)


__all__ = ["NavEntry", "NavEntryPorts"]
