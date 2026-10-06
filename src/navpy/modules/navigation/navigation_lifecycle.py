"""Lifecycle coordination for Navigation and its background command session."""

from __future__ import annotations

import threading
from typing import Optional

from navpy.logger.navigation_logger import ClosestSnap, NavigationLogger
from navpy.modules.navigation.navigation_command_session import NavigationCommandSession
from navpy.modules.navigation.navigation_postprocess_dispatcher import (
    NavigationPostprocessFailure,
)
from navpy.modules.navigation.navigation_mode import NavigationModeSelector, NavigationModeState
from navpy.modules.navigation.legacy_destination_resolver import LegacyNavigationState


class NavigationLifecycle:
    """Coordinate phase resets without owning algorithm or target behavior."""

    def __init__(
        self,
        mode_state: NavigationModeState,
        selector: NavigationModeSelector,
        legacy_state: LegacyNavigationState,
        navigation_logger: NavigationLogger,
        command_session: NavigationCommandSession,
    ) -> None:
        self._lock = threading.RLock()
        self._mode_state = mode_state
        self._selector = selector
        self._legacy_state = legacy_state
        self._navigation_logger = navigation_logger
        self._command_session = command_session

    def start(self) -> ClosestSnap:
        with self._lock:
            snap = self.reset()
            self._command_session.start()
            return snap

    def init(self) -> None:
        with self._lock:
            active = self._selector.initialize()
            active.runtime.reset_phase()

    def reset(self) -> ClosestSnap:
        with self._lock:
            with self._mode_state.session() as previous:
                previous.runtime.invalidate_commands()
                algorithm, kp = self._selector.algorithm_info
                snap = self._navigation_logger.write_summary_and_reset(
                    algorithm=algorithm,
                    kp=kp,
                )
                self._legacy_state.reset()
                active = self._selector.initialize()
                active.runtime.reset_phase()
                return snap

    def pause(self) -> None:
        with self._lock:
            with self._mode_state.runtime_session() as runtime:
                runtime.invalidate_commands()

    def stop(self) -> None:
        with self._lock:
            try:
                self._command_session.stop()
            except NavigationPostprocessFailure:
                self._navigation_logger.close()
                raise
            self._navigation_logger.close()

    def raise_if_failed(self) -> None:
        self._command_session.raise_if_failed()

    @property
    def algorithm_info(self) -> tuple[str, Optional[float]]:
        return self._selector.algorithm_info


__all__ = ["NavigationLifecycle"]
