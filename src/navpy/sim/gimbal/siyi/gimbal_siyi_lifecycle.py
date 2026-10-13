"""Idempotent SIYI simulator lifecycle."""

from __future__ import annotations

import threading
from collections.abc import Callable

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.sim.gimbal.gimbal_cadence import FixedCadenceRunner
from navpy.sim.gimbal.siyi.gimbal_angles import MODE_FOLLOW
from navpy.sim.gimbal.siyi.gimbal_siyi_state import (
    SiyiSimState,
)


class SiyiSimLifecycle:
    def __init__(
        self,
        state: SiyiSimState,
        runner: FixedCadenceRunner,
        logger: ILogger,
        catch_up: Callable[[], None] | None = None,
    ) -> None:
        self._state = state
        self._runner = runner
        self._logger = logger
        # Pays owed aircraft time before start/stop clears the commands.
        self._catch_up = catch_up
        self._lifecycle_lock = threading.Lock()

    def start(self) -> None:
        with self._lifecycle_lock:
            self._pay_owed_time()
            with self._state.lock:
                if self._state.started and self._runner.is_running:
                    return
                self._state.started = False
                self._clear_commands()
            started = self._runner.start()
            if not started:
                raise RuntimeError("previous SIYI simulator runner is still active")
            with self._state.lock:
                self._state.started = True
        self._logger.info("SIYI ZR10 simulator started")

    def stop(self) -> bool:
        with self._lifecycle_lock:
            self._pay_owed_time()
            with self._state.lock:
                self._state.started = False
                self._clear_commands()
            return self._runner.stop()

    def is_connected(self) -> bool:
        with self._state.lock:
            return self._state.started and self._runner.is_running

    def raise_if_failed(self) -> None:
        self._runner.raise_if_failed()

    def _pay_owed_time(self) -> None:
        if self._catch_up is not None:
            self._catch_up()

    def _clear_commands(self) -> None:
        self._state.angular.set_speed(0.0, 0.0)
        self._state.zoom.stop()
        self._state.angular.set_motion_mode(MODE_FOLLOW)


__all__ = ["SiyiSimLifecycle"]
