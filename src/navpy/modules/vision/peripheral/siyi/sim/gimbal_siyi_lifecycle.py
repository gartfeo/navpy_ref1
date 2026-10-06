"""Idempotent SIYI simulator lifecycle."""

from __future__ import annotations

import threading

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.gimbal_cadence import FixedCadenceRunner
from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import MODE_FOLLOW
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_state import (
    SiyiSimState,
)


class SiyiSimLifecycle:
    def __init__(
        self,
        state: SiyiSimState,
        runner: FixedCadenceRunner,
        logger: ILogger,
    ) -> None:
        self._state = state
        self._runner = runner
        self._logger = logger
        self._lifecycle_lock = threading.Lock()

    def start(self) -> None:
        with self._lifecycle_lock:
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
            with self._state.lock:
                self._state.started = False
                self._clear_commands()
            return self._runner.stop()

    def is_connected(self) -> bool:
        with self._state.lock:
            return self._state.started and self._runner.is_running

    def raise_if_failed(self) -> None:
        self._runner.raise_if_failed()

    def _clear_commands(self) -> None:
        self._state.angular.set_speed(0.0, 0.0)
        self._state.zoom.stop()
        self._state.angular.set_motion_mode(MODE_FOLLOW)


__all__ = ["SiyiSimLifecycle"]
