"""SIYI simulation command capability."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_state import (
    SiyiSimState,
)


class SiyiSimControl:
    def __init__(self, state: SiyiSimState, logger: ILogger) -> None:
        self._state = state
        self._logger = logger

    def set_att(self, attitude: Attitude) -> None:
        with self._state.lock:
            self._state.angular.set_target_angles(
                attitude.yaw,
                attitude.pitch,
            )

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        with self._state.lock:
            self._state.angular.set_speed(yaw_rate, pitch_rate)

    def set_zoom(self, zoom: float | str) -> bool:
        try:
            level = float(zoom)
        except (TypeError, ValueError):
            return False
        if not 1.0 <= level < 31.0:
            return False
        with self._state.lock:
            self._state.zoom.set_target(level)
        return True

    def zoom_in(self) -> bool:
        with self._state.lock:
            self._state.zoom.start(1)
        return True

    def zoom_out(self) -> bool:
        with self._state.lock:
            self._state.zoom.start(-1)
        return True

    def zoom_hold(self) -> bool:
        with self._state.lock:
            self._state.zoom.stop()
        return True

    def set_motion_mode(self, mode: int) -> None:
        with self._state.lock:
            self._state.angular.set_motion_mode(mode)

    def request_autofocus(self) -> None:
        self._logger.debug("GimbalSiyiSim: autofocus requested (no-op)")


__all__ = ["SiyiSimControl"]
