"""SIYI simulation command capability."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.attitude import Attitude
from navpy.sim.gimbal.siyi.gimbal_siyi_state import (
    SiyiSimState,
)


def _no_catch_up() -> None:
    return None


class SiyiSimControl:
    def __init__(
        self,
        state: SiyiSimState,
        logger: ILogger,
        catch_up: Callable[[], None] = _no_catch_up,
    ) -> None:
        self._state = state
        self._logger = logger
        # Brings the plant to the aircraft's current time before a command.
        self._catch_up = catch_up

    def set_att(self, attitude: Attitude) -> None:
        self._catch_up()
        with self._state.lock:
            self._state.angular.set_target_angles(
                attitude.yaw,
                attitude.pitch,
            )

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        self._catch_up()
        with self._state.lock:
            self._state.angular.set_speed(yaw_rate, pitch_rate)

    def set_zoom(self, zoom: float | str) -> bool:
        try:
            level = float(zoom)
        except (TypeError, ValueError):
            return False
        if not 1.0 <= level < 31.0:
            return False
        self._catch_up()
        with self._state.lock:
            self._state.zoom.set_target(level)
        return True

    def zoom_in(self) -> bool:
        self._catch_up()
        with self._state.lock:
            self._state.zoom.start(1)
        return True

    def zoom_out(self) -> bool:
        self._catch_up()
        with self._state.lock:
            self._state.zoom.start(-1)
        return True

    def zoom_hold(self) -> bool:
        self._catch_up()
        with self._state.lock:
            self._state.zoom.stop()
        return True

    def set_motion_mode(self, mode: int) -> None:
        self._catch_up()
        with self._state.lock:
            self._state.angular.set_motion_mode(mode)

    def request_autofocus(self) -> None:
        self._logger.debug("GimbalSiyiSim: autofocus requested (no-op)")


__all__ = ["SiyiSimControl"]
