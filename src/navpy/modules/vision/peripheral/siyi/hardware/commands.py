"""SIYI command translation and validation."""

from __future__ import annotations

from collections.abc import Callable

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.siyi.hardware.ports import SiyiSdkPort
from navpy.modules.vision.peripheral.siyi.hardware.session import SiyiSdkSession
from navpy.modules.vision.peripheral.siyi.hardware.state import SiyiReadbackStore


class SiyiHardwareCommands:
    def __init__(
        self,
        session: SiyiSdkSession,
        store: SiyiReadbackStore,
        logger: ILogger,
    ) -> None:
        self._session = session
        self._store = store
        self._logger = logger

    @staticmethod
    def _clamp_rate(value: float) -> int:
        return max(-100, min(100, int(round(value))))

    def set_att(self, att: Attitude) -> None:
        self._send(lambda sdk: sdk.requestSetAngles(att.yaw, att.pitch))

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        pitch_sign = self._store.snapshot().pitch_sign
        yaw_command = self._clamp_rate(yaw_rate)
        pitch_command = self._clamp_rate(pitch_sign * pitch_rate)
        self._send(
            lambda sdk: sdk.requestGimbalSpeed(yaw_command, pitch_command),
        )

    def set_motion_mode(self, mode: int) -> None:
        requests: dict[int, tuple[str, Callable[[SiyiSdkPort], bool]]] = {
            0: ("LOCK", lambda sdk: sdk.requestLockMode()),
            1: ("FOLLOW", lambda sdk: sdk.requestFollowMode()),
            2: ("FPV", lambda sdk: sdk.requestFPVMode()),
        }
        with self._session.borrow() as sdk:
            if sdk is None:
                return
            selected = requests.get(mode)
            if selected is None:
                self._logger.warning(
                    f"SIYI: ignoring unknown motion mode {mode}"
                )
                return
            label, request = selected
            result = request(sdk)
        if result is not True:
            raise RuntimeError(f"SIYI: set_motion_mode({label}) UDP send failed")

    def set_zoom(self, zoom: float | str) -> bool:
        try:
            level = float(zoom)
        except (TypeError, ValueError):
            return False
        result = self._send(lambda sdk: sdk.requestAbsoluteZoom(level))
        return result is True

    def zoom_in(self) -> bool:
        return self._send(lambda sdk: sdk.requestZoomIn()) is True

    def zoom_out(self) -> bool:
        return self._send(lambda sdk: sdk.requestZoomOut()) is True

    def zoom_hold(self) -> bool:
        return self._send(lambda sdk: sdk.requestZoomHold()) is True

    def request_autofocus(self) -> None:
        self._send(lambda sdk: sdk.requestAutoFocus())

    def _send(
        self,
        request: Callable[[SiyiSdkPort], bool],
    ) -> bool | None:
        with self._session.borrow() as sdk:
            return None if sdk is None else request(sdk)


__all__ = ["SiyiHardwareCommands"]
