"""Stateless public facets for the hardware SIYI adapter."""

from __future__ import annotations

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.hardware.composition import (
    SiyiHardwareParts,
)


class SiyiHardwareLifecycleFacet:
    _parts: SiyiHardwareParts

    def start(self) -> None:
        self._parts.lifecycle.start()

    def stop(self) -> bool:
        return self._parts.lifecycle.stop()

    def is_connected(self) -> bool:
        return self._parts.lifecycle.is_connected()

    def raise_if_failed(self) -> None:
        self._parts.lifecycle.raise_if_failed()


class SiyiHardwareReadbackFacet:
    _parts: SiyiHardwareParts

    def get_data(self) -> GimbalData:
        return self._parts.readback.get_data()

    def get_frame_state_sample(
        self,
    ) -> tuple[GimbalData, float | None, float | None, int | None]:
        return self._parts.readback.get_frame_state_sample()

    def get_zoom_level(self) -> float | None:
        return self._parts.readback.get_zoom_level()

    def get_zoom_level_age_s(self) -> float:
        return self._parts.readback.get_zoom_level_age_s()

    def supports_zoom_readback(self) -> bool:
        return self._parts.readback.supports_zoom_readback()

    def get_zoom_level_sample_id(self) -> int | None:
        return self._parts.readback.get_zoom_level_sample_id()

    def get_zoom_level_sample(self) -> tuple[float, float, int] | None:
        return self._parts.readback.get_zoom_level_sample()


class SiyiHardwareControlFacet:
    _parts: SiyiHardwareParts

    def set_att(self, att: Attitude) -> None:
        self._parts.control.set_att(att)

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        self._parts.control.set_rate(yaw_rate, pitch_rate)

    def set_motion_mode(self, mode: int) -> None:
        self._parts.control.set_motion_mode(mode)

    def set_zoom(self, zoom: float | str) -> bool:
        return self._parts.control.set_zoom(zoom)

    def supports_absolute_zoom_control(self) -> bool:
        return True

    def supports_continuous_zoom_control(self) -> bool:
        return True

    def zoom_in(self) -> bool:
        return self._parts.control.zoom_in()

    def zoom_out(self) -> bool:
        return self._parts.control.zoom_out()

    def zoom_hold(self) -> bool:
        return self._parts.control.zoom_hold()

    def request_autofocus(self) -> None:
        self._parts.control.request_autofocus()


__all__ = [
    "SiyiHardwareControlFacet",
    "SiyiHardwareLifecycleFacet",
    "SiyiHardwareReadbackFacet",
]
