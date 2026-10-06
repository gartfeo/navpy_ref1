"""Hardware zoom validation and command-space conversion."""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import TYPE_CHECKING

from navpy.modules.vision.camera_mount_ports import (
    CameraZoomPort,
    GimbalZoomReadbackPort,
)
from navpy.modules.vision.camera_mount_types import LiveZoomSample

if TYPE_CHECKING:
    from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable


LIVE_ZOOM_READBACK_STALE_S = 2.0


def _format_cmd_key(command: float) -> str:
    return f"{command:.2f}"


class MountZoomReadback:
    """Own all interpretation of raw gimbal zoom telemetry."""

    def __init__(
        self,
        camera: CameraZoomPort,
        gimbal: GimbalZoomReadbackPort,
        calibration: Callable[[], "ZoomCalibrationTable | None"],
    ) -> None:
        self._camera = camera
        self._gimbal = gimbal
        self._calibration = calibration

    def current_zoom(self) -> str | None:
        level = self.hardware_level(require_fresh=False)
        return str(level) if level is not None else self.camera_zoom_key()

    def camera_zoom_key(self) -> str | None:
        return self._camera.zoom_key()

    def zoom_key_from_hardware(self) -> str | None:
        level = self.hardware_level(require_fresh=False)
        if level is None:
            return None
        return self.zoom_key_from_level(level)

    def zoom_key_from_level(self, level: float) -> str | None:
        calibration = self._calibration()
        if calibration is None:
            return str(level)
        command = calibration.readback_to_command(float(level))
        return None if command is None else _format_cmd_key(command)

    def hardware_level(self, *, require_fresh: bool) -> float | None:
        if require_fresh and not self.hardware_is_fresh():
            return None
        try:
            level = self._gimbal.zoom_level()
        except (TypeError, ValueError):
            return None
        return self.valid_level(level)

    def hardware_is_fresh(self) -> bool:
        if self._gimbal.has_zoom_age():
            try:
                age = float(self._gimbal.zoom_age())
            except (TypeError, ValueError):
                return False
            return self.age_is_fresh(age)
        if self._gimbal.has_zoom_fresh_flag():
            return self._gimbal.zoom_fresh_flag() is True
        return False

    @staticmethod
    def valid_level(level: object) -> float | None:
        if (
            isinstance(level, bool)
            or not isinstance(level, (int, float))
            or level <= 0
        ):
            return None
        if not math.isfinite(float(level)):
            return None
        return level

    @staticmethod
    def age_is_fresh(age: float) -> bool:
        return (
            math.isfinite(age)
            and age >= 0.0
            and age <= LIVE_ZOOM_READBACK_STALE_S
        )

    def live_sample(self) -> LiveZoomSample | None:
        try:
            value = self._gimbal.zoom_sample()
        except (TypeError, ValueError):
            return None
        if value is None:
            return None
        try:
            raw_level, raw_age, sample_id = value
        except (TypeError, ValueError):
            return None
        level = self.valid_level(raw_level)
        if level is None:
            return None
        try:
            age = float(raw_age)
        except (TypeError, ValueError):
            return None
        if not self.age_is_fresh(age) or sample_id is None:
            return None
        return LiveZoomSample(level, str(sample_id))

    def current_command(self) -> str | None:
        zoom_key = self.zoom_key_from_hardware()
        return zoom_key if zoom_key is not None else self.camera_zoom_key()

    def fresh_command(self) -> str | None:
        sample = self.live_sample()
        if sample is not None:
            return self.zoom_key_from_level(sample.level)
        level = self.hardware_level(require_fresh=True)
        return None if level is None else self.zoom_key_from_level(level)

    def fresh_sample_id(self) -> str | None:
        sample = self.live_sample()
        return None if sample is None else sample.sample_id

    def has_zoom_readback(self) -> bool:
        return self._gimbal.supports_zoom_readback()


__all__ = ["LIVE_ZOOM_READBACK_STALE_S", "MountZoomReadback"]
