"""Stateless compatibility facets for the public ``CameraMount`` surface."""

from __future__ import annotations

import numpy as np

from navpy.modules.vision.camera_mount_composition import CameraMountParts
from navpy.modules.vision.camera_mount_types import (
    CameraMountFrameState,
    CameraMountOptics,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


class CameraMountOpticsFacet:
    _parts: CameraMountParts

    def get_k(self) -> np.ndarray:
        return self._parts.optics.get_k()

    def get_k_for_frame(self, frame_w: int, frame_h: int) -> np.ndarray:
        return self._parts.optics.get_k_for_frame(frame_w, frame_h)

    def _get_k_for_frame_locked(self, frame_w: int, frame_h: int) -> np.ndarray:
        return self._parts.optics.get_k_for_frame_locked(frame_w, frame_h)

    def get_dist(self) -> np.ndarray:
        return self._parts.optics.get_dist()

    def _get_dist_locked(self) -> np.ndarray:
        return self._parts.optics.get_dist_locked()

    @property
    def image_width(self) -> int | None:
        return self._parts.optics.image_width

    @property
    def image_height(self) -> int | None:
        return self._parts.optics.image_height

    def is_valid(self, u: float, v: float) -> bool:
        return self._parts.optics.is_valid(u, v)

    def get_zoom_levels(self) -> list[str]:
        return self._parts.calibration.zoom_levels()

    @property
    def zoom_ratio(self) -> float:
        return self._parts.calibration.zoom_ratio

    @property
    def base_fy(self) -> float | None:
        return self._parts.calibration.base_fy

    @property
    def has_zoom(self) -> bool:
        return self._parts.calibration.has_zoom

    def get_live_optics(self) -> CameraMountOptics | None:
        return self._parts.live_optics.get()

    def _get_live_optics_locked(self) -> CameraMountOptics | None:
        return self._parts.live_optics._get_locked()

    def _optics_from_intrinsics(
        self,
        *,
        zoom_command: str | None,
        zoom_level: str | None,
        sample_id: str,
        sample_from_hardware: bool,
    ) -> CameraMountOptics | None:
        return self._parts.live_optics._from_intrinsics(
            zoom_command=zoom_command,
            zoom_level=zoom_level,
            sample_id=sample_id,
            sample_from_hardware=sample_from_hardware,
        )


class CameraMountFrameFacet:
    _parts: CameraMountParts

    def get_gimbal_data(self) -> GimbalData:
        return self._parts.gimbal.get_data()

    def capture_frame_state(
        self,
        frame_w: int,
        frame_h: int,
    ) -> CameraMountFrameState | None:
        return self._parts.frame_capture.capture(frame_w, frame_h)

    def _capture_frame_state_locked(
        self,
        frame_w: int,
        frame_h: int,
    ) -> CameraMountFrameState | None:
        return self._parts.frame_capture._capture_locked(frame_w, frame_h)


class CameraMountZoomReadbackFacet:
    _parts: CameraMountParts

    def get_current_zoom(self) -> str | None:
        with self._parts.lock:
            return self._parts.readback.current_zoom()

    def _camera_zoom_key(self) -> str | None:
        return self._parts.readback.camera_zoom_key()

    def _zoom_key_from_hardware(self) -> str | None:
        return self._parts.readback.zoom_key_from_hardware()

    def _zoom_key_from_hardware_level(self, level: float) -> str | None:
        return self._parts.readback.zoom_key_from_level(level)

    def _hardware_zoom_level(self, *, require_fresh: bool) -> float | None:
        return self._parts.readback.hardware_level(require_fresh=require_fresh)

    def _hardware_zoom_is_fresh(self) -> bool:
        return self._parts.readback.hardware_is_fresh()

    def _valid_hardware_zoom_level(self, level: object) -> float | None:
        return self._parts.readback.valid_level(level)

    def _hardware_zoom_age_is_fresh(self, age: float) -> bool:
        return self._parts.readback.age_is_fresh(age)

    def _live_hardware_zoom_sample(self) -> tuple[float, str] | None:
        sample = self._parts.readback.live_sample()
        if sample is None:
            return None
        return sample.level, sample.sample_id

    def get_current_zoom_command(self) -> str | None:
        with self._parts.lock:
            return self._parts.readback.current_command()

    def get_fresh_zoom_command(self) -> str | None:
        return self._parts.readback.fresh_command()

    def get_fresh_zoom_sample_id(self) -> str | None:
        return self._parts.readback.fresh_sample_id()

    def _gimbal_has_zoom_readback(self) -> bool:
        return self._parts.readback.has_zoom_readback()


class CameraMountZoomControlFacet:
    _parts: CameraMountParts

    def sync_zoom_from_hardware(self) -> bool:
        return self._parts.zoom_control.sync_from_hardware()

    def command_zoom(self, zoom: str | float) -> bool:
        return self._parts.zoom_control.command_hardware(zoom)

    def set_zoom(self, zoom: str | float) -> bool:
        return self._parts.zoom_control.set_zoom(zoom)


class CameraMountLifecycleFacet:
    _parts: CameraMountParts

    def start(self) -> None:
        self._parts.lifecycle.start()

    def stop(self) -> bool:
        return self._parts.lifecycle.stop()

    def refresh(self) -> None:
        self._parts.lifecycle.refresh()

    def raise_if_failed(self) -> None:
        self._parts.lifecycle.raise_if_failed()


class CameraMountTargetZoomFacet:
    _parts: CameraMountParts

    def zoom_actuator_identity(self) -> object:
        return self._parts.camera.identity, self._parts.gimbal.identity

    def supports_absolute_zoom(self) -> bool:
        return self._parts.gimbal_target_zoom.supports_absolute_zoom()

    def supports_continuous_zoom(self) -> bool:
        return self._parts.gimbal_target_zoom.supports_continuous_zoom()

    def start_continuous_zoom(self, direction: ZoomTrackingState) -> bool:
        if direction is ZoomTrackingState.ZOOMING_IN:
            return self._parts.gimbal_target_zoom.zoom_in()
        return self._parts.gimbal_target_zoom.zoom_out()

    def hold_zoom(self) -> bool:
        return self._parts.gimbal_target_zoom.zoom_hold()


__all__ = [
    "CameraMountFrameFacet",
    "CameraMountLifecycleFacet",
    "CameraMountOpticsFacet",
    "CameraMountTargetZoomFacet",
    "CameraMountZoomControlFacet",
    "CameraMountZoomReadbackFacet",
]
