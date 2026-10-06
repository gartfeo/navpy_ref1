"""Atomic gimbal, zoom-provenance, and camera-intrinsic frame capture."""

from __future__ import annotations

import copy
import math
import threading
import time

import numpy as np

from navpy.modules.vision.camera_mount_hardware import (
    FrameStateCapabilityUnavailable,
)
from navpy.modules.vision.camera_mount_ports import (
    CameraZoomPort,
    FrameOpticsPort,
    FrameZoomReadbackPort,
    GimbalFramePort,
)
from navpy.modules.vision.camera_mount_types import CameraMountFrameState
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class MountFrameStateCapture:
    """Freeze all frame-local camera state under the shared optics lock."""

    def __init__(
        self,
        camera: CameraZoomPort,
        gimbal: GimbalFramePort,
        optics: FrameOpticsPort,
        readback: FrameZoomReadbackPort,
        lock: threading.RLock,
    ) -> None:
        self._camera = camera
        self._gimbal = gimbal
        self._optics = optics
        self._readback = readback
        self._lock = lock

    def capture(
        self,
        frame_w: int,
        frame_h: int,
    ) -> CameraMountFrameState | None:
        with self._lock:
            return self._capture_locked(frame_w, frame_h)

    def _capture_locked(
        self,
        frame_w: int,
        frame_h: int,
    ) -> CameraMountFrameState | None:
        is_static = self._gimbal.state_is_static()
        zoom_command = self._readback.camera_zoom_key()
        zoom_sample_id: str | None = None
        zoom_sample_age_s: float | None = None

        if is_static:
            gimbal_data = copy.deepcopy(self._gimbal.get_data())
            zoom_sample_id = f"static:{zoom_command or 'fixed'}"
            zoom_sample_age_s = 0.0
        else:
            captured = self._capture_dynamic_gimbal()
            if captured is None:
                return None
            gimbal_data, zoom_level, sample_age_s, sample_id, sample_read_s = captured
            if self._readback.has_zoom_readback():
                candidate_level = self._readback.valid_level(zoom_level)
                try:
                    candidate_age_s = float(sample_age_s)
                except (TypeError, ValueError):
                    candidate_age_s = None
                candidate_command = (
                    None
                    if candidate_level is None
                    else self._readback.zoom_key_from_level(candidate_level)
                )
                candidate_is_proven = (
                    candidate_level is not None
                    and sample_id is not None
                    and candidate_age_s is not None
                    and math.isfinite(candidate_age_s)
                    and candidate_age_s >= 0.0
                    and candidate_command is not None
                )
                if (
                    candidate_is_proven
                    and self._readback.camera_zoom_key() != candidate_command
                    and not self._camera.apply_zoom(candidate_command).accepted
                ):
                    candidate_is_proven = False
                if candidate_is_proven:
                    zoom_command = candidate_command
                    zoom_sample_id = str(sample_id)
                    zoom_sample_age_s = candidate_age_s + max(
                        0.0,
                        time.monotonic() - sample_read_s,
                    )
                else:
                    zoom_command = self._readback.camera_zoom_key()
            else:
                zoom_sample_id = f"camera:{zoom_command or 'fixed'}"
                zoom_sample_age_s = 0.0

        return self._freeze_result(
            frame_w,
            frame_h,
            gimbal_data,
            is_static,
            zoom_command,
            zoom_sample_id,
            zoom_sample_age_s,
        )

    def _capture_dynamic_gimbal(
        self,
    ) -> tuple[GimbalData, object, object, object, float] | None:
        try:
            gimbal_data, zoom_level, sample_age_s, sample_id = (
                self._gimbal.frame_state_sample()
            )
        except FrameStateCapabilityUnavailable:
            gimbal_data = self._gimbal.get_data()
            zoom_level = None
            sample_age_s = None
            sample_id = None
        sample_read_s = time.monotonic()
        if not isinstance(gimbal_data, GimbalData):
            return None
        return (
            copy.deepcopy(gimbal_data),
            zoom_level,
            sample_age_s,
            sample_id,
            sample_read_s,
        )

    def _freeze_result(
        self,
        frame_w: int,
        frame_h: int,
        gimbal_data: GimbalData,
        is_static: bool,
        zoom_command: str | None,
        zoom_sample_id: str | None,
        zoom_sample_age_s: float | None,
    ) -> CameraMountFrameState | None:
        try:
            matrix = np.array(
                self._optics.get_k_for_frame_locked(frame_w, frame_h),
                dtype=float,
                copy=True,
            )
            distortion = np.array(
                self._optics.get_dist_locked(),
                dtype=float,
                copy=True,
            )
        except (TypeError, ValueError):
            return None
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            return None
        if not np.all(np.isfinite(distortion)):
            return None
        matrix.setflags(write=False)
        distortion.setflags(write=False)
        return CameraMountFrameState(
            gimbal_data=gimbal_data,
            k=matrix,
            dist=distortion,
            gimbal_timestamp_s=getattr(gimbal_data, "timestamp_s", None),
            gimbal_is_static=is_static,
            zoom_command=zoom_command,
            zoom_sample_id=(
                None if zoom_sample_id is None else str(zoom_sample_id)
            ),
            zoom_sample_age_s=(
                None if zoom_sample_age_s is None else float(zoom_sample_age_s)
            ),
        )


__all__ = ["MountFrameStateCapture"]
