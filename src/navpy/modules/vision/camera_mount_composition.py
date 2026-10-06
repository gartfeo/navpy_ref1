"""Composition root for focused camera-mount collaborators."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from navpy.modules.vision.camera_mount_control import (
    MountLifecycle,
    MountZoomControl,
)
from navpy.modules.vision.camera_mount_frame_state import MountFrameStateCapture
from navpy.modules.vision.camera_mount_hardware import (
    MountCameraHardware,
    MountGimbalHardware,
    MountGimbalTargetZoomHardware,
    MountGimbalZoomReadbackHardware,
)
from navpy.modules.vision.camera_mount_optics import (
    MountCalibrationSummary,
    MountLiveOptics,
    MountOpticsReader,
)
from navpy.modules.vision.camera_mount_zoom_readback import MountZoomReadback

if TYPE_CHECKING:
    from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable


@dataclass(frozen=True)
class CameraMountParts:
    lock: threading.RLock
    camera: MountCameraHardware
    gimbal: MountGimbalHardware
    gimbal_readback: MountGimbalZoomReadbackHardware
    gimbal_target_zoom: MountGimbalTargetZoomHardware
    optics: MountOpticsReader
    calibration: MountCalibrationSummary
    readback: MountZoomReadback
    live_optics: MountLiveOptics
    frame_capture: MountFrameStateCapture
    zoom_control: MountZoomControl
    lifecycle: MountLifecycle


def build_camera_mount_parts(
    camera_source: Callable[[], object],
    gimbal_source: Callable[[], object],
    calibration_source: Callable[[], "ZoomCalibrationTable | None"],
    lock: threading.RLock,
) -> CameraMountParts:
    camera = MountCameraHardware(camera_source)
    gimbal = MountGimbalHardware(gimbal_source)
    gimbal_readback = MountGimbalZoomReadbackHardware(gimbal_source)
    gimbal_target_zoom = MountGimbalTargetZoomHardware(gimbal_source)
    optics = MountOpticsReader(camera, lock)
    calibration = MountCalibrationSummary(camera)
    readback = MountZoomReadback(camera, gimbal_readback, calibration_source)
    live_optics = MountLiveOptics(camera, optics, readback, lock)
    frame_capture = MountFrameStateCapture(
        camera,
        gimbal,
        optics,
        readback,
        lock,
    )
    zoom_control = MountZoomControl(camera, gimbal, readback, lock)
    lifecycle = MountLifecycle(camera, gimbal, lock)
    return CameraMountParts(
        lock=lock,
        camera=camera,
        gimbal=gimbal,
        gimbal_readback=gimbal_readback,
        gimbal_target_zoom=gimbal_target_zoom,
        optics=optics,
        calibration=calibration,
        readback=readback,
        live_optics=live_optics,
        frame_capture=frame_capture,
        zoom_control=zoom_control,
        lifecycle=lifecycle,
    )


__all__ = ["CameraMountParts", "build_camera_mount_parts"]
