"""Stable camera-mount facade over focused optics and hardware owners."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from navpy.modules.vision.camera_mount_composition import build_camera_mount_parts
from navpy.modules.vision.camera_mount_facets import (
    CameraMountFrameFacet,
    CameraMountLifecycleFacet,
    CameraMountOpticsFacet,
    CameraMountPoiZoomFacet,
    CameraMountZoomControlFacet,
    CameraMountZoomReadbackFacet,
)
from navpy.modules.vision.camera_mount_types import (
    CameraMountFrameState,
    CameraMountOptics,
)
from navpy.modules.vision.camera_mount_zoom_readback import (
    LIVE_ZOOM_READBACK_STALE_S,
    _format_cmd_key as _format_cmd_key,
)

if TYPE_CHECKING:
    from navpy.modules.vision.peripheral.camera_abc import CameraAbc
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc
    from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable


@dataclass
class CameraMount(
    CameraMountOpticsFacet,
    CameraMountFrameFacet,
    CameraMountZoomReadbackFacet,
    CameraMountZoomControlFacet,
    CameraMountLifecycleFacet,
    CameraMountPoiZoomFacet,
):
    """Public compatibility boundary for one camera and one gimbal."""

    name: str
    camera: "CameraAbc"
    gimbal: "GimbalAbc"
    zoom_calibration: Optional["ZoomCalibrationTable"] = None
    _optics_lock: threading.RLock = field(
        default_factory=threading.RLock,
        init=False,
        repr=False,
        compare=False,
    )
    def __post_init__(self) -> None:
        self._parts = build_camera_mount_parts(
            lambda: self.camera,
            lambda: self.gimbal,
            lambda: self.zoom_calibration,
            self._optics_lock,
        )


__all__ = [
    "CameraMount",
    "CameraMountFrameState",
    "CameraMountOptics",
    "LIVE_ZOOM_READBACK_STALE_S",
]
