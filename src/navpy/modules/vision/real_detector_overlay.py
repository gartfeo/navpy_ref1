"""Exact adapter from detector diagnostics to the mounted overlay renderer."""

from __future__ import annotations

import numpy as np

from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector_overlay import draw_debug_overlay
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_detector_ports import AircraftAttitudeReader


class MountedDetectorOverlayRenderer:
    """Bind the broad camera mount only at the detector composition edge."""

    def __init__(
        self,
        mount: CameraMount,
        vehicle_attitude: AircraftAttitudeReader,
    ) -> None:
        self._mount = mount
        self._vehicle_attitude = vehicle_attitude

    def render(
        self,
        frame: np.ndarray,
        tracks: list[TrackedObject],
        locked: TrackedObject | None,
        *,
        fps_est: float,
        use_lock: bool,
    ) -> np.ndarray:
        return draw_debug_overlay(
            frame,
            tracks,
            locked,
            mount=self._mount,
            vehicle_attitude=self._vehicle_attitude(),
            fps_est=fps_est,
            use_lock=use_lock,
        )


__all__ = ["MountedDetectorOverlayRenderer"]
