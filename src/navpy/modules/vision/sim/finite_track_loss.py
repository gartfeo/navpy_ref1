"""Finite-camera track-loss diagnosis, separate from target projection."""

from __future__ import annotations

import numpy as np
import pymap3d

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.sim_camera_ports import ProjectionCameraPort
from navpy.modules.vision.sim.sim_runtime_ports import (
    PixelCalculator,
    TargetSnapshotReader,
)
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.vision_class_profile import (
    MIN_DETECT_PIXELS,
    get_class_detect_size,
)


class FiniteTrackLossDiagnoser:
    """Explain finite-camera track loss without owning target projection."""

    def __init__(
        self,
        camera: ProjectionCameraPort,
        calc_uv: PixelCalculator,
        target_snapshot: TargetSnapshotReader,
    ) -> None:
        self._camera = camera
        self._calc_uv = calc_uv
        self._target_snapshot = target_snapshot

    def diagnose(
        self,
        tracking_id: int,
        camera_location: Location,
        uas_attitude: Attitude,
        camera_matrix: np.ndarray,
        gimbal_data: GimbalData,
    ) -> str:
        for target in self._target_snapshot():
            if target.uid != tracking_id:
                continue
            return self._diagnose_target(
                target,
                camera_location,
                uas_attitude,
                camera_matrix,
                gimbal_data,
            )
        return f"uid={tracking_id}_not_in_targets"

    def _diagnose_target(
        self,
        target: SimulationObject,
        camera_location: Location,
        uas_attitude: Attitude,
        camera_matrix: np.ndarray,
        gimbal_data: GimbalData,
    ) -> str:
        p_ned = pymap3d.geodetic2ned(
            target.g_loc.lat,
            target.g_loc.lng,
            target.g_loc.alt,
            camera_location.lat,
            camera_location.lng,
            camera_location.alt,
        )
        distance_m = float(np.linalg.norm(p_ned))
        if distance_m <= 0.0:
            return "dist_zero"
        focal_y = float(camera_matrix[1, 1])
        pixels = focal_y * get_class_detect_size(0) / distance_m
        if pixels < MIN_DETECT_PIXELS and distance_m > gimbal_data.max_detect_distance:
            return f"too_far dist={distance_m:.0f}m px={pixels:.1f}"
        x_error, y_error = self._calc_uv(
            np.asarray(p_ned, dtype=float),
            camera_matrix,
            gimbal_data,
            uas_attitude,
        )
        if x_error is None or y_error is None:
            return f"behind_cam dist={distance_m:.0f}m px={pixels:.1f}"
        if not self._camera.pixel_valid(x_error, y_error):
            return f"out_of_fov uv=({x_error:.0f},{y_error:.0f})"
        return f"unknown dist={distance_m:.0f}m px={pixels:.1f}"


__all__ = ["FiniteTrackLossDiagnoser"]
