"""Small dispatcher over finite and ideal simulator target projectors."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import (
    DetectedObject,
    DetectResult,
    DetectStatus,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.finite_target_projector import FiniteTargetProjector
from navpy.modules.vision.sim.ideal_camera_state import copy_gimbal_data as _copy_gimbal_data
from navpy.modules.vision.sim.ideal_target_projector import IdealTargetProjector
from navpy.modules.vision.simulation_object import SimulationObject


class SimTargetProjector:
    """Select one immutable projection policy for the detector lifetime."""

    def __init__(
        self,
        ideal_360: bool,
        finite: FiniteTargetProjector,
        ideal: IdealTargetProjector,
    ) -> None:
        self._ideal_360 = bool(ideal_360)
        self._finite = finite
        self._ideal = ideal

    def update(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        navigation_attitude: Optional[Attitude] = None,
    ) -> Optional[DetectedObject]:
        return self.detect(
            camera_location,
            target,
            uas_attitude,
            timestamp_s=timestamp_s,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
            navigation_attitude=navigation_attitude,
        ).target

    def detect(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        navigation_attitude: Optional[Attitude] = None,
    ) -> DetectResult:
        if not self._ideal_360:
            return self._finite.detect(
                camera_location,
                target,
                uas_attitude,
                timestamp_s=timestamp_s,
                uas_body_rates_rad_s=uas_body_rates_rad_s,
            )
        projected = self.project_ideal_target(
            camera_location,
            target,
            uas_attitude,
            timestamp_s=timestamp_s,
            navigation_attitude=navigation_attitude,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
        )
        if projected is None:
            return DetectResult(DetectStatus.OutOfView)
        return DetectResult(DetectStatus.DETECTED, projected)

    def project_target(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
    ) -> Optional[DetectedObject]:
        return self._finite.project(
            camera_location,
            target,
            uas_attitude,
            timestamp_s=timestamp_s,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
        )

    def project_ideal_target(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        navigation_attitude: Optional[Attitude] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
    ) -> Optional[DetectedObject]:
        return self._ideal.project(
            camera_location,
            target,
            uas_attitude,
            timestamp_s=timestamp_s,
            navigation_attitude=navigation_attitude,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
        )

    def diagnose_track_loss(
        self,
        tracking_id: int,
        camera_location: Location,
        uas_attitude: Attitude,
        camera_matrix: np.ndarray,
        gimbal_data: GimbalData,
    ) -> str:
        return self._finite.diagnose(
            tracking_id,
            camera_location,
            uas_attitude,
            camera_matrix,
            gimbal_data,
        )

    @staticmethod
    def get_error_from_center_p(height: float | None, error: float) -> float:
        if height is None or height == 0:
            return float("inf")
        return abs(height / 2 - error) / (height / 2) * 100


__all__ = ["SimTargetProjector", "_copy_gimbal_data"]
