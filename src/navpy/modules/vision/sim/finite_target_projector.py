"""Finite-FOV simulator target projection and track-loss diagnostics."""

from __future__ import annotations

from dataclasses import replace
from typing import Optional, Tuple

import numpy as np
import pymap3d

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import (
    DetectedObject,
    DetectResult,
    DetectStatus,
    DetectionSizeClass,
)
from navpy.modules.vision.models.detection_components import (
    ConfirmationEvidence,
    DetectionClassification,
    DetectionGeoDiagnostics,
    DetectionIdentity,
    OpticsProvenance,
    PoseProvenance,
    SourceTiming,
    TrackingEvidence,
)
from navpy.modules.vision.models.pixel_observation import (
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
    aircraft_yaw_rate_rad_s,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.finite_track_loss import FiniteTrackLossDiagnoser
from navpy.modules.vision.sim.finite_projection_evidence import EvidenceErrorSink, EvidenceSink, ProjectionCapture
from navpy.modules.vision.sim.finite_projection_observer import FiniteProjectionObserver
from navpy.modules.vision.sim.finite_projection_geometry import tracking_bbox
from navpy.modules.vision.sim.sim_camera_ports import ProjectionCameraPort
from navpy.modules.vision.sim.sim_runtime_ports import (
    PixelCalculator,
    TargetSnapshotReader,
    TimestampReader,
)
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.visual_ray_projection import camera_to_body_from_gimbal
from navpy.modules.vision.vision_class_profile import (
    MIN_DETECT_PIXELS,
    get_class_detect_size,
)
from navpy.utils.euler_utils import get_att_by_sequence, get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


class FiniteTargetProjector:
    """Project normal simulator detections through exact camera operations."""

    def __init__(
        self,
        camera: ProjectionCameraPort,
        calc_uv: PixelCalculator,
        target_snapshot: TargetSnapshotReader,
        timestamp_now: TimestampReader,
        *,
        evidence_sink: EvidenceSink | None = None,
        evidence_error_sink: EvidenceErrorSink | None = None,
    ) -> None:
        self._camera = camera
        self._calc_uv = calc_uv
        self._timestamp_now = timestamp_now
        self._observer = None if evidence_sink is None else FiniteProjectionObserver(evidence_sink, evidence_error_sink)
        self._diagnoser = FiniteTrackLossDiagnoser(
            camera,
            calc_uv,
            target_snapshot,
        )

    def project(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        _evidence: ProjectionCapture | None = None,
    ) -> Optional[DetectedObject]:
        camera_matrix = self._camera.read_matrix()
        gimbal_data = self._camera.read_gimbal()
        if _evidence is not None:
            _evidence.observe(_evidence.optics, camera_matrix, gimbal_data)
        gimbal_data = _rebase_stabilized_readback(gimbal_data, uas_attitude)
        projection_attitude = uas_attitude
        p_ned = pymap3d.geodetic2ned(
            target.g_loc.lat,
            target.g_loc.lng,
            target.g_loc.alt,
            camera_location.lat,
            camera_location.lng,
            camera_location.alt,
        )
        distance_m = float(np.linalg.norm(p_ned))
        if _evidence is not None:
            _evidence.observe(_evidence.projection_inputs, p_ned, gimbal_data)
        if distance_m <= 0.0:
            if _evidence is not None:
                _evidence.early_outcome = 'zero_distance'
            return None
        focal_y = float(camera_matrix[1, 1])
        characteristic_m = get_class_detect_size(0)
        pixel_ok = (
            focal_y > 0.0
            and focal_y * characteristic_m / distance_m >= MIN_DETECT_PIXELS
        )
        if not pixel_ok and distance_m > gimbal_data.max_detect_distance:
            if _evidence is not None:
                _evidence.early_outcome = 'too_far'
            return None
        x_error, y_error = self._calc_uv(
            np.asarray(p_ned, dtype=float),
            camera_matrix,
            gimbal_data,
            projection_attitude,
        )
        if _evidence is not None:
            _evidence.observe(lambda: dict(pixel_uv=(
                None if x_error is None else float(x_error),
                None if y_error is None else float(y_error),
            )))
        if x_error is None or y_error is None:
            return None
        tracking_bbox = self._tracking_bbox(
            pixel_ok,
            focal_y,
            distance_m,
            x_error,
            y_error,
        )
        timestamp = self._timestamp_now() if timestamp_s is None else timestamp_s
        if _evidence is not None:
            _evidence.observe(lambda: dict(source_timestamp_s=float(timestamp)))
        frame_size = self._camera.frame_size
        calibration = PixelCalibration.from_matrix(camera_matrix)
        return DetectedObject(
            identity=DetectionIdentity(target.uid, target.uid),
            classification=DetectionClassification(
                DetectionSizeClass.L,
                class_id=0,
                confidence=0.95,
                location_type=target.location_type,
            ),
            pixel=PixelObservation(
                u_px=float(x_error),
                v_px=float(y_error),
                calibration=calibration,
                camera_to_body=camera_to_body_from_gimbal(gimbal_data),
                projection=PixelProjectionKind.PINHOLE,
                aircraft_pitch_deg=float(projection_attitude.pitch),
                aircraft_roll_deg=float(projection_attitude.roll),
                source_timestamp_s=float(timestamp),
                pose_is_frame_atomic=True,
                source_name=str(gimbal_data.name),
                aircraft_yaw_rate_rad_s=aircraft_yaw_rate_rad_s(
                    float(projection_attitude.pitch),
                    float(projection_attitude.roll),
                    uas_body_rates_rad_s,
                ),
            ),
            tracking=TrackingEvidence(bbox_cxcywh=tracking_bbox),
            confirmation=ConfirmationEvidence(),
            pose=PoseProvenance(
                aircraft_attitude=projection_attitude,
                gimbal_data=gimbal_data,
                pose_timestamp_s=timestamp,
                vehicle_attitude_timestamp_s=timestamp,
                gimbal_attitude_timestamp_s=timestamp,
                pose_age_s=0.0,
                is_frame_atomic=True,
                status="sim_frame_pose",
                body_rates_rad_s=uas_body_rates_rad_s,
            ),
            optics=OpticsProvenance(
                calibration,
                frame_width_px=float(frame_size.width_px),
                frame_height_px=float(frame_size.height_px),
            ),
            timing=SourceTiming(
                detection_timestamp_s=timestamp,
                camera_frame_timestamp_s=timestamp,
                tracker_timestamp_s=timestamp,
                detection_now_s=self._timestamp_now,
            ),
            geo=DetectionGeoDiagnostics(
                reference_height_m=target.height,
                camera_location=camera_location,
                truth_target_location=target.g_loc,
                is_simulation=True,
            ),
        )

    def detect(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
    ) -> DetectResult:
        if self._observer is not None:
            return self._observer.detect(self.project, self._camera.pixel_valid, self._camera.frame_size,
                                         camera_location, target, uas_attitude, timestamp_s, uas_body_rates_rad_s)
        projected = self.project(
            camera_location,
            target,
            uas_attitude,
            timestamp_s=timestamp_s,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
        )
        if projected is None or not self._camera.pixel_valid(
            float(projected.pixel.u_px),
            float(projected.pixel.v_px),
        ):
            return DetectResult(DetectStatus.OutOfView)
        return DetectResult(DetectStatus.DETECTED, projected)

    @property
    def evidence_failures(self) -> int:
        return 0 if self._observer is None else self._observer.failures

    def diagnose(
        self,
        tracking_id: int,
        camera_location: Location,
        uas_attitude: Attitude,
        camera_matrix: np.ndarray,
        gimbal_data: GimbalData,
    ) -> str:
        return self._diagnoser.diagnose(
            tracking_id,
            camera_location,
            uas_attitude,
            camera_matrix,
            gimbal_data,
        )

    @staticmethod
    def _tracking_bbox(
        pixel_ok: bool, focal_y: float, distance_m: float,
        x_error: float, y_error: float,
    ) -> tuple[float, float, float, float] | None:
        return tracking_bbox(pixel_ok, focal_y, distance_m, x_error, y_error)

def _rebase_stabilized_readback(
    gimbal_data: GimbalData,
    aircraft_attitude: Attitude,
) -> GimbalData:
    """Express a simulator world-lock readback in the current body frame."""
    reference = gimbal_data.reference_aircraft_attitude
    if reference is None:
        return gimbal_data
    reference_to_ned = Rotation.from_euler(
        "ZYX",
        get_euler_by_sequence(reference, "ZYX"),
        degrees=True,
    ).as_matrix()
    current_to_ned = Rotation.from_euler(
        "ZYX",
        get_euler_by_sequence(aircraft_attitude, "ZYX"),
        degrees=True,
    ).as_matrix()
    gimbal_to_reference = Rotation.from_euler(
        gimbal_data.g_seq,
        get_euler_by_sequence(gimbal_data.att, gimbal_data.g_seq),
        degrees=gimbal_data.degrees,
    ).as_matrix()
    gimbal_to_current = current_to_ned.T @ reference_to_ned @ gimbal_to_reference
    euler = Rotation.from_matrix(gimbal_to_current).as_euler(
        gimbal_data.g_seq,
        degrees=gimbal_data.degrees,
    )
    return replace(
        gimbal_data,
        att=get_att_by_sequence(euler, gimbal_data.g_seq),
        reference_aircraft_attitude=aircraft_attitude,
    )


__all__ = ["FiniteTargetProjector", "FiniteTrackLossDiagnoser"]
