"""Grouped DetectedObject fixtures used across tests."""

from __future__ import annotations

from typing import Any

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
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
    CameraToBodyTransform,
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
    aircraft_yaw_rate_rad_s,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.visual_ray_projection import camera_to_body_from_gimbal


_DEFAULT_MATRIX = np.asarray(
    [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
    dtype=float,
)


def make_detected_poi(
    *,
    obj_id: int = 1,
    task_id: int | None = None,
    x_error: float = 320.0,
    y_error: float = 240.0,
    reference_height_m: float | None = 2.0,
    k: Any = _DEFAULT_MATRIX,
    g_data: GimbalData | None = None,
    uas_att: Attitude | None = None,
    c_g_loc: Location | None = None,
    size_class: DetectionSizeClass = DetectionSizeClass.L,
    class_id: int = 0,
    confidence: float = 1.0,
    t_g_loc_debug: Location | None = None,
    detection_frame: Any = None,
    bbox_cxcywh=None,
    tracking_bbox_cxcywh=None,
    frame_bboxes=None,
    x_velocity: float | None = None,
    y_velocity: float | None = None,
    timestamp: float | None = None,
    camera_frame_timestamp_s: float | None = None,
    tracker_timestamp_s: float | None = None,
    pose_timestamp_s: float | None = None,
    vehicle_attitude_timestamp_s: float | None = None,
    gimbal_attitude_timestamp_s: float | None = None,
    pose_age_s: float | None = None,
    pose_is_frame_atomic: bool | None = None,
    pose_status: str = "unknown",
    frame_width_px: float | None = None,
    frame_height_px: float | None = None,
    camera_frame_sequence: int | None = None,
    camera_optics_sample_id: str | None = None,
    camera_zoom_command: str | None = None,
    supports_confirmation_frame: bool = True,
    timestamp_now_s=None,
    source_receipt_timestamp_s: float | None = None,
    source_receipt_now_s=None,
    source_air_speed_mps: float | None = None,
    uas_body_rates_rad_s=None,
    location_type: str | None = None,
    is_simulation: bool = False,
    confirmation_degraded: bool = False,
    p_t_g_l: Location | None = None,
    projection: PixelProjectionKind = PixelProjectionKind.PINHOLE,
) -> DetectedObject:
    gimbal = g_data or GimbalData(att=Attitude(0.0, 0.0, 0.0), name="test")
    attitude = uas_att or Attitude(0.0, 0.0, 0.0)
    try:
        calibration = PixelCalibration.from_matrix(k)
        has_calibration = True
    except (TypeError, ValueError):
        calibration = PixelCalibration.from_matrix(_DEFAULT_MATRIX)
        has_calibration = False
    try:
        camera_to_body = camera_to_body_from_gimbal(gimbal)
    except (AttributeError, TypeError, ValueError):
        camera_to_body = CameraToBodyTransform.from_matrix(np.eye(3))
    pitch_deg = getattr(attitude, "pitch", 0.0)
    roll_deg = getattr(attitude, "roll", 0.0)
    if not isinstance(pitch_deg, (int, float)):
        pitch_deg = 0.0
    if not isinstance(roll_deg, (int, float)):
        roll_deg = 0.0
    return DetectedObject(
        identity=DetectionIdentity(obj_id, obj_id if task_id is None else task_id),
        classification=DetectionClassification(
            size_class,
            class_id,
            confidence,
            location_type,
        ),
        pixel=PixelObservation(
            x_error,
            y_error,
            calibration,
            camera_to_body,
            projection,
            pitch_deg,
            roll_deg,
            timestamp,
            pose_is_frame_atomic,
            gimbal.name if g_data is not None else "test",
            aircraft_yaw_rate_rad_s(
                float(pitch_deg),
                float(roll_deg),
                uas_body_rates_rad_s,
            ) if uas_body_rates_rad_s is not None else 0.0,
        ),
        tracking=TrackingEvidence(
            tracking_bbox_cxcywh,
            x_velocity,
            y_velocity,
        ),
        confirmation=ConfirmationEvidence.capture(
            detection_frame,
            bbox_cxcywh,
            frame_bboxes,
            supports_frame=supports_confirmation_frame,
            degraded=confirmation_degraded,
        ),
        pose=PoseProvenance(
            uas_att,
            g_data,
            pose_timestamp_s,
            vehicle_attitude_timestamp_s,
            gimbal_attitude_timestamp_s,
            pose_age_s,
            pose_is_frame_atomic,
            pose_status,
            uas_body_rates_rad_s,
        ),
        optics=OpticsProvenance(
            calibration if has_calibration else None,
            frame_width_px,
            frame_height_px,
            camera_frame_sequence,
            camera_optics_sample_id,
            camera_zoom_command,
        ),
        timing=SourceTiming(
            timestamp,
            camera_frame_timestamp_s,
            tracker_timestamp_s,
            source_receipt_timestamp_s,
            timestamp_now_s,
            source_receipt_now_s,
            source_air_speed_mps,
        ),
        geo=DetectionGeoDiagnostics(
            reference_height_m,
            c_g_loc,
            p_t_g_l,
            t_g_loc_debug,
            is_simulation,
        ),
    )


__all__ = ["make_detected_poi"]
