"""Construction of the public detected-target evidence groups."""

from __future__ import annotations

import time

from navpy.modules.vision.camera_mount_types import CameraMountFrameState
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
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
    aircraft_yaw_rate_rad_s,
)
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_confirmation_frame_selector import (
    SelectedConfirmationFrame,
)
from navpy.modules.vision.real_frame_association import RealFrameAssociation
from navpy.modules.vision.real_track_observation import MappedTrackObservation
from navpy.modules.vision.visual_ray_projection import camera_to_body_from_gimbal


class DetectedObjectBuilder:
    """Assemble immutable frame evidence into the public target DTO."""

    def __init__(self, reference_height_m: float) -> None:
        self._reference_height_m = reference_height_m

    def build(
            self,
            track: TrackedObject,
            observation: MappedTrackObservation,
            confirmation: SelectedConfirmationFrame,
            association: RealFrameAssociation,
            mount_state: CameraMountFrameState,
            visual_attitude: tuple[float, float],
    ) -> DetectedObject:
        calibration = PixelCalibration.from_matrix(mount_state.k)
        identity = DetectionIdentity(track.id, track.id)
        classification = DetectionClassification(
            DetectionSizeClass.L,
            class_id=track.class_id,
            confidence=track.confidence,
        )
        pixel = _pixel_observation(
            observation,
            calibration,
            association,
            mount_state,
            visual_attitude,
        )
        tracking = TrackingEvidence(
            bbox_cxcywh=observation.bbox_cxcywh,
            x_velocity_px_s=track.vx,
            y_velocity_px_s=track.vy,
        )
        confirmation_evidence = ConfirmationEvidence.capture(
            confirmation.frame,
            confirmation.bbox_cxcywh,
            None,
        )
        pose = _pose_provenance(association, mount_state)
        optics = _optics_provenance(calibration, association, mount_state)
        timing = _source_timing(track, association)
        geo = DetectionGeoDiagnostics(
            reference_height_m=self._reference_height_m,
            camera_location=association.c_g_loc,
        )
        return DetectedObject(
            identity=identity,
            classification=classification,
            pixel=pixel,
            tracking=tracking,
            confirmation=confirmation_evidence,
            pose=pose,
            optics=optics,
            timing=timing,
            geo=geo,
        )


def visual_attitude_components(
        association: RealFrameAssociation,
) -> tuple[float, float]:
    """Keep a rejected observation representable without reading live pose."""
    if association.uas_att is None:
        return 0.0, 0.0
    return float(association.uas_att.pitch), float(association.uas_att.roll)


def _measurement_timestamp_s(
        association: RealFrameAssociation,
) -> float | None:
    """GEOMETRY/measurement time: the capture estimate when one exists.

    Decision doc §3d: the capture estimate feeds everything the law
    differentiates and the measurement-age policy; the PUBLICATION time
    (`association.frame_timestamp_s`) stays the receipt-liveness stamp and
    is never substituted here. A frame without capture timing keeps the
    publication stamp for representability -- it is non-atomic regardless.
    """
    if association.capture is not None:
        return association.capture.estimate_s
    return association.frame_timestamp_s


def _pixel_observation(
        observation: MappedTrackObservation,
        calibration: PixelCalibration,
        association: RealFrameAssociation,
        mount_state: CameraMountFrameState,
        visual_attitude: tuple[float, float],
) -> PixelObservation:
    aircraft_pitch_deg, aircraft_roll_deg = visual_attitude
    gimbal_data = mount_state.gimbal_data
    return PixelObservation(
        u_px=float(observation.u_px),
        v_px=float(observation.v_px),
        calibration=calibration,
        camera_to_body=camera_to_body_from_gimbal(gimbal_data),
        projection=PixelProjectionKind.PINHOLE,
        aircraft_pitch_deg=aircraft_pitch_deg,
        aircraft_roll_deg=aircraft_roll_deg,
        source_timestamp_s=_measurement_timestamp_s(association),
        pose_is_frame_atomic=association.pose_is_frame_atomic,
        source_name=str(gimbal_data.name),
        aircraft_yaw_rate_rad_s=aircraft_yaw_rate_rad_s(
            aircraft_pitch_deg,
            aircraft_roll_deg,
            association.uas_body_rates_rad_s,
        ),
    )


def _pose_provenance(
        association: RealFrameAssociation,
        mount_state: CameraMountFrameState,
) -> PoseProvenance:
    return PoseProvenance(
        aircraft_attitude=association.uas_att,
        gimbal_data=mount_state.gimbal_data,
        pose_timestamp_s=association.attitude_receipt_time_s,
        vehicle_attitude_timestamp_s=association.attitude_receipt_time_s,
        gimbal_attitude_timestamp_s=(
            association.frame_timestamp_s
            if mount_state.gimbal_is_static
            else mount_state.gimbal_timestamp_s
        ),
        pose_age_s=association.pose_age_s,
        is_frame_atomic=association.pose_is_frame_atomic,
        status=association.pose_status,
        body_rates_rad_s=association.uas_body_rates_rad_s,
    )


def _optics_provenance(
        calibration: PixelCalibration,
        association: RealFrameAssociation,
        mount_state: CameraMountFrameState,
) -> OpticsProvenance:
    return OpticsProvenance(
        calibration,
        frame_width_px=float(association.frame_width),
        frame_height_px=float(association.frame_height),
        camera_frame_sequence=association.frame_sequence,
        sample_id=mount_state.zoom_sample_id,
        zoom_command=mount_state.zoom_command,
    )


def _source_timing(
        track: TrackedObject,
        association: RealFrameAssociation,
) -> SourceTiming:
    # Three roles, three values (decision doc §3d): measurement age judges
    # the CAPTURE estimate; receipt liveness keeps the PUBLICATION stamp.
    return SourceTiming(
        detection_timestamp_s=_measurement_timestamp_s(association),
        camera_frame_timestamp_s=_measurement_timestamp_s(association),
        tracker_timestamp_s=track.timestamp,
        source_receipt_timestamp_s=association.frame_timestamp_s,
        detection_now_s=time.time,
        source_receipt_now_s=time.time,
        source_air_speed_mps=association.air_speed_mps,
    )


__all__ = ["DetectedObjectBuilder", "visual_attitude_components"]
