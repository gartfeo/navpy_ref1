"""Infinite-FOV static-camera renderer for ideal simulator vision."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
import pymap3d

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
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
    aircraft_yaw_rate_rad_s,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.ideal_camera_state import IdealCameraReader
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.sim.sim_runtime_ports import TimestampReader
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.visual_ray_projection import (
    body_ray_to_pixel,
    camera_to_body_from_gimbal,
)
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


@dataclass(frozen=True)
class UasFrameConvention:
    sequence: str
    degrees: bool


def ideal_angular_camera_matrix(frame_size: FrameSize) -> np.ndarray:
    """Map each nominal 180-degree angular span across one image canvas."""
    width = float(frame_size.width_px)
    height = float(frame_size.height_px)
    if not all((math.isfinite(width), math.isfinite(height), width > 0, height > 0)):
        raise ValueError("ideal angular canvas must be positive and finite")
    return np.array([
        [width / math.pi, 0.0, width / 2.0],
        [0.0, height / math.pi, height / 2.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def poi_los_uas(
    p_ned: Sequence[float],
    uas_attitude: Attitude,
    gimbal_data: GimbalData,
    convention: UasFrameConvention,
) -> np.ndarray:
    p_ned_vec = np.asarray(p_ned, dtype=float).reshape(3)
    if not np.all(np.isfinite(p_ned_vec)):
        raise ValueError("POI vector must be finite")
    rotation = Rotation.from_euler(
        convention.sequence,
        get_euler_by_sequence(uas_attitude, convention.sequence),
        degrees=convention.degrees,
    ).as_matrix()
    camera_origin = np.asarray(gimbal_data.setup_dist, dtype=float).reshape(3)
    los_uas = np.transpose(rotation) @ p_ned_vec - camera_origin
    norm = float(np.linalg.norm(los_uas))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("POI LOS must be non-zero")
    result = los_uas / norm
    result.setflags(write=False)
    return result


class IdealPoiProjector:
    """Render POI truth into a static aircraft-relative visual ray."""

    def __init__(
        self,
        camera: IdealCameraReader,
        frame_size: FrameSize,
        convention: UasFrameConvention,
        timestamp_now: TimestampReader,
    ) -> None:
        self._camera = camera
        self._frame_size = frame_size
        self._convention = convention
        self._timestamp_now = timestamp_now

    def project(
        self,
        camera_location: Location,
        poi: SimulationObject,
        render_attitude: Attitude,
        *,
        timestamp_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        navigation_attitude: Optional[Attitude] = None,
    ) -> Optional[DetectedObject]:
        if navigation_attitude is None:
            return None
        try:
            camera_matrix = ideal_angular_camera_matrix(self._frame_size)
            calibration = PixelCalibration.from_matrix(camera_matrix)
            gimbal_data = self._camera.read()
            camera_to_body = camera_to_body_from_gimbal(gimbal_data)
            p_ned = pymap3d.geodetic2ned(
                poi.g_loc.lat,
                poi.g_loc.lng,
                poi.g_loc.alt,
                camera_location.lat,
                camera_location.lng,
                camera_location.alt,
            )
            vision_los_uas = poi_los_uas(
                p_ned,
                render_attitude,
                gimbal_data,
                self._convention,
            )
            x_error, y_error = body_ray_to_pixel(
                vision_los_uas,
                calibration,
                camera_to_body,
                PixelProjectionKind.SPHERICAL_EQUIANGULAR,
            )
        except ValueError:
            return None
        timestamp = self._timestamp_now() if timestamp_s is None else timestamp_s
        yaw_free_attitude = Attitude(
            pitch=float(navigation_attitude.pitch),
            yaw=0.0,
            roll=float(navigation_attitude.roll),
        )
        return DetectedObject(
            identity=DetectionIdentity(poi.uid, poi.uid),
            classification=DetectionClassification(
                DetectionSizeClass.L,
                class_id=0,
                confidence=1.0,
                location_type=poi.location_type,
            ),
            pixel=PixelObservation(
                u_px=float(x_error),
                v_px=float(y_error),
                calibration=calibration,
                camera_to_body=camera_to_body,
                projection=PixelProjectionKind.SPHERICAL_EQUIANGULAR,
                aircraft_pitch_deg=float(yaw_free_attitude.pitch),
                aircraft_roll_deg=float(yaw_free_attitude.roll),
                source_timestamp_s=float(timestamp),
                pose_is_frame_atomic=True,
                source_name=str(gimbal_data.name),
                aircraft_yaw_rate_rad_s=aircraft_yaw_rate_rad_s(
                    float(yaw_free_attitude.pitch),
                    float(yaw_free_attitude.roll),
                    uas_body_rates_rad_s,
                ),
            ),
            tracking=TrackingEvidence(),
            confirmation=ConfirmationEvidence(supports_frame=False),
            pose=PoseProvenance(
                aircraft_attitude=yaw_free_attitude,
                gimbal_data=gimbal_data,
                pose_timestamp_s=timestamp,
                vehicle_attitude_timestamp_s=timestamp,
                gimbal_attitude_timestamp_s=timestamp,
                pose_age_s=0.0,
                is_frame_atomic=True,
                status="sim_ideal_360_static",
            ),
            optics=OpticsProvenance(
                calibration,
                frame_width_px=float(self._frame_size.width_px),
                frame_height_px=float(self._frame_size.height_px),
            ),
            timing=SourceTiming(
                detection_timestamp_s=timestamp,
                camera_frame_timestamp_s=timestamp,
                tracker_timestamp_s=timestamp,
                detection_now_s=self._timestamp_now,
            ),
            geo=DetectionGeoDiagnostics(
                reference_height_m=None,
                camera_location=camera_location,
                truth_poi_location=poi.g_loc,
                is_simulation=True,
            ),
        )


__all__ = [
    "IdealPoiProjector",
    "UasFrameConvention",
    "ideal_angular_camera_matrix",
    "poi_los_uas",
]
