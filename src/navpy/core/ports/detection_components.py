"""Cohesive immutable evidence groups for one detected POI."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, TYPE_CHECKING

import numpy as np

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.board_pose import BoardPoseResult
from navpy.core.ports.pixel_observation import PixelCalibration
from navpy.core.ports.gimbal_abc import GimbalData

if TYPE_CHECKING:
    from navpy.core.ports.detect_data import DetectionSizeClass


BoundingBox = tuple[float, float, float, float]


@dataclass(frozen=True)
class DetectionIdentity:
    obj_id: int
    task_id: int
    #: Which instance of its source detector made it: a restarted detector is a new generation whose local
    #: ``obj_id`` values start again (docs/design/navpy_recovery.md, step 5b). 0 for the first instance.
    source_generation: int = 0


@dataclass(frozen=True)
class DetectionClassification:
    size_class: DetectionSizeClass
    class_id: int = 0
    confidence: float = 1.0
    location_type: str | None = None


@dataclass(frozen=True)
class TrackingEvidence:
    bbox_cxcywh: BoundingBox | None = None
    x_velocity_px_s: float | None = None
    y_velocity_px_s: float | None = None


@dataclass(frozen=True)
class ConfirmationEvidence:
    frame: Any = None
    bbox_cxcywh: BoundingBox | None = None
    frame_bboxes: tuple[BoundingBox, ...] | None = None
    supports_frame: bool = True
    degraded: bool = False

    @classmethod
    def capture(
        cls,
        frame: Any,
        bbox_cxcywh: BoundingBox | None,
        frame_bboxes: Sequence[BoundingBox] | None,
        *,
        supports_frame: bool = True,
        degraded: bool = False,
    ) -> "ConfirmationEvidence":
        return cls(
            frame=frame,
            bbox_cxcywh=bbox_cxcywh,
            frame_bboxes=None if frame_bboxes is None else tuple(frame_bboxes),
            supports_frame=supports_frame,
            degraded=degraded,
        )


@dataclass(frozen=True)
class PoseProvenance:
    aircraft_attitude: Attitude | None
    gimbal_data: GimbalData | None
    pose_timestamp_s: float | None = None
    vehicle_attitude_timestamp_s: float | None = None
    gimbal_attitude_timestamp_s: float | None = None
    pose_age_s: float | None = None
    is_frame_atomic: bool | None = None
    status: str = "unknown"
    body_rates_rad_s: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class OpticsProvenance:
    calibration: PixelCalibration | None
    frame_width_px: float | None = None
    frame_height_px: float | None = None
    camera_frame_sequence: int | None = None
    sample_id: str | None = None
    zoom_command: str | None = None

    def camera_matrix(self) -> np.ndarray | None:
        return None if self.calibration is None else self.calibration.matrix()


@dataclass(frozen=True)
class SourceTiming:
    detection_timestamp_s: float | None = None
    camera_frame_timestamp_s: float | None = None
    tracker_timestamp_s: float | None = None
    source_receipt_timestamp_s: float | None = None
    detection_now_s: Callable[[], float] | None = None
    source_receipt_now_s: Callable[[], float] | None = None
    source_air_speed_mps: float | None = None
    # Visual rates difference this when set (encoded PTS, HITL step 11).
    interval_timestamp_s: float | None = None


@dataclass(frozen=True)
class DetectionGeoDiagnostics:
    reference_height_m: float | None = None
    camera_location: Location | None = None
    projected_poi_location: Location | None = None
    truth_poi_location: Location | None = None
    is_simulation: bool = False
    # Acquisition only (HITL step 22): the board pose from this frame's
    # corners and the camera location at its capture instant. Neither may
    # reach the final-approach command record.
    board_pose: BoardPoseResult | None = None
    capture_camera_location: Location | None = None


__all__ = [
    "BoundingBox",
    "ConfirmationEvidence",
    "DetectionClassification",
    "DetectionGeoDiagnostics",
    "DetectionIdentity",
    "OpticsProvenance",
    "PoseProvenance",
    "SourceTiming",
    "TrackingEvidence",
]
