"""Public grouped detection model for visual observations.

A selected observation is the POI (detector class ``dock``). DetectionIdentity
and task IDs provide correlation only. Serialized keys stay compatible with
existing telemetry and evaluation consumers.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum, unique

from navpy.modules.common.models.location import Location
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
    PixelObservation,
    VisualDetection,
)


class DetectStatus(Enum):
    DETECTED = 0
    OutOfView = 2


@unique
class DetectionSizeClass(Enum):
    S = 1
    M = 2
    L = 3
    XL = 4


class _WeakReferenceable:
    __slots__ = ("__weakref__",)


@dataclass(eq=False)
class DetectedObject(_WeakReferenceable):
    __slots__ = (
        "identity",
        "classification",
        "pixel",
        "tracking",
        "confirmation",
        "pose",
        "optics",
        "timing",
        "geo",
    )

    identity: DetectionIdentity
    classification: DetectionClassification
    pixel: PixelObservation
    tracking: TrackingEvidence
    confirmation: ConfirmationEvidence
    pose: PoseProvenance
    optics: OpticsProvenance
    timing: SourceTiming
    geo: DetectionGeoDiagnostics

    def visual_detection(self) -> VisualDetection:
        return VisualDetection(
            task_id=self.identity.task_id,
            obj_id=self.identity.obj_id,
            observation=self.pixel,
        )

    def reidentify(self, *, obj_id: int | None = None, task_id: int | None = None) -> None:
        self.identity = replace(
            self.identity,
            obj_id=self.identity.obj_id if obj_id is None else obj_id,
            task_id=self.identity.task_id if task_id is None else task_id,
        )

    def replace_classification(self, value: DetectionClassification) -> None:
        self.classification = value

    def replace_pixel(self, value: PixelObservation) -> None:
        self.pixel = value

    def replace_tracking(self, value: TrackingEvidence) -> None:
        self.tracking = value

    def capture_confirmation(self, value: ConfirmationEvidence) -> None:
        self.confirmation = value

    def set_confirmation_degraded(self, degraded: bool) -> None:
        self.confirmation = replace(self.confirmation, degraded=bool(degraded))

    def record_receipt(
        self,
        timestamp_s: float | None,
        now_s: Callable[[], float] | None,
        air_speed_mps: float | None,
    ) -> None:
        self.timing = replace(
            self.timing,
            source_receipt_timestamp_s=timestamp_s,
            source_receipt_now_s=now_s,
            source_air_speed_mps=air_speed_mps,
        )

    def replace_timing(self, value: SourceTiming) -> None:
        self.timing = value

    def replace_pose(self, value: PoseProvenance) -> None:
        self.pose = value

    def replace_geo(self, value: DetectionGeoDiagnostics) -> None:
        self.geo = value

    def set_p_t_g_loc(self, location: Location | None) -> None:
        self.geo = replace(self.geo, projected_poi_location=location)

    def to_dict(self) -> dict:
        location = self.geo.projected_poi_location
        return {
            "obj_id": self.identity.obj_id,
            "task_id": self.identity.task_id,
            "x_error": self.pixel.u_px,
            "y_error": self.pixel.v_px,
            "reference_height_m": self.geo.reference_height_m,
            "p_t_g_l": {
                "lat": location.lat if location else None,
                "lng": location.lng if location else None,
                "alt": location.alt if location else None,
            },
        }

    def __str__(self) -> str:
        return (
            f"{self.identity.obj_id} - x_error: {self.pixel.u_px}, "
            f"y_error: {self.pixel.v_px}, g_data: {self.pose.gimbal_data}, "
        )


@dataclass
class DetectResult:
    status: DetectStatus
    poi: DetectedObject | None = None


__all__ = [
    "DetectedObject",
    "DetectResult",
    "DetectStatus",
    "DetectionSizeClass",
]
