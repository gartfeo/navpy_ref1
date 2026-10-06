"""Exact runtime operation ports for simulator detection."""

from __future__ import annotations

from typing import Optional, Protocol, Sequence, Tuple

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.simulation_object import SimulationObject


BoundingBox = tuple[float, float, float, float]
OverlayPosition = tuple[float, float, float]
RenderedFrame = tuple[np.ndarray, list[BoundingBox]]
BestFrame = tuple[np.ndarray, BoundingBox, float, list[BoundingBox]]


class TimestampReader(Protocol):
    def __call__(self) -> float: ...


class OptionalFloatReader(Protocol):
    def __call__(self) -> Optional[float]: ...


class OptionalTextReader(Protocol):
    def __call__(self) -> Optional[str]: ...


class LocationReader(Protocol):
    def __call__(self) -> Location: ...


class AttitudeReader(Protocol):
    def __call__(self) -> Attitude: ...


class AttitudeSampleView(Protocol):
    attitude: Attitude
    time_boot_s: Optional[float]
    receipt_time_s: Optional[float]
    body_rates_rad_s: Optional[Tuple[float, float, float]]


class AttitudeSampleReader(Protocol):
    def __call__(self) -> Optional[AttitudeSampleView]: ...


class AttitudeTimestampAcceptor(Protocol):
    def __call__(
        self,
        value: Optional[float],
        *,
        record_emitted: bool,
    ) -> Optional[float]: ...


class TargetSnapshotReader(Protocol):
    def __call__(self) -> tuple[SimulationObject, ...]: ...


class TargetProjectorPort(Protocol):
    def __call__(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float],
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]],
        navigation_attitude: Optional[Attitude],
    ) -> Optional[DetectedObject]: ...


class SourceNameResolver(Protocol):
    def __call__(
        self,
        targets: Sequence[DetectedObject],
    ) -> Optional[str]: ...


class PixelCalculator(Protocol):
    def __call__(
        self,
        p_ned: np.ndarray,
        camera_matrix: np.ndarray,
        gimbal_data: GimbalData,
        uas_attitude: Attitude,
    ) -> tuple[Optional[float], Optional[float]]: ...


class DebugSink(Protocol):
    def __call__(self, message: str) -> None: ...


class InfoSink(Protocol):
    def __call__(self, message: str) -> None: ...


class ResetAction(Protocol):
    def __call__(self) -> None: ...


class FrameOutcomeSink(Protocol):
    def __call__(
        self,
        boot_s: Optional[float],
        outcome: str,
        frame_timestamp_s: Optional[float],
    ) -> None: ...


class DetectFramePort(Protocol):
    def __call__(
        self,
        camera_location: Location,
        uas_attitude: Attitude,
        attitude_time_boot_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        *,
        frame_timestamp_s: Optional[float] = None,
        frame_receipt_timestamp_s: Optional[float] = None,
        frame_air_speed_mps: Optional[float] = None,
        frame_navigation_attitude: Optional[Attitude] = None,
        frame_generation: Optional[FrameGeneration] = None,
        frame_source_discontinuity: Optional[bool] = None,
    ) -> bool: ...


class ErrorSink(Protocol):
    def __call__(self, message: str, error: Exception | None = None) -> None: ...


class CapacityWarningSink(Protocol):
    def __call__(self, capacity: int) -> None: ...


class TrackLossDiagnoser(Protocol):
    def __call__(
        self,
        tracking_id: int,
        camera_location: Location,
        uas_attitude: Attitude,
        camera_matrix: np.ndarray,
        gimbal_data: GimbalData,
    ) -> str: ...


class TrackingRuntimePort(Protocol):
    @property
    def tracking_obj_id(self) -> Optional[int]: ...

    def apply_detection_update(
        self,
        target: Optional[DetectedObject],
        timestamp_s: float,
    ) -> None: ...


class TrackingStatusPort(Protocol):
    @property
    def tracking_obj_id(self) -> Optional[int]: ...


class GapSpecificationReader(Protocol):
    def __call__(self) -> Optional[str]: ...


class ConfirmationFrameRenderer(Protocol):
    @property
    def is_available(self) -> bool: ...

    def sprite_height_for(self, location_type: Optional[str]) -> int: ...

    def sprite_width_for(self, location_type: Optional[str]) -> int: ...

    def generate_frame(
        self,
        detections: list[OverlayPosition],
        location_type: Optional[str] = None,
    ) -> Optional[RenderedFrame]: ...


class ConfirmationFrameFactory(Protocol):
    def __call__(
        self,
        assets_dir: str,
        frame_size: tuple[int, int],
    ) -> ConfirmationFrameRenderer: ...


__all__ = [
    "AttitudeTimestampAcceptor",
    "AttitudeReader",
    "AttitudeSampleReader",
    "AttitudeSampleView",
    "BestFrame",
    "BoundingBox",
    "CapacityWarningSink",
    "ConfirmationFrameFactory",
    "ConfirmationFrameRenderer",
    "DebugSink",
    "DetectFramePort",
    "ErrorSink",
    "GapSpecificationReader",
    "FrameOutcomeSink",
    "InfoSink",
    "OverlayPosition",
    "LocationReader",
    "OptionalFloatReader",
    "OptionalTextReader",
    "PixelCalculator",
    "RenderedFrame",
    "ResetAction",
    "SourceNameResolver",
    "TargetProjectorPort",
    "TargetSnapshotReader",
    "TimestampReader",
    "TrackLossDiagnoser",
    "TrackingRuntimePort",
    "TrackingStatusPort",
]
