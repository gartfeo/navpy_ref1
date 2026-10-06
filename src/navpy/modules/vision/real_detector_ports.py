"""Narrow callable and data ports shared by real-detector leaves."""

from __future__ import annotations

from typing import Protocol

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.camera_mount import CameraMountFrameState
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class TimestampedVisionItem(Protocol):
    timestamp: float | None


class AircraftAttitudeReader(Protocol):
    def __call__(self) -> Attitude | None: ...


class MonotonicClock(Protocol):
    def __call__(self) -> float: ...


class SleepAction(Protocol):
    def __call__(self, seconds: float) -> None: ...


class PoseRateResolver(Protocol):
    def __call__(self) -> float: ...


class PoseStreamRequest(Protocol):
    def __call__(self, rate_hz: float) -> None: ...


class CleanupAction(Protocol):
    def __call__(self) -> None: ...


class LifecycleLog(Protocol):
    def __call__(self, message: str) -> None: ...


class WorkerLoop(Protocol):
    def run(self) -> None: ...


class ModelResourceCloser(Protocol):
    def close(self) -> None: ...


class DetectorOverlayRenderer(Protocol):
    def render(
        self,
        frame: np.ndarray,
        tracks: list[TrackedObject],
        locked: TrackedObject | None,
        *,
        fps_est: float,
        use_lock: bool,
    ) -> np.ndarray: ...


class GimbalTrackingCommandPort(Protocol):
    @property
    def tracking_obj_id(self) -> int | None: ...

    @property
    def rate_result(self) -> GimbalTrackResult | None: ...

    @property
    def is_zoom_stable(self) -> bool: ...

    @property
    def zoom_result(self) -> ZoomTrackResult | None: ...

    def zoom_target_pixels(self, class_id: object) -> float | None: ...

    def set_zoom_size_demand(self, enabled: bool) -> None: ...

    def freeze_terminal_zoom_at_min(self) -> bool: ...

    def start_tracking(self, obj_id: int) -> None: ...

    def stop_tracking(self, to_neutral: bool = True) -> None: ...


class GimbalGeoCommandPort(Protocol):
    def start_geo_tracking(
        self,
        target_loc: Location,
        geo_ref: GeoRefCalc,
    ) -> None: ...

    def update_geo(self, uav_loc: Location, uav_att: Attitude) -> None: ...

    def prepare_geo_acquisition(
        self,
        uav_loc: Location,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool: ...

    def stop_geo_tracking(self) -> None: ...

    @property
    def is_geo_armed(self) -> bool: ...

    @property
    def is_detection_armed(self) -> bool: ...

    @property
    def loss_hold_sec(self) -> float | None: ...


class GimbalMeasurementPort(Protocol):
    @property
    def tracking_obj_id(self) -> int | None: ...

    def update(self, target: DetectedObject | None) -> None: ...


class AttitudeSampleView(Protocol):
    attitude: Attitude
    body_rates_rad_s: tuple[float, float, float] | None
    time_boot_s: float | None
    receipt_time_s: float | None


class AttitudeSampleReader(Protocol):
    def __call__(self) -> AttitudeSampleView | None: ...


class AttitudeHistoryReader(Protocol):
    """Newest-last retained ATTITUDE samples for capture-time interpolation."""

    def __call__(self, count: int) -> tuple[AttitudeSampleView, ...]: ...


class LinkIdentityReader(Protocol):
    """Identity of the LIVE MAVLink link, produced by the open connection."""

    def __call__(self) -> str | None: ...


class MountFrameStateReader(Protocol):
    def __call__(
        self,
        frame_width: int,
        frame_height: int,
    ) -> CameraMountFrameState | None: ...


class LocationReader(Protocol):
    def __call__(self) -> Location | None: ...


class AirSpeedReader(Protocol):
    def __call__(self) -> float | None: ...


__all__ = [
    "AirSpeedReader",
    "AircraftAttitudeReader",
    "AttitudeSampleReader",
    "AttitudeSampleView",
    "CleanupAction",
    "DetectorOverlayRenderer",
    "GimbalGeoCommandPort",
    "GimbalMeasurementPort",
    "GimbalTrackingCommandPort",
    "LocationReader",
    "LifecycleLog",
    "ModelResourceCloser",
    "MonotonicClock",
    "MountFrameStateReader",
    "PoseRateResolver",
    "PoseStreamRequest",
    "SleepAction",
    "TimestampedVisionItem",
    "WorkerLoop",
]
