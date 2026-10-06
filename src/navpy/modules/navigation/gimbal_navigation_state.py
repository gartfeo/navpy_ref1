"""Explicit resources, state owners, and snapshots for gimbal navigation."""

from __future__ import annotations

import threading
import math
from dataclasses import dataclass, field
from typing import Any, Optional, TYPE_CHECKING

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.gimbal_hardware_ports import (
    GimbalActuatorPort,
    GimbalMountPort,
)
from navpy.modules.vision.gimbal_rate_tracker import (
    GimbalRateTracker,
    GimbalRateTrackerConfig,
)
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_zoom_orchestrator import TargetZoomTracker
from navpy.modules.vision.target_zoom_types import (
    TargetZoomTrackerConfig,
    ZoomTrackResult,
)

if TYPE_CHECKING:
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc


@dataclass(frozen=True)
class GimbalHardware:
    mount: GimbalMountPort
    gimbal: GimbalActuatorPort
    logger: ILogger


@dataclass(frozen=True)
class GimbalTrackers:
    rate: Optional[GimbalRateTracker]
    zoom: Optional[TargetZoomTracker]
    zoom_config: Optional[TargetZoomTrackerConfig]


@dataclass(frozen=True)
class GimbalLossPolicy:
    hold_sec: float = 0.5
    repoint_sec: float = 2.0
    preserve_zoom_during_loss: bool = True

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.hold_sec)
            or not math.isfinite(self.repoint_sec)
            or not 0.0 <= self.hold_sec < self.repoint_sec
        ):
            raise ValueError(
                "loss policy must satisfy 0 <= hold_sec < repoint_sec"
            )


@dataclass(frozen=True)
class GimbalTrackingSetup:
    rate: GimbalRateTrackerConfig
    loss: GimbalLossPolicy = field(default_factory=GimbalLossPolicy)


@dataclass
class GimbalSessionFence:
    lock: Any = field(default_factory=threading.Lock)
    generation: int = 0


@dataclass
class GimbalSessionCommandGate:
    """Serialize command transactions without holding the state fence."""

    lock: Any = field(default_factory=threading.RLock)


@dataclass
class GimbalDetectionMemory:
    tracking_obj_id: Optional[int] = None
    last_track_time: Optional[float] = None
    recentered: bool = False
    holding: bool = False
    last_tracked_for_zoom: Optional[DetectedObject] = None
    terminal_zoom_frozen_at_min: bool = False


@dataclass
class GimbalGeoMemory:
    target: Location | None = None
    geo_ref: GeoRefCalc | None = None
    zoom_key: Optional[str] = None
    last_ray_log_key: Optional[tuple] = None

    def clear(self) -> None:
        self.target = None
        self.geo_ref = None
        self.zoom_key = None
        self.last_ray_log_key = None


@dataclass(frozen=True)
class GimbalDetectionSnapshot:
    tracking_obj_id: Optional[int]
    last_track_time: Optional[float]
    recentered: bool
    holding: bool
    last_tracked_for_zoom: Optional[DetectedObject]
    terminal_zoom_frozen_at_min: bool


@dataclass(frozen=True)
class GimbalGeoSnapshot:
    target: Location | None
    geo_ref: GeoRefCalc | None
    zoom_key: Optional[str]
    last_ray_log_key: Optional[tuple]


class GimbalNavigationStatus:
    """Read-only status and diagnostics outside the visual command path."""

    def __init__(
        self,
        trackers: GimbalTrackers,
        fence: GimbalSessionFence,
        detection: GimbalDetectionMemory,
        geo: GimbalGeoMemory,
        loss_policy: GimbalLossPolicy,
    ) -> None:
        self._trackers = trackers
        self._fence = fence
        self._detection = detection
        self._geo = geo
        self._loss_policy = loss_policy

    @property
    def generation(self) -> int:
        with self._fence.lock:
            return self._fence.generation

    @property
    def detection(self) -> GimbalDetectionSnapshot:
        with self._fence.lock:
            state = self._detection
            return GimbalDetectionSnapshot(
                state.tracking_obj_id,
                state.last_track_time,
                state.recentered,
                state.holding,
                state.last_tracked_for_zoom,
                state.terminal_zoom_frozen_at_min,
            )

    @property
    def geo(self) -> GimbalGeoSnapshot:
        with self._fence.lock:
            state = self._geo
            return GimbalGeoSnapshot(
                state.target,
                state.geo_ref,
                state.zoom_key,
                state.last_ray_log_key,
            )

    @property
    def rate_result(self) -> GimbalTrackResult | None:
        tracker = self._trackers.rate
        return None if tracker is None else tracker.last_result

    @property
    def zoom_result(self) -> ZoomTrackResult | None:
        tracker = self._trackers.zoom
        return None if tracker is None else tracker.last_result

    @property
    def zoom_is_stable(self) -> bool:
        tracker = self._trackers.zoom
        return True if tracker is None else tracker.is_zoom_stable

    def zoom_target_pixels(self, class_id: object) -> float | None:
        tracker = self._trackers.zoom
        return (
            None
            if tracker is None
            else tracker.get_target_pixels_for_class(class_id)
        )

    @property
    def loss_hold_sec(self) -> float:
        return self._loss_policy.hold_sec


__all__ = [
    "GimbalDetectionMemory",
    "GimbalDetectionSnapshot",
    "GimbalGeoMemory",
    "GimbalGeoSnapshot",
    "GimbalNavigationStatus",
    "GimbalHardware",
    "GimbalLossPolicy",
    "GimbalSessionFence",
    "GimbalSessionCommandGate",
    "GimbalTrackers",
    "GimbalTrackingSetup",
]
