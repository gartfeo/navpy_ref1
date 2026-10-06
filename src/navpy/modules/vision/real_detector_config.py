"""Typed configuration and public ports for one real detector."""

from __future__ import annotations

from dataclasses import dataclass, field

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.capture_lookup import CaptureLookupCalibration
from navpy.modules.vision.device import DeviceT
from navpy.modules.vision.detector_ports import (
    DetectionEventPort,
    SchedulerCadence,
    SimulationControlPort,
    SourceIdentity,
)
from navpy.modules.vision.real_detector_controls import (
    DetectionQuery,
    DetectorGeoControl,
    DetectorResetController,
    DetectorTrackingControl,
)
from navpy.modules.vision.real_detector_diagnostics import (
    DetectorDebugConfig,
    DetectorDiagnostics,
)
from navpy.modules.vision.real_detector_lifecycle import DetectorLifecycle
from navpy.modules.vision.target_zoom_types import TargetZoomTrackerConfig
from navpy.modules.vision.vision_profile_types import VisionProfile


@dataclass(frozen=True)
class DetectorDependencies:
    vehicle: IVehicle
    mount: CameraMount
    logger: ILogger


@dataclass(frozen=True)
class DetectorModelConfig:
    model_path: str
    imgsz: int = 640
    conf: float = 0.35
    device: DeviceT = "auto"
    classes: list[int] | None = None
    deep_search: VisionProfile | None = None
    tracker: VisionProfile | None = None
    appearance: VisionProfile | None = None


@dataclass(frozen=True)
class DetectorPipelineConfig:
    detect_hz: float = 20.0
    track_hz: float = 60.0
    reference_height_m: float = 2.0
    output_mode: str = "all"
    use_target_lock: bool = True
    auto_target_lock: bool = True
    frame_source: int | str | None = None
    # Certified capture-lookup artifact for `frame_source` on the live link.
    # Absent by default: an uncalibrated real source can never be
    # frame-atomic (docs/decisions/vision-nav-capture-time-association).
    capture_calibration: "CaptureLookupCalibration | None" = None


@dataclass(frozen=True)
class DetectorGimbalConfig:
    tracking: GimbalTrackingSetup | None = None
    zoom: TargetZoomTrackerConfig | None = None


@dataclass(frozen=True)
class RealDetectorConfig:
    model: DetectorModelConfig
    pipeline: DetectorPipelineConfig = field(default_factory=DetectorPipelineConfig)
    debug: DetectorDebugConfig = field(default_factory=DetectorDebugConfig)
    gimbal: DetectorGimbalConfig = field(default_factory=DetectorGimbalConfig)


@dataclass(frozen=True)
class RealDetectorPorts:
    mount: CameraMount
    identity: SourceIdentity
    events: DetectionEventPort
    simulation: SimulationControlPort
    cadence: SchedulerCadence
    compatibility_navigation: GimbalNavigation | None
    tracking: DetectorTrackingControl
    geo: DetectorGeoControl
    lifecycle: DetectorLifecycle
    reset: DetectorResetController
    query: DetectionQuery
    diagnostics: DetectorDiagnostics


__all__ = [
    "DetectorDependencies",
    "DetectorGimbalConfig",
    "DetectorModelConfig",
    "DetectorPipelineConfig",
    "RealDetectorConfig",
    "RealDetectorPorts",
]
