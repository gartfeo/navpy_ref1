"""Typed configuration and public ports for one real detector."""

from __future__ import annotations

from dataclasses import dataclass, field

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.capture_lookup import CaptureLookupCalibration
from navpy.modules.vision.charuco_board import CharucoBoardSpec
from navpy.args.detector_backend import (
    YOLO_BACKEND,
    DetectorBackend,
    validate_detector_backend,
)
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
from navpy.modules.vision.poi_zoom_types import PoiZoomTrackerConfig
from navpy.modules.vision.vision_profile_types import VisionProfile


@dataclass(frozen=True)
class DetectorDependencies:
    vehicle: IVehicle
    mount: CameraMount
    logger: ILogger


@dataclass(frozen=True)
class DetectorModelConfig:
    """Per-frame inference backend configuration.

    ``backend`` defaults to ``yolo`` here because a config built around an
    explicit ``model_path`` describes a model detector (bench tools rely on
    this). The product default backend is chosen once at the CLI seam
    (``VisionArgs``), which passes ``backend`` explicitly.
    ``model_path``, ``imgsz``, ``device``, ``classes`` and ``deep_search``
    apply to ``yolo`` only; ``charuco_board`` applies to ``charuco`` only;
    ``conf`` is the minimum detection confidence for either backend.
    """

    model_path: str
    imgsz: int = 640
    conf: float = 0.35
    device: DeviceT = "auto"
    classes: list[int] | None = None
    deep_search: VisionProfile | None = None
    tracker: VisionProfile | None = None
    appearance: VisionProfile | None = None
    backend: DetectorBackend = YOLO_BACKEND
    charuco_board: CharucoBoardSpec = field(default_factory=CharucoBoardSpec)

    def __post_init__(self) -> None:
        validate_detector_backend(self.backend)


@dataclass(frozen=True)
class DetectorPipelineConfig:
    detect_hz: float = 20.0
    track_hz: float = 60.0
    reference_height_m: float = 2.0
    output_mode: str = "all"
    use_poi_lock: bool = True
    auto_poi_lock: bool = True
    frame_source: int | str | None = None
    # Certified capture-lookup artifact for `frame_source` on the live link.
    # Absent by default: an uncalibrated real source can never be
    # frame-atomic.
    capture_calibration: "CaptureLookupCalibration | None" = None


@dataclass(frozen=True)
class DetectorGimbalConfig:
    tracking: GimbalTrackingSetup | None = None
    zoom: PoiZoomTrackerConfig | None = None


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
