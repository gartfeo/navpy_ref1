"""Typed configuration and public ports for one real detector."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.command.gimbal.gimbal_navigation import GimbalNavigation
from navpy.core.command.gimbal.gimbal_navigation_state import GimbalTrackingSetup
from navpy.core.command.gimbal.gimbal_search import GimbalSearchSetup
from navpy.core.perception.camera.camera_mount import CameraMount
from navpy.core.perception.camera.capture_lookup import CaptureLookupCalibration
from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig
from navpy.core.perception.pipeline.integration_log import TimingLog
from navpy.core.perception.zoom.fixed_step_zoom_drive import FixedStepZoom
from navpy.core.ports.frame_capture_ports import CaptureBackendWrapper
from navpy.plugins.detection.charuco.charuco_board import (
    CharucoBoardSpec,
    CharucoDetectionSettings,
)
from navpy.core.kernel.config.detector_backend import (
    YOLO_BACKEND,
    DetectorBackend,
    validate_detector_backend,
)
from navpy.core.kernel.device import DeviceT
from navpy.core.perception.detector_ports import (
    DetectionEventPort,
    SchedulerCadence,
    SimulationControlPort,
    SourceIdentity,
)
from navpy.core.perception.pipeline.real_detector_controls import (
    DetectionQuery,
    DetectorGeoControl,
    DetectorResetController,
    DetectorTrackingControl,
)
from navpy.core.perception.pipeline.real_detector_diagnostics import (
    DetectorDebugConfig,
    DetectorDiagnostics,
)
from navpy.core.perception.pipeline.real_detector_lifecycle import DetectorLifecycle
from navpy.core.perception.zoom.poi_zoom_types import PoiZoomTrackerConfig
from navpy.core.perception.profile.vision_profile_types import VisionProfile


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
    apply to ``yolo`` only; ``charuco_board`` and ``charuco_detection`` apply
    to ``charuco`` only;
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
    charuco_detection: CharucoDetectionSettings = field(
        default_factory=CharucoDetectionSettings,
    )

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
    # HITL-only uncertified timing (stage 1 step 11), exclusive with
    # capture_calibration; set by the HITL runner only, never by a profile.
    integration_timing: "IntegrationTimingConfig | None" = None
    # Receives one record per association on integration timing.
    integration_timing_log: "TimingLog | None" = None
    # HITL commissioning only (stage 1 step 15): wraps the decoded capture;
    # supplied with integration timing by a launcher, never by a profile.
    capture_wrapper: "CaptureBackendWrapper | None" = None
    # HITL step 18: calibrated zoom steps and their settle evidence, built with
    # integration timing from the profile's reading table; never alone.
    fixed_zoom: "FixedStepZoom | None" = None
    # HITL step 20: the law's only airspeed reader with integration timing (the eligible pitot source,
    # app/approach/law_airspeed.py); None outside HITL, where the vehicle's airspeed is read.
    law_airspeed: "Callable[[], float | None] | None" = None

    def __post_init__(self) -> None:
        if self.fixed_zoom is not None and self.integration_timing is None:
            raise ValueError("fixed zoom needs integration timing")
        if (self.integration_timing is None) != (self.law_airspeed is None):
            # Never VFR_HUD on integration timing; never the eligible source outside it.
            raise ValueError("integration timing and the eligible airspeed reader go together")


@dataclass(frozen=True)
class DetectorGimbalConfig:
    tracking: GimbalTrackingSetup | None = None
    zoom: PoiZoomTrackerConfig | None = None
    # HITL step 21: the profile's search sweep, built with integration timing only (app/perception).
    search: GimbalSearchSetup | None = None


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
