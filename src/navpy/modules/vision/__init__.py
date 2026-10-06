"""Vision module - Detection, cameras, gimbals, and delivery-reference tracking.

This module owns ALL vision-related functionality:
- VisionController - Self-contained orchestrator for all vision components
- CameraMount - Encapsulates camera + gimbal
- Detector interfaces and implementations
- Camera interfaces and simulation
- Gimbal interfaces and simulation
- Visual-reference models and simulator delivery-reference providers

POI-prefixed exports retain compatible names. A detector observation or
stable track is not authenticated recipient identification.

Usage:
    from navpy.modules.vision import VisionController, CameraMount
    from navpy.modules.vision import DetectorAbc, DetectorSim
    from navpy.modules.vision import CameraAbc, CameraIntrinsics
    from navpy.modules.vision import GimbalAbc, GimbalSim
"""
# Controller
from navpy.modules.vision.vision_controller import VisionController
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detection_coordinator import DetectionCoordinator

# Core detector
from navpy.modules.vision.detector_abc import DetectorAbc
from navpy.modules.vision.sim.detector_sim import DetectorSim

# Camera
from navpy.modules.vision.peripheral.camera_abc import CameraAbc
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics

# Gimbal
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalAbc,
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.peripheral.fixed_gimbal import FixedGimbal
from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi
from navpy.modules.vision.sim.gimbal_sim import GimbalSim

# Tracker
from navpy.modules.vision.gimbal_rate_tracker import (
    GimbalRateTracker,
    GimbalRateTrackerConfig,
    GimbalTrackResult,
    TrackingState,
)
from navpy.modules.vision.poi_zoom_tracker import (
    PoiZoomTracker,
    PoiZoomTrackerConfig,
    ZoomTrackResult,
    ZoomTrackingState,
)
from navpy.modules.vision.poi_angle_estimator import (
    PoiAngleEstimator,
    PoiAngleEstimatorConfig,
)
# Models
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.poi_provider import PoiProvider
from navpy.modules.vision.models.detect_data import DetectedObject, DetectResult, DetectStatus
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detect_response import DetectResponse

__all__ = [
    # Controller
    "VisionController",
    "CameraMount",
    "DetectionCoordinator",
    # Detector
    "DetectorAbc",
    "DetectorSim",
    # Camera
    "CameraAbc",
    "CameraIntrinsics",
    # Gimbal
    "GimbalAbc",
    "GimbalData",
    "GimbalMountSetup",
    "FixedGimbal",
    "GimbalSiyi",
    "GimbalSim",
    # Tracker
    "GimbalRateTracker",
    "GimbalRateTrackerConfig",
    "GimbalTrackResult",
    "TrackingState",
    "PoiZoomTracker",
    "PoiZoomTrackerConfig",
    "ZoomTrackResult",
    "ZoomTrackingState",
    "PoiAngleEstimator",
    "PoiAngleEstimatorConfig",
    # Models
    "SimulationObject",
    "PoiProvider",
    "DetectedObject",
    "DetectResult",
    "DetectStatus",
    "DetectRequest",
    "DetectResponse",
]
