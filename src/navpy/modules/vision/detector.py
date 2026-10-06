"""Public real-detector adapter and compatibility exports."""

from __future__ import annotations

from navpy.modules.vision.detector_abc import DetectorAbc
from navpy.modules.vision.detector_public_control import (
    DetectorGeoFacet,
    DetectorIdentityFacet,
    DetectorTrackingFacet,
)
from navpy.modules.vision.detector_public_diagnostics import (
    DetectorDiagnosticsFacet,
)
from navpy.modules.vision.detector_public_runtime import (
    DetectorEventsFacet,
    DetectorLifecycleFacet,
    DetectorQueryFacet,
    DetectorSimulationFacet,
)
from navpy.modules.vision.multi_object_tracker import (
    KalmanCV,
    TrackedObject,
    hungarian,
    iou_xyxy,
)
from navpy.modules.vision.real_detector_composition import (
    DetectorDependencies,
    DetectorGimbalConfig,
    DetectorModelConfig,
    DetectorPipelineConfig,
    RealDetectorConfig,
    RealDetectorPorts,
    build_real_detector,
)
from navpy.modules.vision.real_detector_diagnostics import DetectorDebugConfig
from navpy.modules.vision.yolo_detector import Detection, DeviceT, YoloDetector


class Detector(
    DetectorIdentityFacet,
    DetectorTrackingFacet,
    DetectorGeoFacet,
    DetectorLifecycleFacet,
    DetectorEventsFacet,
    DetectorQueryFacet,
    DetectorSimulationFacet,
    DetectorDiagnosticsFacet,
    DetectorAbc,
):
    """One-field public adapter over focused real-detector capabilities."""

    def __init__(
        self,
        dependencies: DetectorDependencies,
        config: RealDetectorConfig,
    ) -> None:
        self._parts: RealDetectorPorts = build_real_detector(
            dependencies,
            config,
        )


__all__ = [
    "Detector",
    "DetectorDebugConfig",
    "DetectorDependencies",
    "DetectorGimbalConfig",
    "DetectorModelConfig",
    "DetectorPipelineConfig",
    "RealDetectorConfig",
    "Detection",
    "TrackedObject",
    "YoloDetector",
    "KalmanCV",
    "hungarian",
    "iou_xyxy",
    "DeviceT",
]
