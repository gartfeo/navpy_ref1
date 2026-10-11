"""Public real-detector adapter and compatibility exports."""

from __future__ import annotations

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
from navpy.modules.vision.real_detector_composition import (
    DetectorDependencies,
    RealDetectorConfig,
    RealDetectorPorts,
    build_real_detector,
)
from navpy.modules.vision.real_detector_config import DetectorGimbalConfig
from navpy.core.kernel.device import DeviceT


class Detector(
    DetectorIdentityFacet,
    DetectorTrackingFacet,
    DetectorGeoFacet,
    DetectorLifecycleFacet,
    DetectorEventsFacet,
    DetectorQueryFacet,
    DetectorSimulationFacet,
    DetectorDiagnosticsFacet,
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
    "DetectorDependencies",
    "DetectorGimbalConfig",
    "RealDetectorConfig",
    "DeviceT",
]
