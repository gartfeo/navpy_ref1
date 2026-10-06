"""Stable DetectionCoordinator API over focused coordination owners."""

from __future__ import annotations

from collections.abc import Sequence

from navpy.modules.vision.detection_coordination import (
    DetectionCoordination,
    build_detection_coordination,
)
from navpy.modules.vision.detection_coordinator_facets import (
    CoordinatorCompositionFacet,
    CoordinatorEventsFacet,
    CoordinatorGeoFacet,
    CoordinatorIdentityFacet,
    CoordinatorLifecycleFacet,
    CoordinatorSimulationFacet,
    CoordinatorTrackingFacet,
    CoordinatorZoomFacet,
)
from navpy.modules.vision.detector_abc import DetectorAbc
from navpy.modules.vision.detector_ports import DetectorFleetMember
from navpy.modules.vision.tracking_command_router import TrackingLogger


class DetectionCoordinator(
    CoordinatorCompositionFacet,
    CoordinatorIdentityFacet,
    CoordinatorEventsFacet,
    CoordinatorLifecycleFacet,
    CoordinatorSimulationFacet,
    CoordinatorTrackingFacet,
    CoordinatorGeoFacet,
    CoordinatorZoomFacet,
    DetectorAbc,
):
    """One-field compatibility boundary over detector-fleet capabilities."""

    def __init__(
        self,
        detectors: Sequence[DetectorFleetMember],
        logger: TrackingLogger,
    ) -> None:
        self._parts: DetectionCoordination = build_detection_coordination(
            detectors,
            logger,
        )



__all__ = ["DetectionCoordinator"]
