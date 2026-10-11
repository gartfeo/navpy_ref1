"""Stable DetectionCoordinator API over focused coordination owners."""

from __future__ import annotations

from collections.abc import Sequence

from navpy.core.perception.detection_coordination import (
    DetectionCoordination,
    build_detection_coordination,
)
from navpy.core.perception.detection_coordinator_facets import (
    CoordinatorCompositionFacet,
    CoordinatorEventsFacet,
    CoordinatorGeoFacet,
    CoordinatorIdentityFacet,
    CoordinatorLifecycleFacet,
    CoordinatorSimulationFacet,
    CoordinatorTrackingFacet,
    CoordinatorZoomFacet,
)
from navpy.core.perception.detector_ports import DetectorFleetMember
from navpy.core.perception.tracking_command_router import TrackingLogger


class DetectionCoordinator(
    CoordinatorCompositionFacet,
    CoordinatorIdentityFacet,
    CoordinatorEventsFacet,
    CoordinatorLifecycleFacet,
    CoordinatorSimulationFacet,
    CoordinatorTrackingFacet,
    CoordinatorGeoFacet,
    CoordinatorZoomFacet,
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
