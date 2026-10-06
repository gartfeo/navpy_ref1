"""Narrow composition views consumed by simulated-detector facets."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision.sim.detection_publication_buffer import (
    DetectionPublicationBuffer,
)
from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_detector_controls import (
    SimDetectorIdentity,
    SimGeoControls,
    SimPoiControls,
    SimTrackingControls,
    SimZoomControls,
)
from navpy.modules.vision.sim.sim_detector_lifecycle import SimDetectorLifecycle
from navpy.modules.vision.sim.sim_detector_loop import SimDetectorWorker
from navpy.modules.vision.sim.sim_poi_projector import SimPoiProjector


class SimIdentityParts(Protocol):
    identity: SimDetectorIdentity
    navigation: GimbalNavigation | None


class SimTrackingParts(Protocol):
    tracking: SimTrackingControls
    zoom: SimZoomControls


class SimGeoParts(Protocol):
    geo: SimGeoControls
    tracking: SimTrackingControls


class SimLifecycleParts(Protocol):
    lifecycle: SimDetectorLifecycle
    cadence: SimDetectorWorker


class SimDetectionParts(Protocol):
    detection: DetectionPublicationBuffer


class SimControlParts(Protocol):
    identity: SimDetectorIdentity
    simulation: SimPoiControls


class SimProjectionParts(Protocol):
    renderer: SimDetectionPipeline
    projector: SimPoiProjector


__all__ = [
    "SimControlParts",
    "SimDetectionParts",
    "SimGeoParts",
    "SimIdentityParts",
    "SimLifecycleParts",
    "SimProjectionParts",
    "SimTrackingParts",
]
