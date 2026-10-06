"""Narrow composition views consumed by real-detector public facets."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision.camera_mount import CameraMount
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
from navpy.modules.vision.real_detector_diagnostics import DetectorDiagnostics
from navpy.modules.vision.real_detector_lifecycle import DetectorLifecycle


class RealIdentityParts(Protocol):
    mount: CameraMount
    identity: SourceIdentity
    compatibility_navigation: GimbalNavigation | None


class RealTrackingParts(Protocol):
    tracking: DetectorTrackingControl


class RealGeoParts(Protocol):
    geo: DetectorGeoControl


class RealLifecycleParts(Protocol):
    lifecycle: DetectorLifecycle
    reset: DetectorResetController
    cadence: SchedulerCadence


class RealEventParts(Protocol):
    events: DetectionEventPort


class RealQueryParts(Protocol):
    query: DetectionQuery


class RealSimulationParts(Protocol):
    simulation: SimulationControlPort


class RealDiagnosticsParts(Protocol):
    diagnostics: DetectorDiagnostics


__all__ = [
    "RealDiagnosticsParts",
    "RealEventParts",
    "RealGeoParts",
    "RealIdentityParts",
    "RealLifecycleParts",
    "RealQueryParts",
    "RealSimulationParts",
    "RealTrackingParts",
]
