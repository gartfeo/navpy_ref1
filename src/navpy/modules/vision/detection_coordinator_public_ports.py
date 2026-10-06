"""Narrow composition views consumed by detector-coordinator facets."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from navpy.modules.vision.detector_ports import (
    DetectionEventPort,
    DetectionSnapshotPort,
    DetectorFleetMember,
    DetectorLifecyclePort,
    DetectorResetPort,
    GeoPointingPort,
    MountCatalogPort,
    SchedulerCadence,
    SimulationControlPort,
    PoiIdentityPort,
    TrackingCommandPort,
    TrackingStatusPort,
    ZoomControlPort,
)


class DetectorMemberCatalog(Protocol):
    @property
    def members(self) -> Sequence[DetectorFleetMember]: ...


class CoordinatorIdentityParts(Protocol):
    lifecycle: DetectorMemberCatalog
    mounts: MountCatalogPort


class CoordinatorEventParts(Protocol):
    snapshot: DetectionSnapshotPort
    events: DetectionEventPort


class CoordinatorLifecycleParts(Protocol):
    lifecycle: DetectorLifecyclePort
    reset: DetectorResetPort
    cadence: SchedulerCadence


class CoordinatorSimulationParts(Protocol):
    simulation: SimulationControlPort


class CoordinatorTrackingParts(Protocol):
    tracking_commands: TrackingCommandPort
    tracking_status: TrackingStatusPort
    poi_identity: PoiIdentityPort


class CoordinatorGeoParts(Protocol):
    geo_pointing: GeoPointingPort


class CoordinatorZoomParts(Protocol):
    zoom: ZoomControlPort


__all__ = [
    "CoordinatorEventParts",
    "CoordinatorGeoParts",
    "CoordinatorIdentityParts",
    "CoordinatorLifecycleParts",
    "CoordinatorSimulationParts",
    "CoordinatorTrackingParts",
    "CoordinatorZoomParts",
    "DetectorMemberCatalog",
]
