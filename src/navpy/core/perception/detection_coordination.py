"""Composition root for detector-fleet capabilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from navpy.modules.navigation.gimbal_navigation_state import GimbalLossPolicy
from navpy.core.perception.detection_aggregation import DetectionAggregator
from navpy.core.perception.detection_identity_registry import DetectionIdentityRegistry
from navpy.core.perception.detector_lifecycle_fleet import DetectorLifecycleFleet
from navpy.core.perception.detector_mount_catalog import DetectorMountCatalog
from navpy.core.perception.detector_ports import (
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
from navpy.core.perception.detector_reset_fleet import DetectorResetFleet
from navpy.core.perception.geo_pointing_fleet import GeoPointingFleet
from navpy.core.perception.scheduler_cadence_fleet import SchedulerCadenceFleet
from navpy.core.perception.simulation_control_fleet import SimulationControlFleet
from navpy.core.perception.tracking_command_router import (
    TrackingCommandRouter,
    TrackingLogger,
)
from navpy.core.perception.tracking_status_fleet import TrackingStatusFleet
from navpy.core.perception.zoom_control_router import ZoomControlRouter


@dataclass(frozen=True)
class DetectionCoordination:
    snapshot: DetectionSnapshotPort
    events: DetectionEventPort
    tracking_commands: TrackingCommandPort
    tracking_status: TrackingStatusPort
    poi_identity: PoiIdentityPort
    geo_pointing: GeoPointingPort
    zoom: ZoomControlPort
    mounts: MountCatalogPort
    simulation: SimulationControlPort
    reset: DetectorResetPort
    lifecycle: DetectorLifecyclePort
    cadence: SchedulerCadence


def build_detection_coordination(
    detectors: Sequence[DetectorFleetMember],
    logger: TrackingLogger,
) -> DetectionCoordination:
    members = tuple(detectors)
    identities = DetectionIdentityRegistry()
    aggregator = DetectionAggregator(members, identities)
    return DetectionCoordination(
        snapshot=aggregator,
        events=aggregator,
        tracking_commands=TrackingCommandRouter(members, identities, logger),
        tracking_status=TrackingStatusFleet(
            members,
            GimbalLossPolicy().hold_sec,
        ),
        poi_identity=identities,
        geo_pointing=GeoPointingFleet(members, logger),
        zoom=ZoomControlRouter(members, identities),
        mounts=DetectorMountCatalog(members),
        simulation=SimulationControlFleet(members),
        reset=DetectorResetFleet(members, identities),
        lifecycle=DetectorLifecycleFleet(members, logger),
        cadence=SchedulerCadenceFleet(members),
    )


__all__ = ["DetectionCoordination", "build_detection_coordination"]
