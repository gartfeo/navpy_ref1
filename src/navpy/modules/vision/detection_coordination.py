"""Composition root for detector-fleet capabilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from navpy.modules.navigation.gimbal_navigation_state import GimbalLossPolicy
from navpy.modules.vision.detection_aggregation import DetectionAggregator
from navpy.modules.vision.detection_identity_registry import DetectionIdentityRegistry
from navpy.modules.vision.detector_lifecycle_fleet import DetectorLifecycleFleet
from navpy.modules.vision.detector_mount_catalog import DetectorMountCatalog
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
    TargetIdentityPort,
    TrackingCommandPort,
    TrackingStatusPort,
    ZoomControlPort,
)
from navpy.modules.vision.detector_reset_fleet import DetectorResetFleet
from navpy.modules.vision.geo_pointing_fleet import GeoPointingFleet
from navpy.modules.vision.scheduler_cadence_fleet import SchedulerCadenceFleet
from navpy.modules.vision.simulation_control_fleet import SimulationControlFleet
from navpy.modules.vision.tracking_command_router import (
    TrackingCommandRouter,
    TrackingLogger,
)
from navpy.modules.vision.tracking_status_fleet import TrackingStatusFleet
from navpy.modules.vision.zoom_control_router import ZoomControlRouter


@dataclass(frozen=True)
class DetectionCoordination:
    snapshot: DetectionSnapshotPort
    events: DetectionEventPort
    tracking_commands: TrackingCommandPort
    tracking_status: TrackingStatusPort
    target_identity: TargetIdentityPort
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
        target_identity=identities,
        geo_pointing=GeoPointingFleet(members, logger),
        zoom=ZoomControlRouter(members, identities),
        mounts=DetectorMountCatalog(members),
        simulation=SimulationControlFleet(members),
        reset=DetectorResetFleet(members, identities),
        lifecycle=DetectorLifecycleFleet(members, logger),
        cadence=SchedulerCadenceFleet(members),
    )


__all__ = ["DetectionCoordination", "build_detection_coordination"]
