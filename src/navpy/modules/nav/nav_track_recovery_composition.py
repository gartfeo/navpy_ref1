"""Composition of POI-loss and identity-reacquisition workflows."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    PoiMissionOwnership,
    TrackRecoveryWorkflows,
    VehicleApproachOwnership,
)
from navpy.modules.nav.track_recovery import (
    GeoHoldCoordinator,
    IdentityReacquisition,
    IdentityReacquisitionPorts,
    TrackRecovery,
)
from navpy.modules.vision.detection_coordination import DetectionCoordination


def _compose_track_recovery(
    detection: DetectionCoordination,
    logger: ILogger,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    reset: ResetWorkflows,
    navigation: NavCapabilities,
) -> TrackRecoveryWorkflows:
    recovery = TrackRecovery(
        state.phase,
        state.geo_hold,
        state.confirm,
        detection.tracking_commands,
        poi.confirmation_manager,
        observation.retry,
        reset.resume_auto,
        logger,
    )
    identity = IdentityReacquisition(
        IdentityReacquisitionPorts(
            ground_location=navigation.legacy_pois.ground_location,
            absolute_location=approach.commands.absolute_location,
        ),
        state.geo_hold,
        state.detections,
        detection.poi_identity,
        observation.retry,
        recovery,
        logger,
    )
    geo_coordinator = GeoHoldCoordinator(
        state.geo_hold,
        state.confirm,
        detection.tracking_commands,
        detection.tracking_status,
        detection.geo_pointing,
        lambda: navigation.final_approach.is_active,
        lambda: state.clock.decision_s(),
        lambda: state.navigation_task.navigation_poi_location,
        recovery,
        identity,
        navigation.legacy_pois.geo_ref,
        logger,
    )
    return TrackRecoveryWorkflows(recovery, geo_coordinator)
