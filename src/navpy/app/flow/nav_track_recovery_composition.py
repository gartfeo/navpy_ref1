"""Composition of POI-loss and identity-reacquisition workflows."""

from __future__ import annotations

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    PoiMissionOwnership,
    TrackRecoveryWorkflows,
    VehicleApproachOwnership,
)
from navpy.core.skills.confirm.track_recovery import (
    GeoHoldCoordinator,
    IdentityReacquisition,
    IdentityReacquisitionPorts,
    TrackRecovery,
)
from navpy.core.flow.nav_state import NavState
from navpy.core.perception.detection_coordination import DetectionCoordination
from navpy.core.perception.tracking_geo_hand_off import GeoReleasingTracking


def _compose_track_recovery(
    detection: DetectionCoordination,
    logger: ILogger,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation: NavCapabilities,
) -> TrackRecoveryWorkflows:
    recovery = TrackRecovery(
        state.phase,
        state.geo_hold,
        state.confirm,
        GeoReleasingTracking(
            detection.tracking_commands,
            detection.geo_pointing.stop_geo_tracking,
        ),
        poi.confirmation_manager,
        observation.retry,
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
        lambda: state.phase.is_current(NavState.NAV),
        lambda: state.clock.decision_s(),
        lambda: state.navigation_task.peer_poi_location,
        recovery,
        identity,
        navigation.legacy_pois.geo_ref,
        logger,
    )
    return TrackRecoveryWorkflows(recovery, geo_coordinator)
