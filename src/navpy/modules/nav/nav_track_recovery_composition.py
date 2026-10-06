"""Composition of target-loss and identity-reacquisition workflows."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    TargetMissionOwnership,
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
    target: TargetMissionOwnership,
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
        target.confirmation_manager,
        observation.retry,
        reset.resume_auto,
        logger,
    )
    identity = IdentityReacquisition(
        IdentityReacquisitionPorts(
            ground_location=navigation.legacy_targets.ground_location,
            absolute_location=approach.commands.absolute_location,
        ),
        state.geo_hold,
        state.detections,
        detection.target_identity,
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
        lambda: navigation.terminal.is_active,
        lambda: state.clock.decision_s(),
        lambda: state.navigation_task.navigation_target_location,
        recovery,
        identity,
        navigation.legacy_targets.geo_ref,
        logger,
    )
    return TrackRecoveryWorkflows(recovery, geo_coordinator)
