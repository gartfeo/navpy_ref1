"""Composition of POI-status decisions after confirmation."""

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
from navpy.modules.nav.confirmed_poi_release import ConfirmedPoiRelease
from navpy.modules.nav.poi_presence_review import (
    PoiPresencePorts,
    PoiPresenceReview,
)
from navpy.modules.nav.poi_rejection_exit import (
    PoiRejectionExit,
    PoiRejectionPorts,
)
from navpy.modules.nav.poi_status_decision import (
    PoiSelectionPorts,
    ConfirmationStatusDecision,
)
from navpy.modules.nav.final_approach_release_gate import FinalApproachReleaseGate
from navpy.modules.vision.detection_coordination import DetectionCoordination


def _compose_poi_status(
    detection: DetectionCoordination,
    logger: ILogger,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    reset: ResetWorkflows,
    navigation: NavCapabilities,
    tracking: TrackRecoveryWorkflows,
    release: FinalApproachReleaseGate,
) -> ConfirmationStatusDecision:
    poi_status = ConfirmationStatusDecision(
        PoiSelectionPorts(
            active_poi=lambda: poi.confirmation_manager.active_poi,
            poi_status=poi.confirmation_manager.get_status,
        ),
        state.phase,
        observation.status,
        PoiPresenceReview(
            PoiPresencePorts(
                final_approach_active=lambda: navigation.final_approach.is_active,
                absolute_location=approach.commands.absolute_location,
            ),
            state.geo_hold,
            observation.source,
            observation.timing,
            tracking.geo_coordinator,
        ),
        ConfirmedPoiRelease(
            state.phase,
            state.geo_hold,
            observation.retry,
            tracking.recovery,
            release,
            lambda: navigation.final_approach.is_active,
        ),
        PoiRejectionExit(
            PoiRejectionPorts(
                wall_s=lambda: state.clock.wall_s(),
                clear_active_poi=(
                    poi.confirmation_manager.clear_active_poi
                ),
                stop_tracking=detection.tracking_commands.stop_tracking,
                logger=logger,
            ),
            state.phase,
            observation.timing,
            observation.retry,
            reset.resume_auto,
        ),
    )
    return poi_status
