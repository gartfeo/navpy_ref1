"""Composition of POI-status decisions after confirmation."""

from __future__ import annotations

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    PoiMissionOwnership,
    TrackRecoveryWorkflows,
    VehicleApproachOwnership,
)
from navpy.core.skills.confirm.confirmed_poi_release import ConfirmedPoiRelease
from navpy.core.skills.confirm.poi_presence_review import (
    PoiPresencePorts,
    PoiPresenceReview,
)
from navpy.core.skills.confirm.poi_rejection_exit import (
    PoiRejectionExit,
    PoiRejectionPorts,
)
from navpy.core.flow.poi_status_decision import (
    PoiSelectionPorts,
    ConfirmationStatusDecision,
)
from navpy.core.skills.confirm.final_approach_release_gate import FinalApproachReleaseGate
from navpy.core.swarm.peer_task_priority import PeerTaskPriority
from navpy.core.flow.nav_state import NavState
from navpy.core.perception.detection_coordination import DetectionCoordination


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
    peer_task: PeerTaskPriority,
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
                final_approach_running=lambda: state.phase.is_current(NavState.NAV),
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
        peer_task,
    )
    return poi_status
