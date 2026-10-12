"""Top-level composition of confirmation and recovery workflows."""

from __future__ import annotations

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.app.flow.nav_composition_types import (
    ConfirmationWorkflows,
    ConfirmationAdmissionWorkflows,
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.app.flow.nav_poi_status_composition import _compose_poi_status
from navpy.core.swarm.peer_task_priority import PeerTaskPriority
from navpy.app.flow.nav_track_recovery_composition import (
    _compose_track_recovery,
)
from navpy.core.perception.detection_coordination import DetectionCoordination


def compose_confirmation_workflows(
    detection: DetectionCoordination,
    logger: ILogger,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    admission: ConfirmationAdmissionWorkflows,
    reset: ResetWorkflows,
    navigation: NavCapabilities,
    peer_task: PeerTaskPriority,
) -> ConfirmationWorkflows:
    tracking = _compose_track_recovery(
        detection,
        logger,
        state,
        poi,
        approach,
        observation,
        reset,
        navigation,
    )
    poi_status = _compose_poi_status(
        detection,
        logger,
        state,
        poi,
        approach,
        observation,
        reset,
        navigation,
        tracking,
        admission.release,
        peer_task,
    )
    return ConfirmationWorkflows(
        reset=reset,
        confirmation_action=admission.action,
        poi_status=poi_status,
        deadline=admission.deadline,
    )


__all__ = ["compose_confirmation_workflows"]
