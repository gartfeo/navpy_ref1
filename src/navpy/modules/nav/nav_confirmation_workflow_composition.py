"""Top-level composition of confirmation and recovery workflows."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.nav_composition_types import (
    ConfirmationWorkflows,
    ConfirmationAdmissionWorkflows,
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    TargetMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.nav_target_status_composition import _compose_target_status
from navpy.modules.nav.nav_track_recovery_composition import (
    _compose_track_recovery,
)
from navpy.modules.vision.detection_coordination import DetectionCoordination


def compose_confirmation_workflows(
    detection: DetectionCoordination,
    logger: ILogger,
    state: NavStateOwnership,
    target: TargetMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    admission: ConfirmationAdmissionWorkflows,
    reset: ResetWorkflows,
    navigation: NavCapabilities,
) -> ConfirmationWorkflows:
    tracking = _compose_track_recovery(
        detection,
        logger,
        state,
        target,
        approach,
        observation,
        reset,
        navigation,
    )
    target_status = _compose_target_status(
        detection,
        logger,
        state,
        target,
        approach,
        observation,
        reset,
        navigation,
        tracking,
        admission.release,
    )
    return ConfirmationWorkflows(
        reset=reset,
        confirmation_action=admission.action,
        target_status=target_status,
        deadline=admission.deadline,
    )


__all__ = ["compose_confirmation_workflows"]
