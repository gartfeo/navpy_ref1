"""Composition of target-status decisions after confirmation."""

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
from navpy.modules.nav.confirmed_target_release import ConfirmedTargetRelease
from navpy.modules.nav.target_presence_review import (
    TargetPresencePorts,
    TargetPresenceReview,
)
from navpy.modules.nav.target_rejection_exit import (
    TargetRejectionExit,
    TargetRejectionPorts,
)
from navpy.modules.nav.target_status_decision import (
    TargetSelectionPorts,
    ConfirmationStatusDecision,
)
from navpy.modules.nav.terminal_release_gate import TerminalReleaseGate
from navpy.modules.vision.detection_coordination import DetectionCoordination


def _compose_target_status(
    detection: DetectionCoordination,
    logger: ILogger,
    state: NavStateOwnership,
    target: TargetMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    reset: ResetWorkflows,
    navigation: NavCapabilities,
    tracking: TrackRecoveryWorkflows,
    release: TerminalReleaseGate,
) -> ConfirmationStatusDecision:
    target_status = ConfirmationStatusDecision(
        TargetSelectionPorts(
            active_target=lambda: target.confirmation_manager.active_target,
            target_status=target.confirmation_manager.get_status,
        ),
        state.phase,
        observation.status,
        TargetPresenceReview(
            TargetPresencePorts(
                terminal_active=lambda: navigation.terminal.is_active,
                absolute_location=approach.commands.absolute_location,
            ),
            state.geo_hold,
            observation.source,
            observation.timing,
            tracking.geo_coordinator,
        ),
        ConfirmedTargetRelease(
            state.phase,
            state.geo_hold,
            observation.retry,
            tracking.recovery,
            release,
            lambda: navigation.terminal.is_active,
        ),
        TargetRejectionExit(
            TargetRejectionPorts(
                wall_s=lambda: state.clock.wall_s(),
                clear_active_target=(
                    target.confirmation_manager.clear_active_target
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
    return target_status
