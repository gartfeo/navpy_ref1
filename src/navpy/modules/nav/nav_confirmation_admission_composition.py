"""Composition of confirmation review and terminal admission gates."""

from __future__ import annotations

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.confirmation_action import (
    ConfirmationAction,
    ConfirmationGeoHold,
    TerminalConfirmationAdmission,
    TerminalConfirmationPorts,
)
from navpy.modules.nav.confirmation_frame_policy import ConfirmationFramePolicy
from navpy.modules.nav.confirmation_legacy_prep import (
    LegacyReviewPorts,
    LegacyReviewPreparation,
)
from navpy.modules.nav.confirmation_local import LocalConfirmationPublisher
from navpy.modules.nav.confirmation_operator_review import OperatorReviewPublisher
from navpy.modules.nav.confirmation_recognition import (
    RecognitionGate,
    RecognitionGatePorts,
)
from navpy.modules.nav.confirmation_review import ConfirmationReview
from navpy.modules.nav.nav_composition_types import (
    ConfirmationAdmissionWorkflows,
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    NavCapabilities,
    NavStateOwnership,
    TargetMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.nav.terminal_record_deadline import TerminalRecordDeadline
from navpy.modules.nav.terminal_release_gate import TerminalReleaseGate
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


def compose_confirmation_admission_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    target: TargetMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_task: NavigationTaskWorkflows,
    navigation: NavCapabilities,
) -> ConfirmationAdmissionWorkflows:
    review = ConfirmationReview(
        LocalConfirmationPublisher(
            lambda detected: target.confirmation_manager.update_status(
                detected,
                ConfirmationStatus.CONFIRMED,
            ),
            logger,
        ),
        OperatorReviewPublisher(
            terminal_active=lambda: navigation.terminal.is_active,
            review=lambda targets: target.confirmation_manager.review(targets),
            freeze_terminal_zoom=observation.zoom.freeze_terminal_wide,
            set_tracking_zoom=observation.zoom.set_recognition_demand,
            logger=logger,
            legacy=(
                None
                if navigation.terminal.is_active
                else LegacyReviewPreparation(
                    LegacyReviewPorts(
                        ground_location=(
                            navigation.legacy_targets.ground_location
                        ),
                        current_location=lambda: vehicle.location(False),
                        goto_target=navigation.vehicle_commands.peer_target,
                        goto_loiter=(
                            navigation.vehicle_commands.peer_target_loiter
                        ),
                        absolute_location=approach.commands.absolute_location,
                    ),
                    state.navigation_task,
                    approach.loiter_radius,
                    approach_kind,
                )
            ),
        ),
    )
    recognition = RecognitionGate(
        RecognitionGatePorts(
            vision_profile=observation.vision_profile,
            logger=logger,
        ),
        review,
        observation.timing,
        state.overrides,
        observation.debug,
        observation.blocked,
    )
    terminal_admission = TerminalConfirmationAdmission(
        TerminalConfirmationPorts(
            terminal_active=lambda: navigation.terminal.is_active,
            can_confirm=navigation.terminal.can_confirm_detection,
            record_confirmed=navigation.terminal.record_confirmed_detection,
            auto_confirm=lambda: args.is_auto_confirm,
        ),
        state.terminal,
        review,
        observation.debug,
    )
    confirmation_geo_hold = ConfirmationGeoHold(
        state.geo_hold,
        target.confirmation_manager,
        detection.geo_pointing,
        navigation_task.peer_geo_acquisition,
        lambda: vehicle.location(False),
        lambda: vehicle.attitude,
    )
    confirmation_action = ConfirmationAction(
        state.detections,
        navigation_task.selector,
        navigation_task.peer_notifier,
        target.confirmation_manager,
        observation.source,
        ConfirmationFramePolicy(observation.debug),
        terminal_admission,
        recognition,
        observation.blocked,
        observation.debug,
        confirmation_geo_hold,
    )
    release = TerminalReleaseGate(
        observation.source,
        observation.freshness,
        observation.debug,
    )
    deadline = TerminalRecordDeadline(
        state.terminal,
        state.navigation_failures.mark_failed,
        lambda: state.clock.decision_s(),
        logger,
        observation.debug,
    )
    return ConfirmationAdmissionWorkflows(
        confirmation_action,
        release,
        deadline,
    )


_compose_confirmation_admission = compose_confirmation_admission_workflows


__all__ = [
    "_compose_confirmation_admission",
    "compose_confirmation_admission_workflows",
]
