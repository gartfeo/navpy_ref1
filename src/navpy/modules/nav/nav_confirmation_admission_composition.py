"""Composition of confirmation review and final-approach admission gates."""

from __future__ import annotations

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.confirmation_action import (
    ConfirmationAction,
    ConfirmationGeoHold,
    FinalApproachConfirmationAdmission,
    FinalApproachConfirmationPorts,
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
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.nav.final_approach_record_deadline import FinalApproachRecordDeadline
from navpy.modules.nav.final_approach_release_gate import FinalApproachReleaseGate
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


def compose_confirmation_admission_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_task: NavigationTaskWorkflows,
    navigation: NavCapabilities,
) -> ConfirmationAdmissionWorkflows:
    review = ConfirmationReview(
        LocalConfirmationPublisher(
            lambda detected: poi.confirmation_manager.update_status(
                detected,
                ConfirmationStatus.CONFIRMED,
            ),
            logger,
        ),
        OperatorReviewPublisher(
            final_approach_active=lambda: navigation.final_approach.is_active,
            review=lambda pois: poi.confirmation_manager.review(pois),
            freeze_final_approach_zoom=observation.zoom.freeze_final_approach_wide,
            set_tracking_zoom=observation.zoom.set_recognition_demand,
            logger=logger,
            legacy=(
                None
                if navigation.final_approach.is_active
                else LegacyReviewPreparation(
                    LegacyReviewPorts(
                        ground_location=(
                            navigation.legacy_pois.ground_location
                        ),
                        current_location=lambda: vehicle.location(False),
                        goto_poi=navigation.vehicle_commands.peer_poi,
                        goto_loiter=(
                            navigation.vehicle_commands.peer_poi_loiter
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
    final_approach_admission = FinalApproachConfirmationAdmission(
        FinalApproachConfirmationPorts(
            final_approach_active=lambda: navigation.final_approach.is_active,
            can_confirm=navigation.final_approach.can_confirm_detection,
            record_confirmed=navigation.final_approach.record_confirmed_detection,
            auto_confirm=lambda: args.is_auto_confirm,
        ),
        state.final_approach,
        review,
        observation.debug,
    )
    confirmation_geo_hold = ConfirmationGeoHold(
        state.geo_hold,
        poi.confirmation_manager,
        detection.geo_pointing,
        navigation_task.peer_geo_acquisition,
        lambda: vehicle.location(False),
        lambda: vehicle.attitude,
    )
    confirmation_action = ConfirmationAction(
        state.detections,
        navigation_task.selector,
        navigation_task.peer_notifier,
        poi.confirmation_manager,
        observation.source,
        ConfirmationFramePolicy(observation.debug),
        final_approach_admission,
        recognition,
        observation.blocked,
        observation.debug,
        confirmation_geo_hold,
    )
    release = FinalApproachReleaseGate(
        observation.source,
        observation.freshness,
        observation.debug,
    )
    deadline = FinalApproachRecordDeadline(
        state.final_approach,
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
