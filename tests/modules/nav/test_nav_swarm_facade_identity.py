"""Exact compatibility identities for the Nav/Swarm SRP facades."""

from importlib import import_module

import pytest


FACADE_EXPORTS = {
    "navpy.modules.nav.confirmation_workflow": {
        "navpy.modules.nav.confirmation_frame_policy": (
            "ConfirmationFramePolicy",
            "extract_confirmation_size",
        ),
        "navpy.modules.nav.confirmation_recognition": (
            "RecognitionGate",
            "RecognitionGatePorts",
        ),
        "navpy.modules.nav.confirmation_review": (
            "ConfirmationReview",
        ),
        "navpy.modules.nav.final_approach_record_deadline": (
            "FinalApproachRecordDeadline",
        ),
        "navpy.modules.nav.final_approach_release_gate": ("FinalApproachReleaseGate",),
    },
    "navpy.modules.nav.navigation_task": {
        "navpy.modules.nav.navigation_task_action": ("NavigationTaskAction",),
        "navpy.modules.nav.peer_poi_notification": (
            "PeerPoiNotifier",
            "PeerPoiNotifierPorts",
        ),
        "navpy.modules.nav.self_detected_approach": (
            "SelfDetectedApproach",
            "SelfDetectedApproachPorts",
        ),
        "navpy.modules.nav.poi_selection": ("PoiSelector",),
    },
    "navpy.modules.nav.nav_confirmation_composition": {
        "navpy.modules.nav.nav_confirmation_admission_composition": (
            "_compose_confirmation_admission",
        ),
        "navpy.modules.nav.nav_confirmation_workflow_composition": (
            "compose_confirmation_workflows",
        ),
        "navpy.modules.nav.nav_reset_composition": ("compose_reset_workflows",),
        "navpy.modules.nav.nav_poi_status_composition": (
            "_compose_poi_status",
        ),
        "navpy.modules.nav.nav_track_recovery_composition": (
            "_compose_track_recovery",
        ),
    },
    "navpy.modules.nav.navigation_transitions": {
        "navpy.modules.nav.nav_transition": (
            "NavExitOutcome",
            "NavTransition",
        ),
        "navpy.modules.nav.navigation_transition_handler": (
            "NavigationTransitionHandler",
            "TransitionOutcome",
            "TransitionPorts",
        ),
        "navpy.modules.nav.navigation_zoom": ("ZoomController",),
    },
    "navpy.modules.nav.poi_confirmation": {
        "navpy.modules.nav.confirmation_dependencies": (
            "ConfirmationFailurePolicy",
            "ConfirmationNetworkSlot",
            "FreshnessGate",
        ),
        "navpy.modules.nav.confirmation_round_runner": (
            "ConfirmationRoundRunner",
        ),
        "navpy.modules.nav.self_assignment_publisher": (
            "SelfAssignmentPublisher",
        ),
        "navpy.modules.nav.confirmation_coordinator": (
            "ConfirmationCoordinator",
        ),
    },
    "navpy.modules.nav.final_approach_navigation": {
        "navpy.modules.nav.final_approach_command_dispatch": (
            "FinalApproachCommandDispatch",
            "FinalApproachCommandPorts",
        ),
        "navpy.modules.nav.final_approach_record_commit": (
            "FinalApproachRecordCommit",
            "FinalApproachRecordDeferral",
            "FinalApproachRecordPorts",
        ),
        "navpy.modules.nav.final_approach_nav_workflow": (
            "NavPeerNotifier",
            "FinalApproachNavPorts",
            "FinalApproachNavWorkflow",
        ),
        "navpy.modules.nav.final_approach_source_admission": (
            "DetectionEventInbox",
            "NavSourceBatch",
            "FinalApproachSourceAdmission",
            "FinalApproachSourcePorts",
        ),
    },
    "navpy.modules.swarm.task_actor_state": {
        "navpy.modules.swarm.task_actor_slots": (
            "MAX_REMOTE_PEERS",
            "PeerRoster",
            "SelectedTaskSlot",
        ),
        "navpy.modules.swarm.task_auction_models": (
            "TaskAssignmentPlanner",
            "TaskOffer",
            "TaskRebroadcastPlan",
            "TaskRejectOutcome",
            "TaskReservation",
            "_TaskAuctionStore",
        ),
        "navpy.modules.swarm.task_auction_state": (
            "TaskAuctionState",
            "_busy_peers",
        ),
        "navpy.modules.swarm.task_auction_lifecycle": (
            "_shutdown_dispatches",
        ),
        "navpy.modules.swarm.task_rebroadcast_state": (
            "TaskRebroadcastState",
        ),
        "navpy.modules.swarm.task_state_composition": ("create_task_state",),
    },
}


IDENTITY_CASES = [
    (facade_name, owner_name, symbol)
    for facade_name, owners in FACADE_EXPORTS.items()
    for owner_name, symbols in owners.items()
    for symbol in symbols
]


@pytest.mark.parametrize(
    ("facade_name", "owner_name", "symbol"),
    IDENTITY_CASES,
)
def test_facade_export_is_exact_owner_identity(
    facade_name: str,
    owner_name: str,
    symbol: str,
) -> None:
    facade = import_module(facade_name)
    owner = import_module(owner_name)

    assert getattr(facade, symbol) is getattr(owner, symbol)
