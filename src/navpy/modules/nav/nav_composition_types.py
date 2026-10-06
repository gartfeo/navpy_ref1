"""Typed ownership boundaries shared by navigation composition stages."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.modules.navigation.navigation_terminal import TerminalNavigationService
from navpy.modules.navigation.navigation_source_dispatch import (
    BindSourceDispatch,
)
from navpy.modules.navigation.navigation_vehicle_commands import NavigationVehicleCommands
from navpy.modules.navigation.legacy_destination_resolver import LegacyDestinationResolver
from navpy.modules.nav.approach_planner import ApproachPlanner
from navpy.modules.nav.confirmation_action import ConfirmationAction
from navpy.modules.nav.confirmation_policy import (
    ConfirmationTimingPolicy,
    TargetRetryPolicy,
)
from navpy.modules.nav.confirmation_reporting import (
    ConfirmBlockedReporter,
    ConfirmDebugReporter,
)
from navpy.modules.nav.terminal_record_deadline import TerminalRecordDeadline
from navpy.modules.nav.terminal_release_gate import TerminalReleaseGate
from navpy.modules.nav.detection_freshness import DetectionFreshnessPolicy
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.navigation_task_action import NavigationTaskAction
from navpy.modules.nav.peer_target_notification import PeerTargetNotifier
from navpy.modules.nav.target_selection import TargetSelector
from navpy.modules.nav.navigation_task_reset import (
    AutoMissionResume,
    NavigationTaskResetTransaction,
)
from navpy.modules.nav.navigation_speedup import NavigationSpeedupLease
from navpy.modules.nav.mission_catalog import MissionCatalog
from navpy.modules.nav.mission_navigation import (
    FallbackMissionNavigation,
    MissionPassPolicy,
)
from navpy.modules.nav.nav_clock import NavClock
from navpy.modules.nav.nav_network import NavNetworkRuntime
from navpy.modules.nav.nav_state import (
    ConfirmGateState,
    ConfirmOverrideInbox,
    NavigationTaskState,
    GeoHoldState,
    NavigationFailureLatch,
    NavPhaseState,
    TerminalNavState,
)
from navpy.modules.nav.nav_status import NavigationStatusReporter
from navpy.modules.nav.navigation_decision import NavigationDecision
from navpy.modules.nav.navigation_zoom import ZoomController
from navpy.modules.nav.pass_tracker import LegacyPassTracker
from navpy.modules.nav.peer_geo import PeerGeoAcquisition
from navpy.modules.nav.recovery import RecoveryAction
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.nav.target_retry import TargetRetryState
from navpy.modules.nav.target_status_decision import ConfirmationStatusDecision
from navpy.modules.nav.terminal_source_admission import TerminalSourceAdmission
from navpy.modules.nav.terminal_source_session import TerminalSourceSession
from navpy.modules.nav.terminal_publication_admission import (
    TerminalPublicationAdmission,
)
from navpy.modules.nav.vehicle_navigation import (
    LoiterRadiusLease,
    VehicleNavigationCommands,
)
from navpy.modules.nav.track_recovery import GeoHoldCoordinator, TrackRecovery
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class NavCapabilities:
    """Only the focused navigation capabilities consumed by navigation."""

    terminal: TerminalNavigationService
    legacy_targets: LegacyDestinationResolver
    vehicle_commands: NavigationVehicleCommands
    init: Callable[[], None]
    reset: Callable[[], object]
    pause: Callable[[], None]
    nav: Callable[[Optional[DetectedObject]], bool]
    bind_source_dispatch: BindSourceDispatch
    algorithm_info: Callable[[], tuple[str, Optional[float]]]


@dataclass(frozen=True)
class NavStateOwnership:
    """Authoritative mutable state, separated from side-effecting workflows."""

    clock: NavClock
    phase: NavPhaseState
    detections: DetectionSnapshot
    navigation_task: NavigationTaskState
    navigation_failures: NavigationFailureLatch
    terminal: TerminalNavState
    geo_hold: GeoHoldState
    confirm: ConfirmGateState
    overrides: ConfirmOverrideInbox


@dataclass(frozen=True)
class TargetMissionOwnership:
    """Target identity, retry, source-session, and mission state owners."""

    mission: MissionCatalog
    pass_tracker: LegacyPassTracker
    retry_state: TargetRetryState
    confirmation_manager: ConfirmationManager
    source_session: TerminalSourceSession


@dataclass(frozen=True)
class VehicleApproachOwnership:
    """Approach planning and reversible vehicle-side leases."""

    planner: ApproachPlanner
    loiter_radius: LoiterRadiusLease
    commands: VehicleNavigationCommands
    speedup: NavigationSpeedupLease


@dataclass(frozen=True)
class DetectionReviewOwnership:
    """Detection admission, review policy, status, and network ownership."""

    network: NavNetworkRuntime
    source: TerminalSourceAdmission
    publication_admission: TerminalPublicationAdmission
    freshness: DetectionFreshnessPolicy
    retry: TargetRetryPolicy
    status: NavigationStatusReporter
    debug: ConfirmDebugReporter
    blocked: ConfirmBlockedReporter
    timing: ConfirmationTimingPolicy
    zoom: ZoomController
    vision_profile: dict


@dataclass(frozen=True)
class MissionNavigationOwnership:
    fallback_navigation: FallbackMissionNavigation
    mission_pass: MissionPassPolicy


@dataclass(frozen=True)
class NavigationTaskWorkflows:
    mission_navigation: MissionNavigationOwnership
    peer_geo_acquisition: PeerGeoAcquisition
    selector: TargetSelector
    peer_notifier: PeerTargetNotifier
    navigation_task_action: NavigationTaskAction


@dataclass(frozen=True)
class ResetWorkflows:
    reset: NavigationTaskResetTransaction
    resume_auto: AutoMissionResume


@dataclass(frozen=True)
class TrackRecoveryWorkflows:
    recovery: TrackRecovery
    geo_coordinator: GeoHoldCoordinator


@dataclass(frozen=True)
class ConfirmationAdmissionWorkflows:
    action: ConfirmationAction
    release: TerminalReleaseGate
    deadline: TerminalRecordDeadline


@dataclass(frozen=True)
class ConfirmationWorkflows:
    reset: ResetWorkflows
    confirmation_action: ConfirmationAction
    target_status: ConfirmationStatusDecision
    deadline: TerminalRecordDeadline


@dataclass(frozen=True)
class DecisionWorkflows:
    recovery: RecoveryAction
    decision: NavigationDecision


__all__ = [
    "ConfirmationAdmissionWorkflows",
    "ConfirmationWorkflows",
    "DecisionWorkflows",
    "DetectionReviewOwnership",
    "NavigationTaskWorkflows",
    "MissionNavigationOwnership",
    "NavCapabilities",
    "NavStateOwnership",
    "ResetWorkflows",
    "TargetMissionOwnership",
    "TrackRecoveryWorkflows",
    "VehicleApproachOwnership",
]
