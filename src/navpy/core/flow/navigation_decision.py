"""Navigation state decisions and inactive safety guards."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.flow.navigation_task_reset import NavigationTaskResetTransaction
from navpy.core.flow.mission_navigation import MissionPassPolicy
from navpy.core.flow.nav_constants import (
    GUIDED_ACCEPT_TIMEOUT_S,
    GUIDED_REQUEST_RETRY_INTERVAL_S,
)
from navpy.core.flow.nav_state import (
    ConfirmGateState,
    NavigationTaskState,
    NavigationFailureLatch,
    NavPhaseState,
    NavState,
)
from navpy.core.flow.nav_status import NavigationStatusReporter
from navpy.core.flow.recovery import RecoveryAction
from navpy.core.review.confirmation_manager import ConfirmationManager
from navpy.core.skills.acquire.vehicle_navigation import LoiterRadiusLease
from navpy.core.ports.flight_mode import FlightMode


@dataclass(frozen=True)
class InactiveGuardPorts:
    vehicle_armed: Callable[[], bool]
    clear_navigation_task: Callable[[], None]
    flow_task_active: Callable[[], bool]
    # A swarm-assigned peer task flies to its dock from wherever the UAV is,
    # so the detection-waypoint hold does not apply to it; own search still
    # waits (DetectAction). Disarm or a manual mode cancels it like any task.
    peer_task_assigned: Callable[[], bool]


class InactiveNavigationGuard:
    """Hold navigation while disarmed, manual, or before the detect waypoint.

    The detect-waypoint hold does not apply once a peer task is assigned.
    """

    ACTIVE_MODES = frozenset({FlightMode.GUIDED, FlightMode.AUTO, FlightMode.LOITER})

    def __init__(
        self,
        ports: InactiveGuardPorts,
        phase: NavPhaseState,
        navigation_task: NavigationTaskState,
        confirm: ConfirmGateState,
        confirmation_manager: ConfirmationManager,
        loiter_radius: LoiterRadiusLease,
        mission_pass: MissionPassPolicy,
        status: NavigationStatusReporter,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._navigation_task = navigation_task
        self._confirm = confirm
        self._confirmation_manager = confirmation_manager
        self._loiter_radius = loiter_radius
        self._mission_pass = mission_pass
        self._status = status

    def apply(self, mode: FlightMode, next_waypoint: int) -> bool:
        has_active_task = (
            self._loiter_radius.original is not None
            or self._confirm.hold_active
            or self._confirmation_manager.active_poi is not None
            or self._navigation_task.navigation_poi_location is not None
            or self._navigation_task.peer_navigation
            or self._ports.flow_task_active()
            or self._ports.peer_task_assigned()
        )
        if not self._ports.vehicle_armed():
            if has_active_task:
                self._ports.clear_navigation_task()
            self._phase.oneshot_completed = False
            self._status.ignore(4, "ONHOLD: DISARMED")
            self._phase.request(NavState.ONHOLD)
            return True
        if mode not in self.ACTIVE_MODES:
            if has_active_task:
                self._ports.clear_navigation_task()
            self._status.ignore(3, f"ONHOLD: MODE {mode.name}")
            self._phase.request(NavState.ONHOLD)
            return True
        if (
            not self._ports.peer_task_assigned()
            and self._mission_pass.holds_own_search(mode, next_waypoint)
        ):
            self._status.ignore(1, f"ONHOLD: WAYPOINT {next_waypoint}")
            self._phase.request(NavState.ONHOLD)
            return True
        return False


class ApproachOutcome(Enum):
    """What the approach decision reports for one tick."""

    NONE = auto()  # nothing to report: the POI review runs
    WAITING = auto()  # waiting for GUIDED
    ABANDONED = auto()  # GUIDED lost or not accepted; the task is cleared
    PASSED = auto()  # the approach completed
    FAILED = auto()  # the approach failed


@dataclass(frozen=True)
class NavDecisionPorts:
    clock_s: Callable[[], float]
    clear_navigation_task: Callable[[], None]
    request_guided: Callable[[], None]
    final_approach_active: Callable[[], bool]


class NavStateDecision:
    """Wait for GUIDED acceptance and classify NAV completion/failure.

    It reports the result and requests no phase; the caller picks the next one.
    """

    def __init__(
        self,
        ports: NavDecisionPorts,
        phase: NavPhaseState,
        navigation_task: NavigationTaskState,
        failures: NavigationFailureLatch,
        pass_policy: MissionPassPolicy,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._navigation_task = navigation_task
        self._failures = failures
        self._pass_policy = pass_policy
        self._logger = logger

    def advance(self, mode: FlightMode) -> ApproachOutcome:
        if self._phase.current == NavState.NAV:
            waiting = self._await_guided(mode)
            if waiting is not ApproachOutcome.NONE:
                return waiting
            if (
                self._navigation_task.nav_mode_observed
                and self._pass_policy.passed_poi()
            ):
                return self._mark_passed()
        if not self._failures.consume():
            return ApproachOutcome.NONE
        if (
            self._phase.current == NavState.NAV
            and (
                self._ports.final_approach_active()
                or self._pass_policy.close_observed
            )
            and self._pass_policy.has_completion_evidence()
        ):
            return self._mark_passed()
        return ApproachOutcome.FAILED

    def _await_guided(self, mode: FlightMode) -> ApproachOutcome:
        if mode == FlightMode.GUIDED:
            self._navigation_task.nav_mode_observed = True
            self._navigation_task.final_approach_navigation_active = True
            self._clear_guided_wait()
            return ApproachOutcome.NONE
        if self._navigation_task.nav_mode_observed:
            self._ports.clear_navigation_task()
            return ApproachOutcome.ABANDONED
        now_s = self._ports.clock_s()
        if self._navigation_task.guided_request_started_at is None:
            self._navigation_task.guided_request_started_at = now_s
        elif (
            now_s - self._navigation_task.guided_request_started_at
            > GUIDED_ACCEPT_TIMEOUT_S
        ):
            self._logger.warning(
                f"GUIDED not accepted within {GUIDED_ACCEPT_TIMEOUT_S:g}s; "
                "aborting navigation task to DETECT",
                key="nav",
            )
            self._ports.clear_navigation_task()
            return ApproachOutcome.ABANDONED
        last_attempt_s = self._navigation_task.guided_last_attempt_at
        if (
            last_attempt_s is None
            or now_s - last_attempt_s >= GUIDED_REQUEST_RETRY_INTERVAL_S
        ):
            self._navigation_task.guided_last_attempt_at = now_s
            self._ports.request_guided()
        return ApproachOutcome.WAITING

    def _clear_guided_wait(self) -> None:
        self._navigation_task.guided_last_attempt_at = None
        self._navigation_task.guided_request_started_at = None

    def _mark_passed(self) -> ApproachOutcome:
        self._navigation_task.final_approach_nav_completed = True
        return ApproachOutcome.PASSED


@dataclass(frozen=True)
class NavigationDecisionPorts:
    vehicle_mode: Callable[[], FlightMode]
    next_waypoint: Callable[[], int]
    is_oneshot: Callable[[], bool]


class NavigationDecision:
    """Execute the ordered state-decision pipeline.

    Core's holds run first; when none acts, the flow decides with the mode.
    """

    def __init__(
        self,
        ports: NavigationDecisionPorts,
        phase: NavPhaseState,
        inactive: InactiveNavigationGuard,
        recovery: RecoveryAction,
        flow: Callable[[FlightMode], None],
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._inactive = inactive
        self._recovery = recovery
        self._flow = flow

    def decide(self) -> None:
        previous = self._phase.current
        if previous is NavState.DOCKED and self._ports.is_oneshot():
            # A single-cycle dock is final (docked_state.md): no other
            # decision may turn it into ONHOLD, DETECT or a new task.
            return
        mode = self._ports.vehicle_mode()
        next_waypoint = self._ports.next_waypoint()
        if self._inactive.apply(mode, next_waypoint):
            return
        if self._recovery.advance():
            return
        self._flow(mode)


__all__ = [
    "ApproachOutcome",
    "NavDecisionPorts",
    "NavStateDecision",
    "InactiveGuardPorts",
    "InactiveNavigationGuard",
    "NavigationDecision",
    "NavigationDecisionPorts",
]
