"""Navigation state decisions and inactive safety guards."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.navigation_task_reset import NavigationTaskResetTransaction
from navpy.modules.nav.mission_catalog import MissionCatalog
from navpy.modules.nav.mission_navigation import MissionPassPolicy
from navpy.modules.nav.nav_constants import (
    GUIDED_ACCEPT_TIMEOUT_S,
    GUIDED_REQUEST_RETRY_INTERVAL_S,
)
from navpy.modules.nav.nav_state import (
    ConfirmGateState,
    NavigationTaskState,
    NavigationFailureLatch,
    NavPhaseState,
    NavState,
)
from navpy.modules.nav.nav_status import NavigationStatusReporter
from navpy.modules.nav.recovery import RecoveryAction
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.nav.poi_status_decision import ConfirmationStatusDecision
from navpy.modules.nav.vehicle_navigation import LoiterRadiusLease
from navpy.modules.vehicle.flight_mode import FlightMode


@dataclass(frozen=True)
class InactiveGuardPorts:
    vehicle_armed: Callable[[], bool]
    clear_navigation_task: Callable[[], None]


class InactiveNavigationGuard:
    """Hold navigation while disarmed, manual, or before the detect waypoint."""

    ACTIVE_MODES = frozenset({FlightMode.GUIDED, FlightMode.AUTO, FlightMode.LOITER})

    def __init__(
        self,
        ports: InactiveGuardPorts,
        phase: NavPhaseState,
        navigation_task: NavigationTaskState,
        confirm: ConfirmGateState,
        mission: MissionCatalog,
        confirmation_manager: ConfirmationManager,
        loiter_radius: LoiterRadiusLease,
        mission_pass: MissionPassPolicy,
        status: NavigationStatusReporter,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._navigation_task = navigation_task
        self._confirm = confirm
        self._mission = mission
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
            or self._mission.fallback_delivery_location_active
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
            mode == FlightMode.AUTO
            and not self._mission_pass.passed_detection_waypoint(next_waypoint)
        ):
            self._status.ignore(1, f"ONHOLD: WAYPOINT {next_waypoint}")
            self._phase.request(NavState.ONHOLD)
            return True
        return False


@dataclass(frozen=True)
class NavDecisionPorts:
    clock_s: Callable[[], float]
    clear_navigation_task: Callable[[], None]
    request_guided: Callable[[], None]
    final_approach_active: Callable[[], bool]


class NavStateDecision:
    """Wait for GUIDED acceptance and classify NAV completion/failure."""

    def __init__(
        self,
        ports: NavDecisionPorts,
        phase: NavPhaseState,
        navigation_task: NavigationTaskState,
        failures: NavigationFailureLatch,
        confirmation_manager: ConfirmationManager,
        pass_policy: MissionPassPolicy,
        status: NavigationStatusReporter,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._navigation_task = navigation_task
        self._failures = failures
        self._confirmation_manager = confirmation_manager
        self._pass_policy = pass_policy
        self._status = status
        self._logger = logger

    def advance(self, mode: FlightMode) -> bool:
        if self._phase.current == NavState.NAV:
            if self._await_guided(mode):
                return True
            if (
                self._navigation_task.nav_mode_observed
                and self._pass_policy.passed_poi()
            ):
                self._mark_passed()
                return True
        if not self._failures.consume():
            return False
        if (
            self._phase.current == NavState.NAV
            and (
                self._ports.final_approach_active()
                or self._pass_policy.close_observed
            )
            and self._pass_policy.has_completion_evidence()
        ):
            self._mark_passed()
            return True
        self._confirmation_manager.clear_active_poi()
        self._phase.request(NavState.RESET)
        return True

    def _await_guided(self, mode: FlightMode) -> bool:
        if mode == FlightMode.GUIDED:
            self._navigation_task.nav_mode_observed = True
            self._navigation_task.final_approach_navigation_active = True
            self._clear_guided_wait()
            return False
        if self._navigation_task.nav_mode_observed:
            self._ports.clear_navigation_task()
            self._phase.request(NavState.DETECT)
            return True
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
            self._phase.request(NavState.DETECT)
            return True
        last_attempt_s = self._navigation_task.guided_last_attempt_at
        if (
            last_attempt_s is None
            or now_s - last_attempt_s >= GUIDED_REQUEST_RETRY_INTERVAL_S
        ):
            self._navigation_task.guided_last_attempt_at = now_s
            self._ports.request_guided()
        return True

    def _clear_guided_wait(self) -> None:
        self._navigation_task.guided_last_attempt_at = None
        self._navigation_task.guided_request_started_at = None

    def _mark_passed(self) -> None:
        self._navigation_task.final_approach_nav_completed = True
        self._status.ignore(2, "RESET: PASSED POI")
        self._phase.request(NavState.RESET)


@dataclass(frozen=True)
class NavigationDecisionPorts:
    vehicle_mode: Callable[[], FlightMode]
    next_waypoint: Callable[[], int]
    clock_s: Callable[[], float]


class NavigationDecision:
    """Execute the ordered state-decision pipeline."""

    def __init__(
        self,
        ports: NavigationDecisionPorts,
        phase: NavPhaseState,
        inactive: InactiveNavigationGuard,
        recovery: RecoveryAction,
        nav: NavStateDecision,
        poi_status: ConfirmationStatusDecision,
        args: NavArgs,
        mission: MissionCatalog,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._inactive = inactive
        self._recovery = recovery
        self._nav = nav
        self._poi_status = poi_status
        self._args = args
        self._mission = mission

    def decide(self) -> None:
        previous = self._phase.current
        mode = self._ports.vehicle_mode()
        next_waypoint = self._ports.next_waypoint()
        if self._inactive.apply(mode, next_waypoint):
            return
        if self._recovery.advance():
            return
        if self._nav.advance(mode):
            return
        if self._poi_status.dispatch():
            return
        self._refresh_onhold(previous)

    def _refresh_onhold(self, previous: NavState) -> None:
        if self._phase.current != NavState.ONHOLD:
            return
        now_s = self._ports.clock_s()
        if previous != NavState.ONHOLD:
            self._phase.last_parameter_refresh_s = now_s
        elif now_s - self._phase.last_parameter_refresh_s > 2.0:
            self._args.refresh()
            self._mission.refresh()
            self._phase.last_parameter_refresh_s = now_s


__all__ = [
    "NavDecisionPorts",
    "NavStateDecision",
    "InactiveGuardPorts",
    "InactiveNavigationGuard",
    "NavigationDecision",
    "NavigationDecisionPorts",
]
