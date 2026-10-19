"""Flow: a dock contact completes the approach on the navigation thread.

physical_docking.md: the contact callback only latches and fences; the
navigation loop picks the completion up in its next decision, sets
`final_approach_nav_completed` and requests RESET. First latch wins between
contact and NAV_FAIL, and the completion lifecycle (one-shot or RESET) is the
existing one.
"""

from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.core.approach.dock_contact_episode import FinalApproachDockContact
from navpy.core.approach.nav_snap_reporter import NavSnapReporter
from navpy.core.approach.navigation_final_approach import (
    FinalApproachNavigationService,
)
from navpy.core.approach.navigation_mode import (
    ActiveNavigationMode,
    NavigationFinalApproachCapabilities,
    NavigationModeState,
)
from navpy.core.approach.nav_exit_cleanup import NavCleanupResult
from navpy.core.flow.mission_navigation import MissionPassPolicy, MissionPassPorts
from navpy.core.flow.nav_state import (
    NavigationFailureLatch,
    NavigationTaskState,
    NavPhaseState,
    NavState,
)
from navpy.core.flow.nav_transition import NavTransition
from navpy.core.flow.navigation_decision import (
    ApproachOutcome,
    NavDecisionPorts,
    NavStateDecision,
)
from navpy.core.flow.navigation_loop import StateActionDispatcher
from navpy.core.flow.navigation_transition_handler import TransitionOutcome
from navpy.core.ports.dock_contact import DockContactEvent
from navpy.core.ports.flight_mode import FlightMode
from navpy.core.skills.geo_approach.pass_tracker import LegacyPassTracker
from navpy.products.delivery.flow import DeliveryFlow
from tests.detection_factory import make_detected_poi
from tests.modules.navigation.test_navigation import RichPoi, _execute_one, _frame, _runtime


class _Sensor:
    def __init__(self) -> None:
        self.armed = []
        self.disarms = 0

    def arm(self, identity, on_docked) -> None:
        self.armed.append((identity, on_docked))

    def disarm(self) -> None:
        self.disarms += 1


class _RecordedApproach:
    """Pass the real approach decision through and keep its last outcome."""

    def __init__(self, decision: NavStateDecision) -> None:
        self._decision = decision
        self.outcome = None

    def advance(self, mode: FlightMode) -> ApproachOutcome:
        self.outcome = self._decision.advance(mode)
        return self.outcome


class _Flow:
    def __init__(self) -> None:
        self.runtime, self.actuator, _, _ = _runtime()
        mode_state = NavigationModeState()
        mode_state.install_initialized(
            ActiveNavigationMode(
                spec=Mock(),
                runtime=self.runtime,
                final_approach=NavigationFinalApproachCapabilities(
                    status=self.runtime.session,
                    confirmation=Mock(),
                ),
            ),
            expected_previous=None,
        )
        self.service = FinalApproachNavigationService(mode_state)
        self.phase = NavPhaseState(NavState.NAV, NavState.NAV)
        self.task = NavigationTaskState()
        self.failures = NavigationFailureLatch(self.service.latch_failure)
        self.confirmation = Mock()
        self.status = Mock()
        policy = MissionPassPolicy(
            MissionPassPorts(
                final_approach_active=lambda: True,
                poi_passed_override=self.service.poi_passed_override,
                last_visual_bearing_deg=lambda: None,
                current_absolute=lambda: None,
                locked_poi_distance=lambda: None,
            ),
            Mock(),
            self.task,
            LegacyPassTracker(
                close_distance_m=10.0,
                distance_epsilon_m=0.1,
                increase_samples=3,
                behind_bearing_deg=90.0,
            ),
            Mock(),
        )
        self.decision = NavStateDecision(
            NavDecisionPorts(
                clock_s=lambda: 0.0,
                clear_navigation_task=Mock(),
                request_guided=Mock(),
                final_approach_active=lambda: True,
            ),
            self.phase,
            self.task,
            self.failures,
            policy,
            Mock(),
        )
        self.approach = _RecordedApproach(self.decision)
        self.flow = DeliveryFlow(
            self.phase,
            self.approach,
            Mock(),
            self.status,
            self.confirmation.clear_active_poi,
        )
        self.sensor = _Sensor()
        self.logger = Mock()
        self.contact = FinalApproachDockContact(
            self.sensor,
            lambda: 7,
            self.service.begin_dock_episode,
            self.service.latch_docked,
            self.logger,
        )
        self.poi = make_detected_poi(obj_id=2, task_id=1)

    def arm(self):
        self.contact.arm(self.poi)
        identity, on_docked = self.sensor.armed[-1]
        return identity, on_docked

    def decide(self) -> bool:
        self.flow.decide(FlightMode.GUIDED)
        return self.approach.outcome is not ApproachOutcome.NONE


def _event(identity) -> DockContactEvent:
    return DockContactEvent(identity, 50.0, 0.2, 0.1, 0.1)


def test_arming_uses_vehicle_task_own_dock_and_a_new_episode_each_entry():
    flow = _Flow()
    first, _ = flow.arm()
    second, _ = flow.arm()  # approach restart / new assignment

    assert (first.vehicle, first.task_id, first.dock_uid) == (7, 1, 2)
    assert (first.episode, second.episode) == (1, 2)
    assert flow.sensor.disarms == 2


def test_contact_from_an_earlier_episode_is_ignored_after_a_restart():
    flow = _Flow()
    first, on_docked = flow.arm()
    flow.arm()
    on_docked(_event(first))
    assert flow.service.poi_passed_override() is False


def test_contact_is_completed_by_the_navigation_thread_not_the_callback():
    flow = _Flow()
    identity, on_docked = flow.arm()
    flow.task.nav_mode_observed = True

    on_docked(_event(identity))
    # The callback latched only: no phase change, no completion flag yet.
    assert flow.phase.current is NavState.NAV
    assert flow.task.final_approach_nav_completed is False
    flow.logger.info.assert_called_once()
    assert flow.logger.info.call_args.args[0].startswith("DOCKED: task=1 dock=2")

    assert flow.decide() is True
    assert flow.task.final_approach_nav_completed is True
    assert flow.phase.current is NavState.RESET
    flow.status.ignore.assert_called_once_with(2, "RESET: PASSED POI")


def test_contact_during_a_navigation_transition_does_not_change_the_phase():
    flow = _Flow()
    identity, on_docked = flow.arm()
    flow.task.nav_mode_observed = True
    flow.phase.request(NavState.CONFIRM)  # some transition is pending

    class Transitions:
        @staticmethod
        def on_change(previous, requested):
            on_docked(_event(identity))  # the reader thread fires mid-transition
            return TransitionOutcome(NavState.NAV, False)

    dispatcher = StateActionDispatcher(
        flow.phase,
        Transitions(),
        {state: (lambda: None) for state in NavState},
        lambda committed: None,
    )
    dispatcher.act()  # commit_transition raises if the phase moved under it

    assert flow.phase.current is NavState.NAV
    assert flow.decide() is True
    assert flow.phase.current is NavState.RESET
    assert flow.task.final_approach_nav_completed is True


def test_contact_then_detection_loss_completes_and_drops_the_failure():
    flow = _Flow()
    identity, on_docked = flow.arm()
    flow.task.nav_mode_observed = True
    on_docked(_event(identity))

    assert flow.failures.mark_failed() is False  # NAV_FAIL: no_detection
    assert flow.failures.failed is False
    assert flow.decide() is True
    assert flow.task.final_approach_nav_completed is True


def test_nav_fail_then_contact_resets_without_completion():
    flow = _Flow()
    identity, on_docked = flow.arm()
    flow.task.nav_mode_observed = True

    assert flow.failures.mark_failed() is True
    on_docked(_event(identity))
    assert flow.service.poi_passed_override() is False
    assert flow.runtime.take_work() is None  # no DOCKED evidence staged

    assert flow.decide() is True
    assert flow.task.final_approach_nav_completed is False
    assert flow.phase.current is NavState.RESET
    flow.confirmation.clear_active_poi.assert_called_once_with()


def test_zero_commands_after_the_contact_latch():
    flow = _Flow()
    identity, on_docked = flow.arm()
    flow.runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(flow.runtime)
    on_docked(_event(identity))
    flow.runtime.take_work()  # the staged DOCKED evidence
    flow.runtime.nav(RichPoi(_frame(2.0)))
    _execute_one(flow.runtime)

    assert len(flow.actuator.calls) == 1


def _nav_transition(task: NavigationTaskState, oneshot: bool):
    cleanup = Mock()
    cleanup.run.return_value = NavCleanupResult(snap=None, errors=())
    completion = Mock()
    completion.run.return_value = ()
    transition = NavTransition(
        Mock(),
        cleanup,
        Mock(spec=NavSnapReporter),
        completion.run,
        task,
        lambda: oneshot,
    )
    return transition, completion


@pytest.mark.parametrize("oneshot", (True, False))
def test_docked_completion_uses_the_existing_lifecycle(oneshot):
    """One-shot force-disarms SITL (caught by the net); normal mode resets."""
    flow = _Flow()
    identity, on_docked = flow.arm()
    flow.task.nav_mode_observed = True
    flow.task.final_approach_navigation_active = True
    on_docked(_event(identity))
    flow.decide()
    transition, completion = _nav_transition(flow.task, oneshot)

    outcome = transition.exit()

    assert completion.run.called is oneshot
    assert outcome.oneshot_completed is oneshot
    assert outcome.redirect is (NavState.ONHOLD if oneshot else None)


def test_no_sensor_never_arms_and_never_completes():
    """A real build wires no sensor: the net catches the aircraft."""
    flow = _Flow()
    contact = FinalApproachDockContact(
        None, lambda: 7, flow.service.begin_dock_episode,
        flow.service.latch_docked, Mock(),
    )
    contact.arm(flow.poi)
    contact.disarm()
    assert flow.service.poi_passed_override() is False


def test_failure_latch_without_an_arbiter_always_latches():
    latch = NavigationFailureLatch()
    assert latch.mark_failed() is True
    assert latch.consume() is True


def test_failure_latch_asks_the_arbiter_before_latching():
    latch = NavigationFailureLatch(lambda _reason: False)
    assert latch.mark_failed() is False
    assert latch.failed is False


def test_episode_rows_carry_the_source_time_at_nav_entry_and_exit():
    flow = _Flow()
    clock = iter((10.0, 10.9))
    begun, ended = [], []
    contact = FinalApproachDockContact(
        flow.sensor, lambda: 7,
        lambda identity, source_s: begun.append((identity, source_s)) or True,
        flow.service.latch_docked, Mock(),
        source_now_s=lambda: next(clock),
        end_episode=lambda identity, source_s: ended.append((identity, source_s)),
    )

    contact.arm(flow.poi)
    contact.disarm()
    contact.disarm()  # nothing armed: no second exit row

    assert [source for _identity, source in begun] == [10.0]
    assert ended == [(begun[0][0], 10.9)]
