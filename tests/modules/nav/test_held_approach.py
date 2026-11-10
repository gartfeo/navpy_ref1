"""A final approach held by NAV recovery: ended without failure, a fresh one after the resume (owner ruling
2026-11-09, "New approach"; found in step 9 SITL).

In SITL a NAV cycle fault mid-approach held the UAV in LOITER and resumed in AUTO, but the re-approach failed at
once: the command worker's held command had gone stale while fenced, latching the episode's failure and command
liveness; the resumed approach consumed it as ``NAV_FAIL: source_command_stale`` and ended DOCK_FAILED (in SITL, a
one-shot disarm). This runs the real decision, ``HeldApproach``, ``NavTransition``, NAV entry and exit cleanup, with
the real command-liveness and episode latches; the navigation stand-in resets them as the lifecycle's reset does
(``reset_phase``).
"""
from __future__ import annotations

from unittest.mock import Mock

from navpy.core.approach.nav_entry import NavEntry, NavEntryPorts
from navpy.core.approach.nav_exit_cleanup import NavExitCleanup, NavExitCleanupPorts
from navpy.core.approach.vision_nav import nav_fail_reason as fail_reason
from navpy.core.approach.vision_nav.command_liveness import (
    FinalApproachCommandLiveness,
    FinalApproachEpisodeFailures,
)
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeLatch
from navpy.core.flow.nav_held_approach import HeldApproach
from navpy.core.flow.nav_state import NavigationFailureLatch, NavigationTaskState, NavPhaseState, NavState
from navpy.core.flow.nav_transition import NavTransition
from navpy.core.flow.navigation_decision import ApproachOutcome, NavDecisionPorts, NavStateDecision
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.flight_mode import FlightMode


class Approach:
    """The final approach's episode state, reset as ``NavigationLifecycle.reset``/``init`` do."""

    def __init__(self) -> None:
        self.liveness = FinalApproachCommandLiveness()
        self.episode = FinalApproachEpisodeLatch()
        self.worker = FinalApproachEpisodeFailures(rt.RLock(), self.episode, self.liveness)
        self.inits = 0
        self.resets = 0
        self.failing_inits = 0      # the next inits that raise

    def init(self) -> None:
        self.inits += 1
        if self.failing_inits:
            self.failing_inits -= 1
            raise RuntimeError("init failed")
        self.liveness.reset()
        self.episode.reset()

    def reset(self) -> object:
        self.resets += 1
        self.liveness.reset()
        self.episode.reset()
        return None


def _rig(recovery=None):
    """``recovery``: a real NAV recovery rig (test_nav_recovery.Rig) for the hold, the resume grant and NavPy's
    GUIDED requests (told to recovery, then sent to its fake autopilot); else plain values a test sets."""
    approach = Approach()
    task = NavigationTaskState()
    failures = NavigationFailureLatch(approach.episode.latch_failure)
    held = HeldApproach()
    holding = [False]
    resumed = [False]
    guided_requests: list[str] = []
    # Arming dock contact begins the armed episode (FinalApproachNavigationService.begin_dock_episode).
    contact = Mock(disarm=Mock(return_value=None), arm=Mock(side_effect=lambda: approach.episode.begin(Mock())))
    outcome = Mock(terminal=Mock(side_effect=approach.episode.terminal_snapshot), record_failed=Mock())
    transition = NavTransition(
        NavEntry(
            NavEntryPorts(
                request_guided=lambda: _request_guided(recovery, guided_requests),
                navigation_init=approach.init,
                final_approach_active=lambda: False,
                arm_dock_contact=contact.arm,
                stop_geo_tracking=lambda: None,
                is_geo_armed=lambda: False,
            ),
            task, Mock(), Mock(),
        ),
        NavExitCleanup(
            NavExitCleanupPorts(
                disarm_dock_contact=contact.disarm,
                close_final_approach_source=Mock(),
                navigation_reset=approach.reset,
                detector_stop=Mock(name="stop_tracking"),
                dock_outcome=outcome,
            ),
            Mock(restore=Mock(return_value=True)), Mock(),
        ),
        Mock(), Mock(return_value=()), task, lambda: True, logger=Mock(), held=held,
    )

    decision = NavStateDecision(
        NavDecisionPorts(
            clock_s=lambda: 0.0,
            clear_navigation_task=Mock(),
            request_guided=lambda: guided_requests.append("GUIDED retry"),
            final_approach_active=lambda: True,
            holding=(lambda: holding[0]) if recovery is None else (lambda: recovery.hold.holding),
            # Recovery's resume grant (RecoveryHold.resumed): set by a test, or by real recovery; used up once.
            resume_granted=(lambda: resumed[0]) if recovery is None else (lambda: recovery.hold.resumed),
            consume_resume=(lambda: resumed.__setitem__(0, False)) if recovery is None else recovery.hold.consume_resume,
            end_held_episode=held.end_episode,
            held_episode_ended=lambda: held.ended,
            start_fresh_episode=held.start_fresh_episode,
        ),
        NavPhaseState(current=NavState.NAV, previous=NavState.CONFIRM),
        task, failures, Mock(passed_poi=Mock(return_value=False)), Mock(),
    )
    return approach, task, failures, held, holding, resumed, guided_requests, contact, outcome, transition, decision


def _request_guided(recovery, requests: list[str]) -> None:
    requests.append("GUIDED")
    if recovery is not None:     # request_guided_owned: recovery first, then the vehicle
        recovery.hold.guided_requested()
        recovery.ap.request(FlightMode.GUIDED)


def test_hold_mid_approach_fence_resume_then_a_fresh_episode_without_source_command_stale():
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()                                         # NAV entry: episode 1
    assert decision.advance(FlightMode.GUIDED) is ApproachOutcome.NONE
    assert task.nav_mode_observed

    # The fault: NAV stops feeding; the fenced worker's held command goes stale (1.0 s) before the hold (1.5 s).
    approach.worker.mark_failed()
    assert approach.episode.terminal_snapshot() is not None   # the held episode failed in the worker...

    holding[0] = True                                          # recovery holds: LOITER, commands fenced
    assert decision.advance(FlightMode.LOITER) is ApproachOutcome.WAITING
    assert held.ended and approach.resets == 1                 # ...and the hold ended it, without failure
    assert contact.disarm.call_count == 1
    assert decision.advance(FlightMode.LOITER) is ApproachOutcome.WAITING
    assert approach.resets == 1                                 # once per hold

    holding[0], resumed[0] = False, True                       # healthy: recovery resumed in NavPy's AUTO
    assert decision.advance(FlightMode.AUTO) is ApproachOutcome.WAITING
    assert not held.ended and approach.inits == 2              # a fresh episode: NAV entry again
    assert requests == ["GUIDED", "GUIDED"] and contact.arm.call_count == 2

    assert decision.advance(FlightMode.GUIDED) is ApproachOutcome.NONE   # the fresh approach flies
    # What act_nav reads first: no stale liveness carried over, so no NAV_FAIL: source_command_stale.
    assert approach.liveness.consume_failure() is False
    assert failures.consume() is False
    assert approach.episode.terminal_snapshot() is None
    outcome.record_failed.assert_not_called()                   # never DOCK_FAILED
    assert task.nav_mode_observed


def test_a_hold_whose_mode_the_operator_takes_ends_the_approach_without_dock_failed():
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()
    decision.advance(FlightMode.GUIDED)
    approach.worker.mark_failed()
    holding[0] = True
    decision.advance(FlightMode.LOITER)
    holding[0] = False                                         # the operator selected FBWA: no resume
    assert decision.advance(FlightMode.FBWA) is ApproachOutcome.ABANDONED
    assert approach.inits == 1                                 # no fresh episode over the operator's mode
    outcome.record_failed.assert_not_called()
    assert approach.liveness.consume_failure() is False


def test_the_reason_the_held_episode_failed_for_is_not_reported():
    """Without the hold's end, the resume would read the latched failure as source_command_stale."""
    approach = Approach()
    approach.worker.mark_failed()
    assert approach.liveness.consume_failure() is True      # what a resumed, un-ended episode would read
    assert fail_reason.SOURCE_COMMAND_STALE == "source_command_stale"
