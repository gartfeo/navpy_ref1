"""Codex review of the step 9 fixes (navpy_recovery.md, step 9): each test fails without its fix."""
from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.core.approach.dock_contact_episode import FinalApproachDockContact
from navpy.core.approach.dock_episode_end import HELD_REASON, dock_episode_end
from navpy.core.approach.nav_exit_cleanup import NavExitCleanup, NavExitCleanupPorts
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeLatch
from navpy.core.flow.navigation_decision import ApproachOutcome
from navpy.core.ports.flight_mode import FlightMode
from navpy.sim.world.closest_approach_tracker import _outcome
from navpy.sim.world.closest_approach_record import OUTCOME_ABORTED, OUTCOME_MISSED
from navpy.core.flow.nav_mode_request import MODE_CONFIRM_S
from navpy.core.flow.nav_recovery import NAV_HOLD_RECOVERY_S, NAV_HOLD_TRIGGER_S, RecoveryStage
from tests.modules.nav.test_held_approach import _rig
from tests.modules.nav.test_nav_recovery import PERIOD_S, FakeAutopilot, Rig


# ---------- finding 1: the DDH transit's GUIDED is NavPy's ----------


def test_r1_the_ddh_transit_asks_for_guided_through_recovery():
    from navpy.products.delivery.fallback_mission_navigation import FallbackMissionNavigation, FallbackMissionPorts

    ports = FallbackMissionPorts(
        next_waypoint=lambda: 9, mission_item_count=lambda: 10, current_relative=lambda: None,
        set_sim_poi=Mock(), detector_is_simulation=lambda: False, request_guided=Mock(),
    )
    planner = Mock(plan_orbit_approach=Mock(return_value=(Mock(orbit_radius=0.0, offset_distance=0.0), 50.0)))
    commands = Mock()
    FallbackMissionNavigation(ports, Mock(), Mock(), planner, commands, Mock()).setup()
    ports.request_guided.assert_called_once()
    commands.request_guided.assert_not_called()


# ---------- finding 2: a fresh episode only on recovery's resume ----------


def test_r2_a_hold_before_guided_then_the_operators_loiter_is_not_resumed_over():
    """The hold began before GUIDED was accepted; the operator selects LOITER; recovery reopens without resuming.
    NavPy must not ask for GUIDED again over the operator's LOITER."""
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()                                         # NAV entry asks for GUIDED: not shown yet
    assert decision.advance(FlightMode.AUTO) is ApproachOutcome.WAITING
    holding[0] = True
    decision.advance(FlightMode.AUTO)                          # hold: the episode ends
    holding[0] = False                                         # healthy, no resume grant
    asked = len(requests)
    assert decision.advance(FlightMode.LOITER) is ApproachOutcome.ABANDONED
    assert len(requests) == asked and approach.inits == 1     # no fresh episode, no GUIDED over the LOITER


# ---------- finding 3: a fresh entry that raises is retried under the same grant ----------


def test_r3_a_fresh_entry_that_raises_is_retried_until_it_succeeds():
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()
    decision.advance(FlightMode.GUIDED)
    holding[0] = True
    decision.advance(FlightMode.LOITER)
    holding[0], resumed[0] = False, True
    approach.failing_inits = 1
    with pytest.raises(RuntimeError, match="init failed"):
        decision.advance(FlightMode.AUTO)
    assert held.ended and resumed[0]                          # still held; the grant is not used up
    assert decision.advance(FlightMode.AUTO) is ApproachOutcome.WAITING   # retried at the next decision
    assert approach.inits == 3 and contact.arm.call_count == 2   # first entry, the failed one, the retry
    assert not held.ended and not resumed[0]


def test_r3_a_retry_yields_to_the_operators_mode():
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()
    decision.advance(FlightMode.GUIDED)
    holding[0] = True
    decision.advance(FlightMode.LOITER)
    holding[0], resumed[0] = False, True
    approach.failing_inits = 1
    with pytest.raises(RuntimeError):
        decision.advance(FlightMode.AUTO)
    assert decision.advance(FlightMode.FBWA) is ApproachOutcome.ABANDONED
    assert approach.inits == 2                                 # no retry over the operator's FBWA


# ---------- Codex round 2 on the step 9 fixes ----------


def _cycle(recovery, decision, *, decide: bool = True) -> bool:
    """One NAV period: the decision (a raise fails the cycle, as NavigationLoop._run_cycle does), then recovery.
    ``decide`` False: the cycle fails before its decision (the injected fault)."""
    ok = decide
    if decide:
        try:
            decision.advance(recovery.ap.mode)
        except Exception:  # noqa: BLE001 - a failed cycle
            ok = False
    recovery.run(ok, PERIOD_S)
    return ok


def test_r2_1_a_grant_does_not_outlive_its_recovery_episode():
    """Recovery resumes; the fresh entry raises; a new hold begins; the operator selects AUTO in it; recovery
    reopens without resuming. NavPy must not take GUIDED back over the operator's AUTO."""
    recovery = Rig(FakeAutopilot())
    approach, task, failures, held, _h, _r, requests, contact, outcome, transition, decision = _rig(recovery)
    transition.enter()
    recovery.run(True, 1.0)
    for _ in range(int((NAV_HOLD_TRIGGER_S + 0.25) / PERIOD_S)):
        _cycle(recovery, decision, decide=False)               # the fault: cycles fail before deciding
    assert recovery.stage is RecoveryStage.HOLD
    while not recovery.hold.resumed:
        assert _cycle(recovery, decision)                      # healthy in the hold: the episode ends; then AUTO
    approach.failing_inits = 10 ** 6                           # the fresh entry raises from now on
    assert not _cycle(recovery, decision)
    assert recovery.hold.resumed                               # the grant waits for a successful entry...
    while recovery.stage is RecoveryStage.NORMAL:
        _cycle(recovery, decision)
    assert not recovery.hold.resumed                           # ...but a new hold voids it
    approach.failing_inits = 0
    while recovery.ap.mode is not FlightMode.LOITER:
        _cycle(recovery, decision)                             # the new hold's LOITER shows
    recovery.ap.mode = FlightMode.AUTO                         # the operator's AUTO, in the new hold
    recovery.ap.heartbeat()
    for _ in range(200):
        if recovery.stage is RecoveryStage.NORMAL:
            break
        _cycle(recovery, decision)
    assert recovery.stage is RecoveryStage.NORMAL and not recovery.hold.resumed
    asked = len(requests)
    assert decision.advance(FlightMode.AUTO) is ApproachOutcome.ABANDONED
    assert len(requests) == asked                              # no GUIDED over the operator's AUTO


def test_r2_2_a_failed_abandonment_is_retried_not_turned_into_guided_acquisition():
    """The operator's LOITER ends the hold; clearing the task raises once. The next cycle must abandon again, not
    fall through to ordinary GUIDED acquisition over the LOITER."""
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()
    decision.advance(FlightMode.GUIDED)
    holding[0] = True
    decision.advance(FlightMode.LOITER)
    holding[0] = False
    clear = decision._ports.clear_navigation_task
    clear.side_effect = [RuntimeError("reset failed"), None]
    task.nav_mode_observed = False                             # what the failed reset transaction left
    with pytest.raises(RuntimeError, match="reset failed"):
        decision.advance(FlightMode.LOITER)
    asked = len(requests)
    assert decision.advance(FlightMode.LOITER) is ApproachOutcome.ABANDONED
    assert len(requests) == asked and held.ended               # held until NAV exits


def test_r2_3_an_entry_that_always_raises_reaches_rtl_in_bounded_aircraft_time():
    """Each resume's fresh entry raises (after its GUIDED request): HOLD, resume, fail, hold again. One incident
    keeps the first hold's deadline (NAV_INCIDENT_CLEAR_S), so RTL comes NAV_HOLD_RECOVERY_S after the first hold,
    plus at most one resume and hold trigger."""
    recovery = Rig(FakeAutopilot())
    approach, task, failures, held, _h, _r, requests, contact, outcome, transition, decision = _rig(recovery)
    transition.enter()
    recovery.run(True, 1.0)
    for _ in range(int((NAV_HOLD_TRIGGER_S + 0.25) / PERIOD_S)):
        _cycle(recovery, decision, decide=False)
    assert recovery.stage is RecoveryStage.HOLD
    first_hold_s = recovery.t
    approach.failing_inits = 10 ** 6
    holds = 0
    stage = recovery.stage
    while recovery.ap.mode is not FlightMode.RTL:
        assert recovery.t - first_hold_s <= NAV_HOLD_RECOVERY_S + 3 * NAV_HOLD_TRIGGER_S + MODE_CONFIRM_S, (
            f"no RTL {recovery.t - first_hold_s:.1f} s after the first hold ({holds} re-holds)")
        _cycle(recovery, decision)
        holds += stage is not RecoveryStage.HOLD and recovery.stage is RecoveryStage.HOLD
        stage = recovery.stage
    assert holds >= 2                                          # it did cycle through resumes and re-holds
    assert recovery.t - first_hold_s >= NAV_HOLD_RECOVERY_S


def test_r2_4_a_fresh_entry_that_raises_after_arming_ends_its_partial_episode():
    """NAV entry raises after arming dock contact (its last step): the partial episode is ended (held), so the retry
    arms again only after a matching end."""
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()
    decision.advance(FlightMode.GUIDED)
    holding[0] = True
    decision.advance(FlightMode.LOITER)
    holding[0], resumed[0] = False, True
    events: list[str] = []
    contact.arm.side_effect = lambda: (events.append("arm"), approach.episode.begin(Mock()))
    contact.disarm.side_effect = lambda: events.append("end")
    entry_logger = transition._entry._logger
    entry_logger.defer_status_texts.side_effect = [RuntimeError("status deferral failed"), None]
    with pytest.raises(RuntimeError, match="status deferral failed"):
        decision.advance(FlightMode.AUTO)
    assert decision.advance(FlightMode.AUTO) is ApproachOutcome.WAITING
    assert events == ["arm", "end", "arm"]                     # the partial episode ended before the re-arm


# ---------- finding 4: the held episode is never reported as a miss ----------


def test_r4_the_held_episode_is_reported_aborted_for_the_hold_never_missed(monkeypatch):
    """The real dock contact, episode end and terminal latch; the sensor scores what it receives with the sim
    record's own outcome rule, on a pass that went by the dock."""
    import navpy.core.approach.dock_contact_episode as episode_module

    monkeypatch.setattr(episode_module, "get_poi_task_id", lambda _poi: 3)
    latch = FinalApproachEpisodeLatch()
    received = []
    sensor = Mock(disarm=Mock(side_effect=lambda end: received.append(_outcome(end, passed=True))))
    contact = FinalApproachDockContact(
        sensor, lambda: 1, lambda identity, source_s: latch.begin(identity) or True, Mock(), Mock(),
        ending=lambda identity, docked: dock_episode_end(identity, docked, latch.terminal_snapshot(), False),
    )
    poi = Mock()
    poi.identity.obj_id = 7
    cleanup = NavExitCleanup(
        NavExitCleanupPorts(
            disarm_dock_contact=contact.disarm, close_final_approach_source=Mock(), navigation_reset=Mock(),
            detector_stop=Mock(), disarm_held_dock_contact=contact.disarm_held,
        ),
        Mock(), Mock(),
    )

    contact.arm(poi)
    received.clear()
    latch.latch_failure("source_command_stale")             # the fenced worker's stale command, in the outage
    cleanup.end_episode()
    assert received == [(OUTCOME_ABORTED, HELD_REASON)]

    # The ordinary disarm of the same latched episode publishes a miss: what the hold must not do.
    contact.arm(poi)
    received.clear()
    latch.latch_failure("source_command_stale")
    contact.disarm()
    assert received == [(OUTCOME_MISSED, None)]


# ---------- the fault hook, composed ----------


def test_the_composed_nav_application_installs_the_fault_hook_only_with_the_flag(tmp_path):
    """A guard, not a regression test: it passes on the commit that added the hook too, and fails if the
    composition stops installing the wrapper (Codex review of the step 9 fixes)."""
    from navpy.sim.cycle_fault import FaultingCycle
    from tests.modules.nav.test_nav_controller import (
        _create_mock_args,
        _create_mock_detector,
        _create_mock_navigation,
        _create_mock_vehicle,
    )
    from navpy.app.flow.nav_composition import create_nav_application
    from navpy.core.skills.acquire.approach_strategy import ApproachKind
    from tests.modules.nav.nav_test_rig import as_detection_coordination

    def loop_cycle(fault_file):
        vehicle = _create_mock_vehicle()
        vehicle.is_simulated_autopilot = Mock(return_value=True)
        args = _create_mock_args()
        args.sim_cycle_fault_file = fault_file
        application = create_nav_application(
            vehicle, as_detection_coordination(_create_mock_detector()), _create_mock_navigation(), args, Mock(),
            ApproachKind.OFFSET, None, None,
        )
        return application._loop._cycle

    assert isinstance(loop_cycle(str(tmp_path / "fault")), FaultingCycle)
    assert not isinstance(loop_cycle(None), FaultingCycle)


def test_r2_3_an_incident_ends_after_a_healthy_spell_as_long_as_the_hold_budget():
    """A new fault long after a real recovery is a new incident, with a fresh deadline."""
    from navpy.core.flow.nav_recovery_incident import NAV_INCIDENT_CLEAR_S, RecoveryIncident

    incident = RecoveryIncident(30.0)
    incident.hold(100.0)
    incident.reopened()
    incident.hold(110.0)                                       # a re-hold: the first hold's deadline
    assert not incident.expired(129.9) and incident.expired(130.0)
    incident.reopened()
    incident.healthy(140.0, 140.0 + NAV_INCIDENT_CLEAR_S - 0.1)
    incident.hold(175.0)
    assert incident.expired(175.0)                             # not yet healthy long enough: same incident
    incident.reopened()
    incident.healthy(176.0, 176.0 + NAV_INCIDENT_CLEAR_S)       # healthy as long as the hold budget
    incident.hold(300.0)
    assert not incident.expired(329.9) and incident.expired(330.0)
