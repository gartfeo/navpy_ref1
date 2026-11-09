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
from tests.modules.nav.test_held_approach import _rig


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
    holding[0] = False                                         # healthy, not resumed: NavPy did not own the mode
    asked = len(requests)
    assert decision.advance(FlightMode.LOITER) is ApproachOutcome.ABANDONED
    assert len(requests) == asked and approach.inits == 1     # no fresh episode, no GUIDED over the LOITER
    assert not held.ended and not held.resume_pending


# ---------- finding 3: a fresh entry that raises is retried ----------


def test_r3_a_fresh_entry_that_raises_is_retried_until_it_succeeds():
    approach, task, failures, held, holding, resumed, requests, contact, outcome, transition, decision = _rig()
    transition.enter()
    decision.advance(FlightMode.GUIDED)
    holding[0] = True
    decision.advance(FlightMode.LOITER)
    holding[0], resumed[0] = False, True
    approach.failing_inits = 1
    with pytest.raises(RuntimeError, match="init failed"):
        decision.advance(FlightMode.AUTO)                      # the resume's entry raises; the resume is consumed
    assert held.ended and held.resume_pending                 # still held, still owed a fresh entry
    assert decision.advance(FlightMode.AUTO) is ApproachOutcome.WAITING   # retried at the next decision
    assert approach.inits == 3 and contact.arm.call_count == 2   # first entry, the failed one, the retry
    assert not held.ended and not held.resume_pending


def test_r3_a_pending_retry_yields_to_the_operators_mode():
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
    assert approach.inits == 2 and not held.resume_pending   # no retry over the operator's FBWA


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
