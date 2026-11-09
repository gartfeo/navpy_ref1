"""NavPy recovery step 10 (docs/design/navpy_recovery.md): the step 9 follow-ups the coordinator cleared."""
from __future__ import annotations

from unittest.mock import Mock

from navpy.core.approach.dock_contact_episode import FinalApproachDockContact
from navpy.core.approach.dock_episode_end import HELD_REASON, dock_episode_end
from navpy.core.approach.nav_entry import NavEntry, NavEntryPorts
from navpy.core.approach.nav_exit_cleanup import NavExitCleanup, NavExitCleanupPorts
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeLatch
from navpy.core.flow.nav_held_approach import HeldApproach
from navpy.core.flow.nav_state import NavigationTaskState, NavState
from navpy.core.flow.nav_transition import NavTransition
from navpy.sim.world.closest_approach_record import OUTCOME_ABORTED, OUTCOME_MISSED
from navpy.sim.world.closest_approach_tracker import _outcome


def _held_exit_rig(monkeypatch, holding: bool):
    """A real dock contact, terminal latch and NAV transition; the sensor scores what it receives with the sim
    record's own outcome rule, on a pass that went by the dock."""
    import navpy.core.approach.dock_contact_episode as episode_module

    monkeypatch.setattr(episode_module, "get_poi_task_id", lambda _poi: 3)
    latch = FinalApproachEpisodeLatch()
    received = []
    armed = []

    def disarm(end) -> None:
        # As the sim sensor (dock_contact_sensor.py): an episode reports once, and only while one is armed.
        if armed:
            armed.clear()
            received.append(_outcome(end, passed=True))

    sensor = Mock(arm=Mock(side_effect=lambda identity, on_docked: armed.append(identity)),
                  disarm=Mock(side_effect=disarm))
    contact = FinalApproachDockContact(
        sensor, lambda: 1, lambda identity, source_s: latch.begin(identity) or True, Mock(), Mock(),
        ending=lambda identity, docked: dock_episode_end(identity, docked, latch.terminal_snapshot(), False),
    )
    poi = Mock()
    poi.identity.obj_id = 7
    outcome = Mock(terminal=Mock(side_effect=latch.terminal_snapshot), record_failed=Mock())
    task = NavigationTaskState()

    def reset() -> None:
        latch.reset()

    transition = NavTransition(
        NavEntry(
            NavEntryPorts(
                request_guided=Mock(), navigation_init=reset, final_approach_active=lambda: False,
                arm_dock_contact=lambda: contact.arm(poi), stop_geo_tracking=Mock(), is_geo_armed=lambda: False,
            ),
            task, Mock(), Mock(),
        ),
        NavExitCleanup(
            NavExitCleanupPorts(
                disarm_dock_contact=contact.disarm, close_final_approach_source=Mock(), navigation_reset=reset,
                detector_stop=Mock(), dock_outcome=outcome, disarm_held_dock_contact=contact.disarm_held,
            ),
            Mock(restore=Mock(return_value=True)), Mock(),
        ),
        Mock(), Mock(return_value=()), task, lambda: True, logger=Mock(), held=HeldApproach(),
        holding=lambda: holding,
    )
    return transition, latch, received, outcome, task


def test_item1_an_operator_mode_in_the_hold_ends_the_episode_as_held(monkeypatch):
    """Step 9 SITL: the operator's FBWA in the hold ended the task through NAV's inactive-mode guard before the
    decision ran, and the sim record showed ``aborted reason=nav_fail`` from the failure latched while fenced."""
    transition, latch, received, outcome, task = _held_exit_rig(monkeypatch, holding=True)
    transition.enter()
    task.final_approach_navigation_active = True
    received.clear()
    latch.latch_failure("source_command_stale")             # the fenced worker's stale command, in the hold
    transition.exit(NavState.ONHOLD)                           # the inactive-mode guard: FBWA
    assert received == [(OUTCOME_ABORTED, HELD_REASON)]
    outcome.record_failed.assert_not_called()


def test_item1_outside_a_hold_the_nav_exit_reports_as_before(monkeypatch):
    transition, latch, received, outcome, task = _held_exit_rig(monkeypatch, holding=False)
    transition.enter()
    received.clear()
    latch.latch_failure("source_command_stale")
    transition.exit(NavState.ONHOLD)
    assert received == [(OUTCOME_MISSED, None)]                # NAV's own failure after passing the dock


def test_item11_the_held_episode_end_is_a_recovery_report(monkeypatch):
    """Step 9 SITL: "NAV approach held" reached the NavPy log only; it is journaled like every recovery report."""
    from tests.logger.recording_logger import RecordingLogger

    transition, latch, received, outcome, task = _held_exit_rig(monkeypatch, holding=True)
    logger = RecordingLogger()
    transition._logger = logger
    transition.enter()
    transition._held.end_episode()
    assert logger.journal.records == [("nav", "NAV approach held: episode ended, a fresh one follows the resume")]
