"""The confirm record that releases the HITL recovery floor (HITL step 23b, option A).

navpy prints ``NAVPY_CONFIRM_V1 {"task": N}`` once, when a task's confirm releases NAV (step 25's gates holding on
that tick); a confirmation the gates revoke before NAV releases nothing. It is a mission-state event: no altitude read.
"""
from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.app.flow.confirm_record import NAVPY_CONFIRM_MARKER, parse_confirm_record, print_confirm_record
from navpy.core.flow.nav_state import NavPhaseState, NavState
from navpy.core.skills.confirm.acquisition_sequence import OPEN_ACQUISITION, AcquisitionSequence
from navpy.core.skills.confirm.confirmed_poi_release import ConfirmedPoiRelease, ReleaseOutcome
from tests.detection_factory import make_detected_poi
from tests.modules.nav.test_approach_acquisition import _drive
from tests.modules.nav.acquisition_replay import AcquisitionRig


def test_the_record_round_trips_and_malformed_lines_are_not_records(capsys):
    print_confirm_record(4)
    line = capsys.readouterr().out.strip()
    assert line == f'{NAVPY_CONFIRM_MARKER} {{"task":4}}'
    assert parse_confirm_record(line) == {"task": 4}
    for bad in ("", "NAVPY_CONFIRM_V1", 'NAVPY_CONFIRM_V1 {"task":true}', "NAVPY_CONFIRM_V1 [1]",
                'NAVPY_TERMINAL_V1 {"task":4}', 'NAVPY_CONFIRM_V1 {"task":"4"}', None):
        assert parse_confirm_record(bad) is None


def _enter(sequence) -> None:
    """One NAV entry attempt that completes."""
    sequence.entry_started()
    sequence.entered()


def _sequence(released: list):
    return AcquisitionSequence(Mock(), Mock(), Mock(), Mock(), lambda *_a: None, released.append)


def test_the_sequence_reports_one_release_per_task_only_its_own_and_only_after_nav_entry():
    released = []
    sequence = _sequence(released)
    sequence.released(make_detected_poi(task_id=3))                 # no task begun: nothing
    _enter(sequence)
    sequence.begin(make_detected_poi(task_id=3))
    _enter(sequence)                                              # an entry without a confirm: nothing
    sequence.released(make_detected_poi(task_id=9))                 # another task's POI: nothing
    _enter(sequence)
    assert released == []
    sequence.released(make_detected_poi(task_id=3))
    assert released == []                                           # the confirm alone releases nothing
    _enter(sequence)
    sequence.released(make_detected_poi(task_id=3))
    _enter(sequence)
    assert released == [3]                                          # once per task
    sequence.begin(make_detected_poi(task_id=5))
    sequence.released(make_detected_poi(task_id=5))
    _enter(sequence)
    assert released == [3, 5]
    OPEN_ACQUISITION.released(make_detected_poi())                  # no gates: nothing to report
    OPEN_ACQUISITION.entry_started()
    OPEN_ACQUISITION.entered()


def test_a_nav_entry_that_fails_reports_nothing_and_one_that_completes_reports_once():
    """Codex round 1: the floor is released only after NAV entry completes, not when the confirm is seen."""
    from navpy.core.approach.nav_entry import NavEntry, NavEntryPorts

    told = []
    zoom = Mock()
    zoom.freeze_final_approach_wide.return_value = False            # the minimum zoom cannot be held
    entry = NavEntry(NavEntryPorts(Mock(), Mock(), lambda: True, Mock(), Mock(), lambda: False,
                                   entry_started=lambda: told.append("start"),
                                   entered=lambda: told.append(1)), Mock(), zoom, Mock())
    with pytest.raises(RuntimeError, match="minimum zoom"):
        entry.run()
    assert told == ["start"]                                        # the attempt started, never completed
    zoom.freeze_final_approach_wide.return_value = True
    entry.run()
    assert told == ["start", "start", 1]


@pytest.mark.parametrize("current,previous,ready,outcome,told", [
    (NavState.CONFIRM, NavState.CONFIRM, True, ReleaseOutcome.READY, 1),
    (NavState.CONFIRM, NavState.CONFIRM, False, ReleaseOutcome.WAIT, 0),
    (NavState.NAV, NavState.NAV, False, ReleaseOutcome.READY, 0),         # NAV committed: no gates, no release
    # Codex round 2: a NAV entry that failed leaves current at NAV, previous at CONFIRM; the retry needs the gates.
    (NavState.NAV, NavState.CONFIRM, False, ReleaseOutcome.WAIT, 0),
    (NavState.NAV, NavState.CONFIRM, True, ReleaseOutcome.READY, 1),
    # Codex round 3: a NAV exit that failed leaves previous at NAV with current moved on; the gates apply.
    (NavState.DETECT, NavState.NAV, False, ReleaseOutcome.WAIT, 0),
    (NavState.RESET, NavState.NAV, True, ReleaseOutcome.READY, 1),
])
def test_the_release_is_told_only_when_a_confirm_releases_an_uncommitted_nav(current, previous, ready, outcome, told):
    seen = []
    gate = Mock()
    gate.is_ready.return_value = True
    geo_hold = Mock()
    geo_hold.active = False
    phase = NavPhaseState(current, previous)
    release = ConfirmedPoiRelease(phase, geo_hold, Mock(), Mock(), gate, lambda: False, lambda: ready, seen.append)
    assert release.apply(make_detected_poi(task_id=2)) is outcome
    assert len(seen) == told
    if outcome is ReleaseOutcome.WAIT:
        assert phase.current is NavState.CONFIRM


def test_a_failed_entry_attempt_uses_the_mark_and_only_a_new_confirm_release_reports():
    """Codex rounds 3-4: a retried or held entry (no confirm release on its tick) leaves the floor armed; the
    authority is the attempt, not elapsed time."""
    released = []
    sequence = _sequence(released)
    sequence.begin(make_detected_poi(task_id=3))
    sequence.released(make_detected_poi(task_id=3))
    sequence.entry_started()                                        # this attempt fails: entered() never comes
    _enter(sequence)                                                # a held retry: no confirm on its tick
    assert released == []
    sequence.released(make_detected_poi(task_id=3))                 # a new confirm release passed the gates
    _enter(sequence)
    assert released == [3]


def test_a_slow_first_entry_still_reports_its_release():
    """Codex round 4: no deadline; however long the completing attempt takes, its release counts."""
    released = []
    sequence = _sequence(released)
    sequence.begin(make_detected_poi(task_id=3))
    sequence.released(make_detected_poi(task_id=3))
    sequence.entry_started()
    sequence.ready(None)                                            # nothing else runs on the tick before entered
    sequence.entered()
    assert released == [3]


def test_a_tick_without_the_gates_drops_a_pending_release():
    """Codex round 2: a confirm whose NAV entry failed must not release the floor at a later entry without a new
    confirm release on a tick where the gates hold."""
    released = []
    sequence = _sequence(released)
    sequence.begin(make_detected_poi(task_id=3))
    sequence.released(make_detected_poi(task_id=3))
    sequence._lock.locked.return_value = False                     # the lock is lost before the retry
    assert sequence.ready(make_detected_poi(task_id=3)) is False
    _enter(sequence)
    assert released == []


def test_through_the_real_graph_the_record_is_printed_once_when_nav_starts(capsys):
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 60.0) == pytest.approx(12.3)
    records = [parse_confirm_record(line) for line in capsys.readouterr().out.splitlines()]
    assert [r for r in records if r is not None] == [{"task": 1}]          # the frames' task (obj_id 1)


def test_through_the_real_graph_no_lock_prints_no_record(capsys):
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 30.0, error=1.5) is None
    assert not any(parse_confirm_record(line) for line in capsys.readouterr().out.splitlines())
