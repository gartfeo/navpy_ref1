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


def _sequence(released: list):
    return AcquisitionSequence(Mock(), Mock(), Mock(), Mock(), lambda *_a: None, released.append)


def test_the_sequence_reports_one_release_per_task_and_only_its_own():
    released = []
    sequence = _sequence(released)
    sequence.released(make_detected_poi(task_id=3))                 # no task begun: nothing
    sequence.begin(make_detected_poi(task_id=3))
    sequence.released(make_detected_poi(task_id=9))                 # another task's POI: nothing
    assert released == []
    sequence.released(make_detected_poi(task_id=3))
    sequence.released(make_detected_poi(task_id=3))
    assert released == [3]
    sequence.begin(make_detected_poi(task_id=5))
    sequence.released(make_detected_poi(task_id=5))
    assert released == [3, 5]
    OPEN_ACQUISITION.released(make_detected_poi())                  # no gates: nothing to report


@pytest.mark.parametrize("phase,ready,expected", [
    (NavState.CONFIRM, True, 1), (NavState.CONFIRM, False, 0), (NavState.NAV, True, 0),
])
def test_the_release_is_told_only_when_a_confirm_releases_nav(phase, ready, expected):
    told = []
    gate = Mock()
    gate.is_ready.return_value = True
    geo_hold = Mock()
    geo_hold.active = False
    release = ConfirmedPoiRelease(NavPhaseState(phase), geo_hold, Mock(), Mock(), gate, lambda: False,
                                  lambda: ready, told.append)
    outcome = release.apply(make_detected_poi(task_id=2))
    assert outcome is (ReleaseOutcome.READY if ready or phase is NavState.NAV else ReleaseOutcome.WAIT)
    assert len(told) == expected


def test_through_the_real_graph_the_record_is_printed_once_when_nav_starts(capsys):
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 60.0) == pytest.approx(12.3)
    records = [parse_confirm_record(line) for line in capsys.readouterr().out.splitlines()]
    assert [r for r in records if r is not None] == [{"task": 1}]          # the frames' task (obj_id 1)


def test_through_the_real_graph_no_lock_prints_no_record(capsys):
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 30.0, error=1.5) is None
    assert not any(parse_confirm_record(line) for line in capsys.readouterr().out.splitlines())
