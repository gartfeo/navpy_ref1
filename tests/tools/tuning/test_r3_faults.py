"""Tests for sitl_faults' fault axis (--axis) and its height floor, without SITL.

Round 2: six of the seven cases ran on roll, which never reached a check run with new gains on
Peron15_hitl2 ('model does not fit'), so they could not inject their faults; the same six on pitch passed
(scratch faults_pitch.py: FAULTS on roll moved to pitch, "pitch: switch off" left out as a repeat).
"""
import json
import sys

import pytest

from tests.tools.tuning.r3_fake_plane import FakePlane
from tools.tuning import sitl_faults
from tools.tuning.sitl_faults import AXIS_BIT, FAULTS, Fault, cases

ROUND2_PITCH_RUN = ["switch off", "mode change", "roll setting", "stick", "second new-gain run: mode change",
                    "AUTOTUNE over MAVLink"]


def test_without_an_axis_every_case_keeps_its_own():
    assert cases(None) == FAULTS
    assert [f.axis for f in cases(None)] == ["roll"] * 4 + ["pitch"] + ["roll"] * 2


def test_on_pitch_the_cases_are_the_six_that_passed_in_round_2():
    moved = cases("pitch")
    assert [f.name for f in moved] == ROUND2_PITCH_RUN and {f.axis for f in moved} == {"pitch"}
    originals = {f.name: f for f in FAULTS}
    assert all((f.expect, f.b_run, f.tuner_owns_after) == (originals[f.name].expect, originals[f.name].b_run,
                                                           originals[f.name].tuner_owns_after) for f in moved)
    assert moved[-1].name == "AUTOTUNE over MAVLink"       # AUTOTUNE may change IMAX and SMAX: still last


def test_on_roll_the_pitch_only_case_is_a_repeat_and_left_out():
    moved = cases("roll")
    assert [f.name for f in moved] == ROUND2_PITCH_RUN and {f.axis for f in moved} == {"roll"}


def test_a_case_tunes_its_axis_and_the_result_says_which(monkeypatch):
    plane = FakePlane(height=118, mode="LOITER")
    monkeypatch.setattr(sitl_faults, "wait_candidate", lambda *args: False)
    result = sitl_faults.run_fault(plane, cases("pitch")[0])
    assert plane.params["FTUN_AXES"] == AXIS_BIT["pitch"]
    assert result == {"fault": "switch off", "axis": "pitch", "pass": False, "why": "no check run with new gains"}


def _run_main(monkeypatch, tmp_path, plane: FakePlane, *extra: str) -> tuple[int, list[tuple[str, str]]]:
    flown: list[tuple[str, str]] = []

    def run_fault(sitl: object, fault: Fault) -> dict:
        flown.append((fault.name, fault.axis))
        plane.fly_for(10)
        return {"fault": fault.name, "axis": fault.axis, "pass": True}
    monkeypatch.setattr(sitl_faults, "open_vehicle", lambda *args: plane)
    monkeypatch.setattr(sitl_faults, "takeoff_to_fbwa", lambda sitl: plane.set_mode("LOITER"))
    monkeypatch.setattr(sitl_faults, "run_fault", run_fault)
    monkeypatch.setattr(sys, "argv", ["sitl_faults", "--workdir", str(tmp_path / "faults"), *extra])
    with pytest.raises(SystemExit) as stop:
        sitl_faults.main()
    return stop.value.code, flown


def test_the_axis_option_flies_every_case_on_that_axis(monkeypatch, tmp_path):
    code, flown = _run_main(monkeypatch, tmp_path, FakePlane(height=118), "--axis", "pitch")
    assert code == 0 and flown == [(name, "pitch") for name in ROUND2_PITCH_RUN]
    assert json.loads((tmp_path / "faults.json").read_text())["axis"] == "pitch"
    code, flown = _run_main(monkeypatch, tmp_path, FakePlane(height=118))
    assert code == 0 and flown == [(f.name, f.axis) for f in FAULTS]


def test_a_fault_run_that_loses_its_height_fails(monkeypatch, tmp_path):
    plane = FakePlane(height=118, loiter_follows_target=False, loiter_sink=5)
    code, flown = _run_main(monkeypatch, tmp_path, plane, "--axis", "pitch")
    assert code == 1 and len(flown) == 2               # 118 m - 2 x 10 s x 5 m/s: below 30 m in the second case
    assert json.loads((tmp_path / "faults.json").read_text())["height"]["breach"]
