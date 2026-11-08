"""Evals honest (navpy_recovery.md, step 8): the evaluator sets the verdict, clean, recovered, invalid or failed,
from NavPy's recovery journal, apart from the evidence writer."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from scripts import eval_gcs_navigation_demo as evaluator
from scripts.eval_gcs_demo_recovery import (
    CLEAN,
    FAILED,
    INVALID,
    RECOVERED,
    faults_planned,
    recovery_evidence,
    run_verdict,
    scored_with,
    unexpected_recovery_errors,
)
from tests.scripts.test_eval_gcs_navigation_demo import (
    CERTIFIED,
    RECOVERY_JOURNAL_OPENED,
    _acked_run,
    _approval_records,
    _log_assignment,
)

HOLD = {"seq": 1, "t": 12.0, "source": "nav", "message": "NavPy hold: commands fenced, LOITER"}
RESUME = {"seq": 2, "t": 15.0, "source": "nav", "message": "NavPy healthy again: AUTO"}


def _journal(tmp_path: Path, sys_id: int, *entries: dict) -> Path:
    path = tmp_path / f"uav_{sys_id}_recovery.jsonl"
    path.write_text(RECOVERY_JOURNAL_OPENED + "".join(json.dumps(e) + "\n" for e in entries), encoding="utf-8")
    return path


def _analyzed(tmp_path: Path):
    _log_assignment(tmp_path, 5, "Task 2 assigned by owner 4")
    _log_assignment(tmp_path, 6, "Task 3 assigned by owner 4")
    return evaluator.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6), approvals=_approval_records())


# ---------- the journal ----------


def test_a_journal_s_recoveries_are_read_across_navpy_restarts(tmp_path):
    path = _journal(tmp_path, 1, HOLD)
    path.write_text(path.read_text(encoding="utf-8") + RECOVERY_JOURNAL_OPENED + json.dumps(RESUME) + "\n",
                    encoding="utf-8")
    evidence = recovery_evidence(tmp_path, 1, "")
    assert evidence.errors == ()
    assert [(e["source"], e["message"]) for e in evidence.events] == [
        ("nav", HOLD["message"]), ("nav", RESUME["message"]),
    ]


def test_a_missing_unreadable_or_incomplete_journal_is_an_evidence_error(tmp_path):
    assert recovery_evidence(tmp_path, 1, "").errors == ("missing uav_1_recovery.jsonl",)
    (tmp_path / "uav_2_recovery.jsonl").write_text('{"event": "open"}\nnot json\n{"x": 1}\n', encoding="utf-8")
    assert [e.split(": ", 1)[1] for e in recovery_evidence(tmp_path, 2, "").errors] == [
        "line 2 is not JSON", "line 3 has no source and message",
    ]
    (tmp_path / "uav_3_recovery.jsonl").write_text(json.dumps(HOLD) + "\n", encoding="utf-8")
    assert recovery_evidence(tmp_path, 3, "").errors == ("recovery journal uav_3_recovery.jsonl: no open line",)
    _journal(tmp_path, 4)
    marked = recovery_evidence(tmp_path, 4, "x ERROR RECOVERY_JOURNAL_FAILURE: OSError: disk full\n")
    assert marked.errors == ("recovery journal incomplete, not scored (RECOVERY_JOURNAL_FAILURE logged)",)


# ---------- the verdict, end to end on a certified run ----------


def test_a_run_that_recovered_nothing_is_clean(tmp_path):
    _acked_run(tmp_path)
    report = _analyzed(tmp_path)
    assert report.passed is True and report.verdict == CLEAN
    assert {vehicle.verdict for vehicle in report.vehicles} == {CLEAN}


def test_a_recovery_in_a_run_without_scheduled_faults_fails_the_clean_run_gate(tmp_path):
    _acked_run(tmp_path)
    _journal(tmp_path, 5, HOLD, RESUME)
    report = _analyzed(tmp_path)
    assert report.passed is False and report.verdict == FAILED
    assert report.errors == [
        "uav 5: unexpected recovery in a clean run: nav: NavPy hold: commands fenced, LOITER; "
        "nav: NavPy healthy again: AUTO",
    ]
    vehicle = next(v for v in report.vehicles if v.sys_id == 5)
    assert vehicle.verdict == RECOVERED and vehicle.passed is True
    assert [r["message"] for r in vehicle.recoveries] == [HOLD["message"], RESUME["message"]]


def test_a_missing_recovery_journal_makes_the_run_invalid_not_failed(tmp_path):
    _acked_run(tmp_path)
    (tmp_path / "uav_6_recovery.jsonl").unlink()
    report = _analyzed(tmp_path)
    assert report.passed is False and report.verdict == INVALID
    vehicle = next(v for v in report.vehicles if v.sys_id == 6)
    assert vehicle.verdict == INVALID and vehicle.errors == ["missing uav_6_recovery.jsonl"]


def test_an_evidence_writer_failure_makes_the_run_invalid(tmp_path):
    _acked_run(tmp_path)
    log = tmp_path / "uav_4_navigation.log"
    log.write_text(log.read_text(encoding="utf-8")
                   + "2026-07-10 12:00:04,000 ERROR NAVIGATION_EVIDENCE_FAILURE: navigation log write OSError: x\n",
                   encoding="utf-8")
    report = _analyzed(tmp_path)
    assert report.verdict == INVALID and report.passed is False


def test_a_product_failure_with_valid_evidence_is_failed(tmp_path):
    _acked_run(tmp_path)
    report = evaluator.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6), approvals=_approval_records())
    assert report.passed is False and report.verdict == FAILED      # the peers' assignments are not logged


# ---------- run level ----------


def _vehicle(verdict: str, *recoveries: dict) -> SimpleNamespace:
    return SimpleNamespace(sys_id=1, verdict=verdict, recoveries=list(recoveries))


def test_a_run_with_scheduled_faults_may_recover():
    vehicles = [_vehicle(RECOVERED, HOLD), _vehicle(CLEAN)]
    assert unexpected_recovery_errors(vehicles, faults_planned=True) == []
    assert run_verdict(vehicles, passed=True, faults_planned=True) == RECOVERED
    assert run_verdict([_vehicle(CLEAN)], passed=True, faults_planned=True) == RECOVERED   # the fault gates' recovery


def test_the_clean_run_gate_quotes_three_recoveries_and_counts_the_rest():
    errors = unexpected_recovery_errors([_vehicle(RECOVERED, *([HOLD] * 5))], faults_planned=False)
    assert errors[0].endswith("(+2 more)") and errors[0].count("nav: ") == 3


def test_invalid_outranks_failed_and_failed_outranks_recovered():
    assert run_verdict([_vehicle(INVALID), _vehicle(FAILED)], passed=False, faults_planned=False) == INVALID
    assert run_verdict([_vehicle(CLEAN)], passed=False, faults_planned=True) == FAILED


def test_lockstep_errors_beside_the_analysis_fail_a_scored_run_but_keep_invalid():
    assert scored_with(CLEAN, ["lockstep outcome 'timeout', not 'ok'"]) == FAILED
    assert scored_with(RECOVERED, []) == RECOVERED
    assert scored_with(INVALID, ["operator: x"]) == INVALID
    assert scored_with("", ["operator: x"]) == ""


def test_faults_are_planned_only_by_a_lockstep_grid(tmp_path):
    assert faults_planned(tmp_path, "lockstep_journal.jsonl") is False
    journal = tmp_path / "lockstep_journal.jsonl"
    journal.write_text(json.dumps({"grid": {"faults": []}}) + "\n", encoding="utf-8")
    assert faults_planned(tmp_path, journal.name) is False
    journal.write_text(json.dumps({"grid": {"faults": [{"id": "link_drop:2"}]}}) + "\n", encoding="utf-8")
    assert faults_planned(tmp_path, journal.name) is True
