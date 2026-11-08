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
    RECOVERY_JOURNAL_OPEN,
    _acked_run,
    _approval_records,
    _log_assignment,
)

HOLD = {"seq": 1, "t": 12.0, "source": "nav", "message": "NavPy hold: commands fenced, LOITER"}
RESUME = {"seq": 2, "t": 15.0, "source": "nav", "message": "NavPy healthy again: AUTO"}


def _process(*entries: dict, closed: bool = True, recorded: int | None = None, dropped: int = 0) -> str:
    """One NavPy process's part of the journal, as RecoveryJournal writes it."""
    close = {"event": "close", "t": 30.0, "dropped": dropped,
             "recorded": len(entries) + dropped if recorded is None else recorded}
    return (RECOVERY_JOURNAL_OPEN + "".join(json.dumps(e) + "\n" for e in entries)
            + (json.dumps(close) + "\n" if closed else ""))


def _journal(tmp_path: Path, sys_id: int, *entries: dict) -> Path:
    path = tmp_path / f"uav_{sys_id}_recovery.jsonl"
    path.write_text(_process(*entries), encoding="utf-8")
    return path


def _write(tmp_path: Path, sys_id: int, *processes: str) -> None:
    (tmp_path / f"uav_{sys_id}_recovery.jsonl").write_text("".join(processes), encoding="utf-8")


def _analyzed(tmp_path: Path):
    _log_assignment(tmp_path, 5, "Task 2 assigned by owner 4")
    _log_assignment(tmp_path, 6, "Task 3 assigned by owner 4")
    return evaluator.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6), approvals=_approval_records())


def _errors(tmp_path: Path, sys_id: int) -> list[str]:
    return [error.split(": ", 1)[1] for error in recovery_evidence(tmp_path, sys_id, "").errors]


# ---------- the journal ----------


def test_a_journal_s_recoveries_are_read_across_navpy_restarts(tmp_path):
    _write(tmp_path, 1, _process(HOLD), _process(RESUME))
    evidence = recovery_evidence(tmp_path, 1, "")
    assert evidence.errors == ()
    assert [(e["source"], e["message"]) for e in evidence.events] == [
        ("nav", HOLD["message"]),
        ("process", "NavPy restarted (process 2; the previous one stopped)"),
        ("nav", RESUME["message"]),
    ]


def test_a_missing_or_malformed_journal_is_an_evidence_error(tmp_path):
    assert recovery_evidence(tmp_path, 1, "").errors == ("missing uav_1_recovery.jsonl",)
    _write(tmp_path, 2, RECOVERY_JOURNAL_OPEN, "not json\n", '{"x": 1}\n', _process()[len(RECOVERY_JOURNAL_OPEN):])
    assert _errors(tmp_path, 2) == ["line 2 is not JSON", "line 3 has no source and message"]
    _write(tmp_path, 3, json.dumps(HOLD) + "\n")
    assert _errors(tmp_path, 3) == ["line 1 comes before an open line", "no open line"]
    _journal(tmp_path, 4)
    marked = recovery_evidence(tmp_path, 4, "x ERROR RECOVERY_JOURNAL_FAILURE: OSError: disk full\n")
    assert marked.errors == ("recovery journal incomplete, not scored (RECOVERY_JOURNAL_FAILURE logged)",)


def test_r1_finding2_a_crashed_and_restarted_navpy_is_a_recovery(tmp_path):
    """Two processes, the first without a close line (it crashed), nothing else: not clean."""
    _write(tmp_path, 1, _process(closed=False), _process())
    evidence = recovery_evidence(tmp_path, 1, "")
    assert evidence.errors == ()
    assert [e["message"] for e in evidence.events] == ["NavPy restarted (process 2; the previous one ended without closing)"]


def test_r1_finding2_a_journal_cut_short_is_invalid(tmp_path):
    _write(tmp_path, 1, RECOVERY_JOURNAL_OPEN)
    assert _errors(tmp_path, 1) == [
        "NavPy's last process has no close line (cut short, or it did not stop cleanly)",
    ]
    _write(tmp_path, 2, _process(HOLD, recorded=2))       # a line lost before the close
    assert _errors(tmp_path, 2) == ["process 1 recorded 2 entries, the journal holds 1"]
    _write(tmp_path, 3, _process(HOLD, dropped=4))         # the writer's backlog overflowed
    assert _errors(tmp_path, 3) == ["process 1 dropped 4 entries"]
    _write(tmp_path, 4, _process(), json.dumps(HOLD) + "\n")
    assert _errors(tmp_path, 4) == ["line 3 follows its process's close line"]


def test_r1_finding5_a_journal_that_cannot_be_read_is_invalid_not_an_abort(tmp_path, monkeypatch):
    _journal(tmp_path, 1)
    original = Path.read_text

    def read_text(self, *args, **kwargs):
        if self.name == "uav_1_recovery.jsonl":
            raise PermissionError(13, "Access is denied", str(self))
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    assert _errors(tmp_path, 1)[0].startswith("unreadable: [Errno 13] Access is denied")


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


def test_r1_finding2_a_restart_in_a_clean_run_fails_the_clean_run_gate(tmp_path):
    _acked_run(tmp_path)
    _write(tmp_path, 4, _process(closed=False), _process())
    report = _analyzed(tmp_path)
    assert report.verdict == FAILED
    assert report.errors == ["uav 4: unexpected recovery in a clean run: process: NavPy restarted (process 2; the "
                             "previous one ended without closing)"]


def test_r1_finding3_the_lua_watchdog_s_recovery_fails_a_clean_run(tmp_path):
    """NavPy journals the autopilot's NAVWD hold (navpy/app/autopilot_watchdog_report.py); the verdict counts it."""
    _acked_run(tmp_path)
    _journal(tmp_path, 5, {"seq": 1, "t": 3.0, "source": "autopilot", "message": "autopilot NavPy watchdog: hold"})
    report = _analyzed(tmp_path)
    assert report.verdict == FAILED and "autopilot NavPy watchdog: hold" in report.errors[0]


def test_r1_finding4_a_missing_backend_log_makes_the_run_invalid(tmp_path):
    from scripts.eval_gcs_demo_constants import backend_log_path

    _acked_run(tmp_path)
    _log_assignment(tmp_path, 5, "Task 2 assigned by owner 4")
    backend_log_path(tmp_path).unlink()
    report = evaluator.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6), approvals=_approval_records())
    assert report.verdict == INVALID
    assert any(error.startswith("invalid global assignment evidence") for error in report.errors)


def test_r1_finding5_an_unreadable_navpy_log_makes_the_vehicle_invalid(tmp_path, monkeypatch):
    _acked_run(tmp_path)
    _log_assignment(tmp_path, 5, "Task 2 assigned by owner 4")
    _log_assignment(tmp_path, 6, "Task 3 assigned by owner 4")
    original = Path.read_text

    def read_text(self, *args, **kwargs):
        if self.name == "uav_6_navigation.log":
            raise PermissionError(13, "Access is denied", str(self))
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    report = evaluator.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6), approvals=_approval_records())
    vehicle = next(v for v in report.vehicles if v.sys_id == 6)
    assert vehicle.verdict == INVALID and report.verdict == INVALID


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
    assert run_verdict(vehicles, [], passed=True, faults_planned=True) == RECOVERED
    assert run_verdict([_vehicle(CLEAN)], [], passed=True, faults_planned=True) == RECOVERED   # the fault gates' recovery


def test_the_clean_run_gate_quotes_three_recoveries_and_counts_the_rest():
    errors = unexpected_recovery_errors([_vehicle(RECOVERED, *([HOLD] * 5))], faults_planned=False)
    assert errors[0].endswith("(+2 more)") and errors[0].count("nav: ") == 3


def test_invalid_outranks_failed_and_failed_outranks_recovered():
    assert run_verdict([_vehicle(INVALID), _vehicle(FAILED)], ["x"], passed=False, faults_planned=False) == INVALID
    assert run_verdict([_vehicle(CLEAN)], ["x"], passed=False, faults_planned=True) == FAILED


def test_r1_finding4_run_level_evidence_errors_are_invalid():
    """Each run-level evidence producer's prefix (the backend log, the lockstep journal and lines, the slice and
    delivery evidence)."""
    for error in ("invalid global assignment evidence: x", "certified lockstep evidence: the certified lockstep run "
                  "needs lockstep_navpy2_lines.jsonl", "slice evidence: x", "delivery: invalid evidence: x"):
        assert run_verdict([_vehicle(CLEAN)], [error], passed=False, faults_planned=False) == INVALID


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
