"""Evaluator-level certificate gates: rate, identity, window, and the pinned bar.

These drive ``eval_navigation_cases`` rather than the helper module -- the point is
that the criteria reach ``passed`` and the process exit code, which is what was
missing when the measured rate and the identity artifact were computed and then
never consulted.
"""
from __future__ import annotations

import json
import sys
from collections import deque
from dataclasses import MISSING, fields
from unittest.mock import patch

import pytest

from scripts import eval_navigation_cases as evaluator


def _cli(*extra: str):
    return evaluator.parse_args(["--python", sys.executable, *extra])


# --------------------------------------------------------------------------
# The pinned accuracy bar.
# --------------------------------------------------------------------------


def test_certificate_mode_refuses_an_explicit_max_distance():
    # `--repetitions 10 --max-distance 100` would have produced a document
    # reading "10/10 passed" for misses two orders of magnitude out.
    with pytest.raises(SystemExit):
        _cli("--repetitions", "10", "--max-distance", "100")
    with pytest.raises(SystemExit):
        _cli("--repetitions", "10", "--max-dist", "0.4")


def test_certificate_mode_pins_the_default_bar():
    assert _cli("--repetitions", "10").max_distance == pytest.approx(
        evaluator.DEFAULT_MAX_DISTANCE_M)


def test_an_ordinary_run_may_still_choose_its_own_bar():
    # Only a certificate has to be comparable to other certificates.
    assert _cli("--max-distance", "2.5").max_distance == pytest.approx(2.5)
    assert _cli().max_distance == pytest.approx(evaluator.DEFAULT_MAX_DISTANCE_M)


# --------------------------------------------------------------------------
# The scoring interval window.
# --------------------------------------------------------------------------


class _PositionMessage:
    lat = 400_000_000
    lon = 440_000_000
    alt = 500_000
    relative_alt = 100_000

    def __init__(self, time_boot_ms):
        self.time_boot_ms = time_boot_ms

    def get_type(self):
        return "GLOBAL_POSITION_INT"

    def get_srcSystem(self):
        return 121


class _PositionMaster:
    def __init__(self, *boot_ms):
        self.messages = deque(_PositionMessage(ms) for ms in boot_ms)

    def recv_match(self, **_kwargs):
        return self.messages.popleft() if self.messages else None


def _drain(master, tracker, monotonic):
    clock = iter(monotonic)
    with patch.object(evaluator.time, "monotonic", lambda: next(clock)), \
         patch.object(evaluator.time, "time", lambda: 0.0):
        evaluator._drain_position_messages(
            master, sysid=121, live_anchors=deque(maxlen=2), scorer=None,
            rate_tracker=tracker)


def test_pre_scoring_interval_samples_cannot_reach_the_scoring_interval_rate():
    """The tracker does not exist before the POI-identity gate passes.

    Taxi, takeoff and cruise are minutes of samples at whatever rate the host
    managed while nothing was being scored. Averaging them into the scoring interval
    rate would let a slow climb cancel a fast dive and report a number the
    scored flight never ran at.
    """
    # Phase 1: no tracker yet (run_case holds None until the scorer is built).
    _drain(_PositionMaster(0, 1000), None, [0.0, 1.0, 2.0, 3.0])
    # Phase 2: the tracker is created at the gate and sees only what follows.
    tracker = evaluator.cert.ClockRateTracker(min_span_s=0.0)
    _drain(_PositionMaster(100_000, 110_000), tracker, [100.0, 101.0, 102.0, 103.0])
    result = tracker.result
    # 10 sim s over 1 monotonic s. The 1x pre-gate segment is simply absent.
    assert result.rate == pytest.approx(10.0)
    assert result.sample_count == 2


def test_the_tracker_and_the_scorer_are_created_together():
    from eval_navigation_case_state import CaseEvidence, ScoringIntervalWindow

    window_fields = fields(ScoringIntervalWindow)
    assert [field.name for field in window_fields] == ["scorer", "rate_tracker"]
    assert all(
        field.default is MISSING and field.default_factory is MISSING
        for field in window_fields
    )
    evidence_fields = {field.name for field in fields(CaseEvidence)}
    assert "scoring_interval" in evidence_fields
    assert "scorer" not in evidence_fields
    assert "rate_tracker" not in evidence_fields


# --------------------------------------------------------------------------
# main(): the rate and the identity must reach the exit code.
# --------------------------------------------------------------------------


def _run_certificate(tmp_path, monkeypatch, rows):
    """Run main()'s certificate branch over pre-built rows, one per repetition."""
    supplied = list(rows)

    def fake_run_case(index, case, attempt, run_dir, args):
        row = dict(supplied[attempt - 1])
        row.update(index=index, attempt=attempt, repetition=attempt,
                   certificate_mode=True, name=f"run-{attempt}")
        return row

    monkeypatch.setattr(evaluator, "run_case", fake_run_case)
    monkeypatch.setattr(evaluator, "WORKTREE", tmp_path)
    code = evaluator.main([
        "--python", sys.executable,
        "--repetitions", str(len(supplied)),
        "--speedups", str(evaluator.cert.CERTIFICATE_SPEEDUP),
        "--winds", "0",
    ])
    run_dir = next(iter((tmp_path / ".sitl-runs").iterdir()))
    summary = json.loads(
        (run_dir / "certificate-0.json").read_text(encoding="utf-8"))["summary"]
    return code, summary


def _good_row(**overrides):
    row = {
        "passed": True,
        "identity_gate_passed": True,
        "error": "",
        "dist_3d_m": 0.21,
        "coordinate_dist_3d_m": 0.23,
        "measured_clock_rate": 10.02,
        "speedup": evaluator.cert.CERTIFICATE_SPEEDUP,
        "certificate_invalid_reason": "",
        "identity_navpy_commit": "abc123",
        "identity_mission_sha": "m1",
        "identity_parameters_sha": "p1",
        "identity_autopilot": "fw1",
    }
    row.update(overrides)
    return row


def test_a_clean_certificate_is_valid_and_exits_zero(tmp_path, monkeypatch):
    code, summary = _run_certificate(
        tmp_path, monkeypatch, [_good_row(), _good_row(), _good_row()])
    assert code == 0
    assert summary["valid"]


def test_a_cross_run_identity_mismatch_voids_the_certificate(
        tmp_path, monkeypatch):
    # Every run passed with a good miss. The certificate is still void: these
    # are not repetitions of one build.
    code, summary = _run_certificate(tmp_path, monkeypatch, [
        _good_row(),
        _good_row(identity_navpy_commit="def456"),
    ])
    assert code == 1
    assert not summary["valid"]
    assert summary["passed_runs"] == 2  # the runs themselves were fine
    assert summary["consistency_errors"]


def test_an_inadmissible_run_voids_the_certificate_and_the_exit_code(
        tmp_path, monkeypatch):
    code, summary = _run_certificate(tmp_path, monkeypatch, [
        _good_row(),
        # run_case marks a 12x run invalid; the approach error is still good.
        _good_row(passed=False, measured_clock_rate=12.0,
                  certificate_invalid_reason=(
                      "scoring-window clock rate 12.00x is outside 9.00-11.00x"),
                  error="scoring-window clock rate 12.00x is outside 9.00-11.00x"),
    ])
    assert code == 1
    assert not summary["valid"]
    assert summary["invalid_runs"] == 1


def test_the_summary_records_the_reason_each_run_was_inadmissible(
        tmp_path, monkeypatch):
    _code, summary = _run_certificate(tmp_path, monkeypatch, [
        _good_row(passed=False, certificate_invalid_reason="NavPy tree dirty"),
    ])
    assert "NavPy tree dirty" in summary["invalid_reasons"][0]
    assert summary["per_run"][0]["invalid_reason"] == "NavPy tree dirty"


# --------------------------------------------------------------------------
# run_case's own admissibility decision.
# --------------------------------------------------------------------------


def _clean_identity():
    return {
        "navpy": {"commit": "abc123", "dirty": False},
        "mission": {"item_count": 3, "sha256": "m1"},
        "parameters": {"complete": True, "parameter_count": 3,
                       "vehicle_param_count": 3, "sha256": "p1"},
        "autopilot": {
            "available": True,
            "flight_sw_version": 123,
            "flight_custom_version": "f" * 16,
        },
    }


def test_a_good_run_at_the_right_rate_is_admissible():
    assert evaluator.certificate_invalid_reason_for(
        True, measured_rate=10.02, requested_speedup=10,
        identity=_clean_identity()) == ""


@pytest.mark.parametrize("rate", [12.0, 7.5, None])
def test_a_wrong_or_missing_rate_makes_the_run_inadmissible(rate):
    # The approach error is irrelevant here: this is about whether the run is
    # even eligible to be counted.
    reason = evaluator.certificate_invalid_reason_for(
        True, measured_rate=rate, requested_speedup=10,
        identity=_clean_identity())
    assert reason
    assert "clock rate" in reason


def test_a_dirty_tree_makes_the_run_inadmissible():
    identity = _clean_identity()
    identity["navpy"]["dirty"] = True
    reason = evaluator.certificate_invalid_reason_for(
        True, measured_rate=10.0, requested_speedup=10, identity=identity)
    assert "dirty" in reason


def test_an_incomplete_parameter_snapshot_makes_the_run_inadmissible():
    identity = _clean_identity()
    identity["parameters"]["complete"] = False
    reason = evaluator.certificate_invalid_reason_for(
        True, measured_rate=10.0, requested_speedup=10, identity=identity)
    assert "incomplete" in reason


def test_all_the_reasons_are_reported_together_not_just_the_first():
    # An operator fixing one and re-flying only to hit the next is a wasted
    # certificate run, and those are expensive.
    identity = _clean_identity()
    identity["navpy"]["dirty"] = True
    identity["autopilot"]["available"] = False
    reason = evaluator.certificate_invalid_reason_for(
        True, measured_rate=None, requested_speedup=10, identity=identity)
    assert "clock rate" in reason
    assert "dirty" in reason
    assert "firmware identity" in reason


def test_an_ordinary_run_is_never_marked_inadmissible():
    # Non-certificate behaviour is unchanged: no rate check, no identity check.
    assert evaluator.certificate_invalid_reason_for(
        False, measured_rate=None, requested_speedup=10, identity=None) == ""
