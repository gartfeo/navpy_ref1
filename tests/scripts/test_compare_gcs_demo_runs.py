"""Offline mutations must remain visible across complete and incomplete captures."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.compare_gcs_demo_runs import main
from scripts import gcs_demo_comparison as comparison
from scripts.gcs_demo_comparison import compare_runs
from tests.scripts.test_eval_gcs_navigation_demo import (
    _approval_records,
    _write_episode,
    _write_resolved_plan,
)


def make_run(path: Path) -> Path:
    path.mkdir()
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(path)
    (path / "operator_approvals.json").write_text(
        json.dumps({"approvals": _approval_records()}), encoding="utf-8"
    )
    (path / "gcs_settings.json").write_text('{"camera":{"vision_profile":"siyi_zr10"}}')
    return path


@pytest.fixture
def runs(tmp_path):
    return [make_run(tmp_path / "first"), make_run(tmp_path / "second")]


def replace(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    assert old in text
    path.write_text(text.replace(old, new), encoding="utf-8")


def test_equal_synthetic_logs_do_not_claim_fresh_boots_or_full_determinism(runs, tmp_path):
    report = compare_runs(runs)
    assert report["recorded_evidence_equal"] is True
    assert report["all_per_run_gates_pass"] is True
    assert report["full_scenario_repeatability"] == "unverified"
    assert report["comparisons"][0]["independence_warnings"]
    assert "sim_speedup" not in report["runs"][0]["recorded_inputs"]
    assert report["runs"][0]["analysis_assumptions"]["sim_speedup"] == 10.0
    assert report["analysis_source_stable"] is True
    assert main([str(p) for p in runs] + ["--output", str(tmp_path / "report.json")]) == 3


def test_same_directory_cannot_count_twice(runs):
    with pytest.raises(ValueError, match="same run directory"):
        compare_runs([runs[0], runs[0] / ".." / "first"])


def test_missing_attempt_evidence_is_retained_and_incomplete(runs):
    (runs[1] / "operator_approvals.json").unlink()
    (runs[1] / "failure.txt").write_text("setup stopped before approvals")
    report = compare_runs(runs)
    assert not report["recorded_evidence_equal"]
    assert report["comparisons"][0]["outcome"]["status"] == "incomplete"
    assert report["runs"][1]["errors"]
    assert "failure.txt" in report["runs"][1]["sha256"]


def test_unstarted_attempt_is_not_dropped(runs, tmp_path):
    report = compare_runs([*runs, tmp_path / "never_started"])
    assert len(report["runs"]) == 3
    assert report["runs"][2]["errors"] == ["run directory is missing"]
    assert report["comparisons"][1]["recorded_inputs"]["status"] == "incomplete"
    assert report["recorded_evidence_equal"] is False


def test_changed_command_is_reported_even_if_each_approach_passes(runs):
    replace(runs[1] / "uav_5_navigation_debug.csv", "cmd_roll=12.00", "cmd_roll=12.01")
    report = compare_runs(runs)
    assert report["all_per_run_gates_pass"]
    difference = report["comparisons"][0]["sampled_commands"]["first_difference"]
    assert difference == {"path": "$.5[0][0].cmd_roll", "left": 12.0, "right": 12.01}
    assert not report["recorded_evidence_equal"]


def test_pass_suppressed_tail_is_not_trimmed(runs):
    path = runs[1] / "uav_4_navigation_debug.csv"
    lines = path.read_text().splitlines()
    tail = next(row for row in lines if "passed=True" in row)
    lines.insert(-1, tail.replace("12:00:00.120", "12:00:00.130").replace(
        "obs_ts=1001.200", "obs_ts=1001.300"))
    path.write_text("\n".join(lines) + "\n")
    report = compare_runs(runs)
    difference = report["comparisons"][0]["sampled_commands"]["first_difference"]
    assert difference["path"] == "$.4[0].length"
    assert difference["left"] == 13 and difference["right"] == 14


def test_boot_nonce_audited_but_message_sequence_remains_exact(runs):
    approvals = runs[1] / "operator_approvals.json"
    replace(approvals, '"10:1"', '"20:1"')
    report = compare_runs(runs)
    assert report["comparisons"][0]["outcome"]["status"] == "equal"
    assert report["comparisons"][0]["approval_audit"]["status"] == "different"
    replace(approvals, '"20:1"', '"20:2"')
    report = compare_runs(runs)
    assert report["comparisons"][0]["outcome"]["status"] == "different"


def test_actual_operator_delay_is_an_input_difference(runs):
    approvals = runs[1] / "operator_approvals.json"
    replace(approvals, "1015.1", "1015.2")
    replace(approvals, "15.1", "15.2")
    report = compare_runs(runs)
    assert report["all_per_run_gates_pass"]
    assert report["comparisons"][0]["recorded_inputs"]["status"] == "different"


def test_changed_target_is_an_input_mismatch_without_coordinate_tolerance(runs):
    replace(runs[1] / "demo_mission_plan.json", "40.1,", "40.10000001,")
    report = compare_runs(runs)
    assert report["comparisons"][0]["recorded_inputs"]["status"] == "different"


def test_good_saved_report_cannot_override_failed_raw_evidence(runs):
    (runs[1] / "report.json").write_text('{"passed":true}')
    replace(runs[1] / "uav_4_navigation_debug.csv", "dist_3d_m=0.400000", "dist_3d_m=0.500000")
    report = compare_runs(runs)
    assert not report["all_per_run_gates_pass"]
    assert report["limits"]["truth"]["max_snap_distance_m"] == 0.5


@pytest.mark.parametrize("settings", ['{"x":NaN}', '{"x":1e999}', '{"x":1,"x":2}'])
def test_malformed_settings_cannot_compare_equal(runs, settings):
    for run in runs:
        (run / "gcs_settings.json").write_text(settings)
    report = compare_runs(runs)
    assert not report["recorded_evidence_equal"]
    assert all(row["errors"] for row in report["runs"])


def test_report_cannot_overwrite_input_or_existing_evidence(runs, tmp_path):
    with pytest.raises(SystemExit) as error:
        main([str(p) for p in runs] + ["--output", str(runs[0] / "report.json")])
    assert error.value.code == 2
    output = tmp_path / "retained.json"
    output.write_text("retained")
    with pytest.raises(SystemExit) as error:
        main([str(p) for p in runs] + ["--output", str(output)])
    assert error.value.code == 2
    assert output.read_text() == "retained"


def test_analysis_error_retains_all_attempts(runs, monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError("injected analysis failure")

    monkeypatch.setattr(comparison, "analyze_run", fail)
    report = compare_runs(runs)
    assert len(report["runs"]) == 2
    assert all(row["errors"] == ["RuntimeError: injected analysis failure"] for row in report["runs"])
    assert all(row["sha256"] for row in report["runs"])


def test_bad_first_vehicle_does_not_hide_peer_traces(runs):
    (runs[1] / "uav_4_navigation_debug.csv").write_text("ts,event,payload\n")
    report = compare_runs(runs)
    assert report["runs"][1]["errors"]
    assert set(report["runs"][1]["sampled_commands"]) == {"5", "6"}


def test_mutated_capture_cannot_be_equal(runs, monkeypatch):
    original = comparison.analyze_run

    def mutate(path, **kwargs):
        result = original(path, **kwargs)
        (path / "late-arrival.log").write_text("arrived during comparison")
        return result

    monkeypatch.setattr(comparison, "analyze_run", mutate)
    report = compare_runs(runs)
    assert not report["recorded_evidence_equal"]
    assert all("run evidence changed while being read" in row["errors"] for row in report["runs"])
