"""Reject falsely successful clock evidence and expose baseline differences."""

import csv
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from eval_simtime_step import validate_evidence  # noqa: E402
from compare_simtime_steps import compare, summarize, revalidate  # noqa: E402
from simtime_step_failures import validate_failure  # noqa: E402


def evidence(case, *, changes=None, peer_change=None, count=1000):
    rows = [{"step": index, "tick": index + 80,
             "before_us": index * 20000, "after_us": index * 20000,
             "complete_us": index * 20000, "wait_us": 100,
             "wall_us": index * 2000} for index in range(1, count + 1)]
    if changes:
        rows[9].update(changes)
    with (case / "navpy-step.csv").open("w", newline="") as stream:
        stream.write("# NVSTEP01,boot=00000000000000010000000000000002,handshake=1,vehicle=121\n")
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    peer = [{"step": index, "tick": index + 80, "source_us": index * 20000,
             "boot": [1, 2], "vehicle": 121} for index in range(1, count + 1)]
    if peer_change:
        peer[9].update(peer_change)
    (case / "peer.json").write_text(json.dumps(
        {"records": peer, "fault": "none", "delayed": False}))


def test_valid_window_and_measured_speed(tmp_path):
    evidence(tmp_path)
    assert validate_evidence(tmp_path, "immediate", 121)["achieved_speed"] == 10


@pytest.mark.parametrize("changes", [{"after_us": 200001}, {"tick": 91},
                                     {"before_us": 160000, "after_us": 160000},
                                     {"step": 11}, {"complete_us": 199999},
                                     {"complete_us": 220000}])
def test_reject_invalid_ap_window(tmp_path, changes):
    evidence(tmp_path, changes=changes)
    with pytest.raises(ValueError):
        validate_evidence(tmp_path, "immediate", 121)


@pytest.mark.parametrize("change", [{"boot": [2, 1]}, {"vehicle": 122},
                                    {"step": 11}, {"source_us": 200001}])
def test_reject_peer_identity_mismatch(tmp_path, change):
    evidence(tmp_path, peer_change=change)
    with pytest.raises(ValueError):
        validate_evidence(tmp_path, "immediate", 121)


def test_reject_incomplete_window(tmp_path):
    evidence(tmp_path, count=999)
    with pytest.raises(ValueError):
        validate_evidence(tmp_path, "immediate", 121)


def test_reject_missing_delays(tmp_path):
    evidence(tmp_path)
    with pytest.raises(ValueError, match="host delay"):
        validate_evidence(tmp_path, "delayed", 121)


def test_reject_wrong_reserved_vehicle(tmp_path):
    evidence(tmp_path)
    with pytest.raises(ValueError, match="reserved instance"):
        validate_evidence(tmp_path, "immediate", 122)


def test_reject_mislabelled_peer_condition(tmp_path):
    evidence(tmp_path)
    path = tmp_path / "peer.json"
    peer = json.loads(path.read_text())
    peer["delayed"] = True
    path.write_text(json.dumps(peer))
    with pytest.raises(ValueError, match="experimental condition"):
        validate_evidence(tmp_path, "immediate", 121)


def test_compare_excludes_only_host_measurements():
    left = [{"step": "1", "tick": "5", "before_us": "100", "roll": "0.1",
             "wait_us": "1", "wall_us": "300"}]
    right = [left[0] | {"wait_us": "1000", "wall_us": "5000"}]
    assert compare(left, right)["exact_equal"]
    right[0]["before_us"] = "101"
    right[0]["roll"] = "0.2"
    result = compare(left, right)
    assert not result["exact_equal"]
    assert set(result["differences"]) == {"before_us", "roll"}


@pytest.mark.parametrize("error,boots", [("connect", 1), ("deadline", 2)])
def test_negative_trial_cannot_pass_for_wrong_cause_or_retry(tmp_path, error, boots):
    (tmp_path / "supervisor.log").write_text(
        "Starting sketch 'ArduPlane'\n" * boots + f"NAVPY_STEP_INVALID {error} step=5\n")
    with pytest.raises(ValueError):
        validate_failure(tmp_path, "timeout")


def test_intended_timeout_evidence(tmp_path):
    (tmp_path / "supervisor.log").write_text(
        "Starting sketch 'ArduPlane'\nNAVPY_STEP_INVALID deadline step=5\n")
    (tmp_path / "peer.json").write_text(json.dumps(
        {"fault": "timeout", "records": [{"step": i} for i in range(1, 6)]}))
    assert validate_failure(tmp_path, "timeout")["failed_step"] == 5


def test_comparison_produces_cross_speed_evidence(tmp_path):
    for speed in (1, 10):
        case = tmp_path / f"speed-{speed}"
        case.mkdir()
        (case / "result.json").write_text(json.dumps(
            {"speed": speed, "mode": "observe", "fault": "none", "passed": True}))
        (case / "navpy-step.csv").write_text(
            "# metadata\nstep,tick,before_us,after_us,complete_us,wait_us,wall_us\n"
            "1,10,1000,1000,1000,0,2000\n")
    reports = summarize(tmp_path)
    assert len(reports) == 1
    assert reports[0]["kind"] == "cross-speed-baseline"
    assert reports[0]["exact_equal"]


def test_revalidation_rejects_identical_window_after_restart(tmp_path):
    case = tmp_path / "case"
    case.mkdir()
    evidence(case)
    (case / "result.json").write_text(json.dumps(
        {"mode": "immediate", "fault": "none", "vehicle": 121}))
    (case / "supervisor.log").write_text("Starting sketch 'ArduPlane'\n" * 2 +
        "instances streaming telemetry at SIM_SPEEDUP=10 (attempt 2)\n")
    with pytest.raises(ValueError, match="first boot"):
        revalidate(tmp_path)
