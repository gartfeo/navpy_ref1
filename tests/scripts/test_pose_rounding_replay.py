"""Counterfactual replay must be explicit, controlled, and isolated from captures."""

import copy
import json

import pytest

from scripts.pose_rounding_evidence import PosePair
from scripts.replay_pose_rounding import replay_arm
from scripts.simtime_navigation_protocol import Snapshot
from tests.scripts.test_replay_navigation_trace import capture


def pairs_for(peer):
    pairs = []
    for record in peer["records"]:
        truth = Snapshot.decode(bytes.fromhex(record["snapshot"])).truth
        rounded = (truth.latitude, truth.longitude, truth.altitude)
        pairs.append(PosePair(rounded, (truth.latitude, truth.longitude+1e-5, truth.altitude)))
    return pairs


def test_legacy_control_matches_and_precast_changes_only_offline_pixels(capture, tmp_path):
    peer, _ = capture
    original = copy.deepcopy(peer)
    pairs = pairs_for(peer)
    legacy = replay_arm(peer, pairs, tmp_path / "legacy", arm="legacy")
    precast = replay_arm(peer, pairs, tmp_path / "precast", arm="precast")
    assert legacy["exact_control"] and not precast["exact_control"]
    assert legacy["records"] == precast["records"] == len(peer["records"])
    assert legacy["command_records"][10]["evidence"]["pixels"] != precast["command_records"][10]["evidence"]["pixels"]
    assert peer == original


def test_legacy_pair_cannot_silently_change_recorded_input(capture, tmp_path):
    peer, _ = capture
    pairs = pairs_for(peer)
    pairs[10] = PosePair(pairs[10].precast, pairs[10].precast)
    with pytest.raises(ValueError, match="legacy pose control"):
        replay_arm(peer, pairs, tmp_path / "bad", arm="legacy")


def matrix_fixture(tmp_path, monkeypatch):
    from scripts import pose_rounding_analysis as analysis
    monkeypatch.setattr(analysis, "ROOT", tmp_path)
    baseline = tmp_path / "baseline.json"
    baseline.write_text(json.dumps(dict(normalized_history_sha256="baseline")))
    monkeypatch.setattr(analysis, "BASELINE", baseline)
    firmware = tmp_path / "firmware.json"
    firmware.write_text(json.dumps(dict(binary_sha256="reviewed", base_commit="base")))
    monkeypatch.setattr(analysis, "FIRMWARE", firmware)
    (tmp_path / "scripts").mkdir()
    for name in ("run_pose_matrix.py", "pose_rounding_evidence.py"):
        (tmp_path / "scripts" / name).write_text(name)
    attempts, runs = [], []
    for i, cell in enumerate(analysis.cells()):
        directory = tmp_path / f"run-{i}"
        directory.mkdir()
        (directory / "fleet.json").write_text("{}")
        attempts.append(dict(cell, directory=str(directory), status="captured", run_id=str(i)))
        runs.append(dict(cell, directory=str(directory), run_id=str(i), boot=[i,1],
            steps=3000, normalized_history_sha256="baseline", pose_hash="pose" if i else None,
            identity=dict(binary_sha256="reviewed",firmware_head="base"),
            manifest_sha256=analysis.digest(directory / "fleet.json")))
    ledger = tmp_path / "attempts.json"
    ledger.write_text(json.dumps(dict(attempts=attempts)))
    report = dict(version=1, passed=True, steps=15000,
        method_sha256=analysis.digest(tmp_path / "scripts/run_pose_matrix.py"),
        pairs_method_sha256=analysis.digest(tmp_path / "scripts/pose_rounding_evidence.py"),
        baseline_sha256=analysis.digest(baseline), ledger=str(ledger),
        ledger_sha256=analysis.digest(ledger), runs=runs)
    return analysis, report


def test_complete_matrix_is_required_before_attribution(tmp_path, monkeypatch):
    analysis, report = matrix_fixture(tmp_path, monkeypatch)
    analysis.require_matrix(report)
    report["runs"] = report["runs"][:-1]
    with pytest.raises(ValueError, match="missing cells"):
        analysis.require_matrix(report)


@pytest.mark.parametrize("change", ["pose", "history", "boot", "manifest", "method", "status", "firmware"])
def test_changed_matrix_cannot_authorize_analysis(tmp_path, monkeypatch, change):
    analysis, report = matrix_fixture(tmp_path, monkeypatch)
    if change == "pose": report["runs"][2]["pose_hash"] = "other"
    elif change == "history": report["runs"][2]["normalized_history_sha256"] = "other"
    elif change == "boot": report["runs"][2]["boot"] = report["runs"][1]["boot"]
    elif change == "manifest": report["runs"][2]["manifest_sha256"] = "other"
    elif change == "method": report["method_sha256"] = "other"
    elif change == "status": report["passed"] = False
    elif change == "firmware": report["runs"][1]["identity"]["binary_sha256"] = "other"
    with pytest.raises(ValueError):
        analysis.require_matrix(report)
