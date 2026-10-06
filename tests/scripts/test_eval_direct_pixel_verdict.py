"""Scoring-policy semantics: truth is authoritative, never silently faked."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from scripts import eval_direct_pixel_verdict as verdict
from scripts.eval_navigation_models import ClosestApproach


def _decision(**overrides):
    arguments = {
        "scoring_policy": verdict.SCORING_POLICY_SITL_TRUTH,
        "child_result": {"snap_3d_m": 0.4},
        "closest": SimpleNamespace(dist_3d_m=0.3),
        "truth_block": {"certification_error": None, "dist_3d_m": 0.05},
        "max_distance_m": 1.0,
        "goal_distance_m": 0.1,
    }
    arguments.update(overrides)
    return verdict.scoring_decision(**arguments)


def test_certified_truth_decides_pass_and_goal() -> None:
    decision = _decision()
    assert decision["accuracy_errors"] == []
    assert decision["passed_accuracy"] is True
    assert decision["meets_goal"] is True
    assert decision["scoring_source"] == "sim_state_truth"


def test_ekf_gates_demote_to_warnings_under_truth_policy() -> None:
    decision = _decision(
        child_result={"snap_3d_m": 5.0},
        closest=SimpleNamespace(dist_3d_m=7.0),
    )
    assert decision["accuracy_errors"] == []
    assert decision["passed_accuracy"] is True
    assert len(decision["ekf_gate_warnings"]) == 2


def test_truth_over_gate_fails_accuracy() -> None:
    decision = _decision(
        truth_block={"certification_error": None, "dist_3d_m": 1.5}
    )
    assert any("truth CPA" in error for error in decision["accuracy_errors"])
    assert decision["passed_accuracy"] is False
    assert decision["meets_goal"] is False


def test_uncertified_truth_fails_closed_with_null_goal() -> None:
    decision = _decision(
        truth_block={
            "certification_error": "truth delivered rate 9.9Hz below 30.0Hz",
            "dist_3d_m": 0.05,
        }
    )
    assert any("uncertified" in error for error in decision["accuracy_errors"])
    assert decision["passed_accuracy"] is None
    assert decision["meets_goal"] is None
    assert decision["scoring_source"] == "none"


def test_missing_truth_recorder_fails_closed() -> None:
    decision = _decision(truth_block=None)
    assert any("unavailable" in error for error in decision["accuracy_errors"])
    assert decision["passed_accuracy"] is None
    assert decision["meets_goal"] is None


def test_estimate_policy_keeps_legacy_gates_and_labels_the_source() -> None:
    decision = _decision(
        scoring_policy=verdict.SCORING_POLICY_VEHICLE_ESTIMATE,
        truth_block=None,
        child_result={"snap_3d_m": 0.05},
    )
    assert decision["accuracy_errors"] == []
    assert decision["passed_accuracy"] is True
    assert decision["meets_goal"] is True
    assert decision["scoring_source"] == "ekf_snap_estimate"
    assert decision["ekf_gate_warnings"] == []


def test_estimate_policy_rejects_boolean_snap() -> None:
    decision = _decision(
        scoring_policy=verdict.SCORING_POLICY_VEHICLE_ESTIMATE,
        truth_block=None,
        child_result={"snap_3d_m": True},
    )
    assert any("SNAP" in error for error in decision["accuracy_errors"])
    assert decision["meets_goal"] is False


def test_boolean_truth_distance_is_not_a_score() -> None:
    decision = _decision(
        truth_block={"certification_error": None, "dist_3d_m": True}
    )
    assert any(
        "no closest approach" in error for error in decision["accuracy_errors"]
    )
    assert decision["passed_accuracy"] is None


# The evidence blocks a real flight attaches to a successful verdict; the
# homogeneity guarantee is the CORE schema, and these exist only when a
# flight produced them (Codex round-5 finding 5: the earlier stub hid three
# of the four and overstated the claim).
_FLIGHT_EVIDENCE = {
    "ground_track": "ok",
    "command_stability": {"sample_count": 1},
    "observation_freshness": {"fresh_fraction": 1.0},
    "command_causality": {"sample_count": 1},
}


def test_unscored_result_matches_the_core_case_verdict_field_set(
    tmp_path: Path, monkeypatch
) -> None:
    """Early bailouts carry every core field; flight evidence is conditional."""
    early = verdict.unscored_result(
        ["SourceChangedError: source changed"],
        scoring_policy=verdict.SCORING_POLICY_SITL_TRUTH,
    )
    full = _case_verdict(tmp_path, monkeypatch)
    assert set(early) == set(full) - set(_FLIGHT_EVIDENCE)
    for key in _FLIGHT_EVIDENCE:
        assert key in full
    assert early["passed"] is False and early["valid"] is False
    assert early["meets_goal"] is None
    assert early["scoring_source"] == "none"
    assert early["truth_stream_acknowledged"] is None


def test_unscored_result_preserves_the_observed_ack_and_persists(
    tmp_path: Path,
) -> None:
    """A failure AFTER the truth request must not erase what was observed
    (Codex round-5 finding 4): the real ACK and any recorder verdict ride
    along, and the row lands on disk like a scored one."""
    truth_block = {"certification_error": "flight aborted", "dist_3d_m": None}
    result = verdict.unscored_result(
        ["RuntimeError: child died"],
        scoring_policy=verdict.SCORING_POLICY_SITL_TRUTH,
        truth_stream_acknowledged=True,
        truth_block=truth_block,
        case_dir=tmp_path,
    )
    assert result["truth_stream_acknowledged"] is True
    assert result["truth"] == truth_block
    on_disk = json.loads(
        (tmp_path / "verdict.json").read_text(encoding="utf-8")
    )
    assert on_disk["truth_stream_acknowledged"] is True
    assert on_disk["passed"] is False and on_disk["valid"] is False


def _case_verdict(tmp_path: Path, monkeypatch, **overrides):
    """Drive the real assembly with validity stubbed at its module seam."""
    validity = overrides.pop("validity", ([], dict(_FLIGHT_EVIDENCE)))
    monkeypatch.setattr(verdict, "validity_errors", lambda *a, **k: validity)
    arguments = {
        "child_result": {"snap_3d_m": 0.4},
        "scorer": SimpleNamespace(
            result=ClosestApproach(0.3, 0.25, 0.15), sample_count=250
        ),
        "track": SimpleNamespace(),
        "truth": SimpleNamespace(
            verdict=lambda: {"certification_error": None, "dist_3d_m": 0.05}
        ),
        "truth_stream_accepted": True,
        "scoring_policy": verdict.SCORING_POLICY_SITL_TRUTH,
        "max_distance_m": 1.0,
        "goal_distance_m": 0.1,
        "wind_speed": 0.0,
        "wind_dir_deg": 0.0,
        "speedup": 1.0,
    }
    arguments.update(overrides)
    return verdict.case_verdict(tmp_path, **arguments)


def test_case_verdict_assembles_and_persists_a_certified_pass(
    tmp_path: Path, monkeypatch
) -> None:
    result = _case_verdict(tmp_path, monkeypatch)
    assert result["passed"] is True
    assert result["valid"] is True
    assert result["scoring_source"] == "sim_state_truth"
    assert result["truth_stream_acknowledged"] is True
    assert abs(result["ekf_truth_divergence_m"] - 0.25) < 1e-9
    assert result["ground_track"] == "ok"
    assert result["coordinate_samples"] == 250
    on_disk = json.loads(
        (tmp_path / "verdict.json").read_text(encoding="utf-8")
    )
    assert on_disk["passed"] is True and on_disk["valid"] is True


def test_case_verdict_uncertified_truth_is_invalid(
    tmp_path: Path, monkeypatch
) -> None:
    result = _case_verdict(
        tmp_path,
        monkeypatch,
        truth=SimpleNamespace(
            verdict=lambda: {
                "certification_error": "truth delivered rate 9.9Hz below 30.0Hz",
                "dist_3d_m": 0.05,
            }
        ),
    )
    assert result["passed"] is False
    assert result["valid"] is False
    assert result["passed_accuracy"] is None
    assert result["scoring_source"] == "none"


def test_case_verdict_certified_over_gate_miss_is_valid_but_failed(
    tmp_path: Path, monkeypatch
) -> None:
    result = _case_verdict(
        tmp_path,
        monkeypatch,
        truth=SimpleNamespace(
            verdict=lambda: {"certification_error": None, "dist_3d_m": 1.5}
        ),
    )
    assert result["passed"] is False
    assert result["valid"] is True
    assert result["passed_accuracy"] is False


def test_case_verdict_validity_failure_invalidates_even_certified_truth(
    tmp_path: Path, monkeypatch
) -> None:
    result = _case_verdict(
        tmp_path,
        monkeypatch,
        validity=(["ground track diverges from the planned leg"], {}),
    )
    assert result["passed"] is False
    assert result["valid"] is False
    assert result["passed_accuracy"] is True
