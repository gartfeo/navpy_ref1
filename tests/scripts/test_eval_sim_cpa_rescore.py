"""The offline rescore CLI: sim_cpa-only rewrite, canonical summary rebuild."""

from __future__ import annotations

import hashlib
import importlib
import json
from pathlib import Path

import pytest

from scripts import eval_sim_cpa_rescore as rescore
from scripts.eval_direct_pixel_summary import SUMMARY_SCHEMA_VERSION

_POI = {
    "lat_deg": 43.0, "lon_deg": 34.0, "rel_alt_m": 60.0, "abs_alt_m": 500.0,
}


def _case_dir(tmp_path: Path, name: str = "speed-1-run-1") -> Path:
    case_dir = tmp_path / name
    case_dir.mkdir()
    (case_dir / "case.json").write_text(
        json.dumps({"poi": _POI}), encoding="utf-8"
    )
    return case_dir


def _eligible_block() -> dict:
    return {
        "schema_version": 1,
        "authority": "cross_check_only",
        "configuration": {"status": "configured"},
        "artifact": {"status": "acquired"},
        "provenance": {},
        "evidence": {"status": "accepted"},
        "comparison": {"status": "compared", "disagreement": False},
    }


def test_rescore_case_rewrites_only_the_sim_cpa_block(
    tmp_path, monkeypatch
) -> None:
    """A rescore recovers module evidence; it never re-judges the flight."""
    case_dir = _case_dir(tmp_path)
    original = {
        "passed": True,
        "valid": True,
        "scoring_source": "sim_state_truth",
        "truth": {"dist_3d_m": 0.05, "certification_error": None},
        "sim_cpa": {"marker": "stale block"},
        "untouched_field": "must survive verbatim",
    }
    (case_dir / "verdict.json").write_text(
        json.dumps(original), encoding="utf-8"
    )
    payload = b"DFBIN-payload"
    bin_path = tmp_path / "some.BIN"
    bin_path.write_bytes(payload)
    score_mod = importlib.import_module(rescore.score_offline.__module__)
    monkeypatch.setattr(
        score_mod, "parse_sim_cpa_records", lambda p: ([], [])
    )

    verdict = rescore.rescore_case(case_dir, bin_path)

    on_disk = json.loads(
        (case_dir / "verdict.json").read_text(encoding="utf-8")
    )
    assert on_disk == verdict
    assert on_disk["untouched_field"] == "must survive verbatim"
    assert on_disk["passed"] is True and on_disk["valid"] is True
    block = on_disk["sim_cpa"]
    assert block["evidence"]["status"] == "no_scpc"
    assert block["artifact"]["binding"]["mode"] == "offline_explicit"
    assert block["artifact"]["sha256"] == hashlib.sha256(payload).hexdigest()


def test_rebuild_summary_goes_through_the_canonical_summarizer(
    tmp_path,
) -> None:
    """R14: the row is swapped and every counter is recomputed by
    summarize() -- counters are never patched individually."""
    case_dir = _case_dir(tmp_path)
    stale_row = {
        "speedup": 1.0, "repetition": 1, "passed": True, "valid": True,
        "passed_accuracy": True, "meets_goal": True,
        "scoring_source": "sim_state_truth",
    }
    (tmp_path / "summary.json").write_text(json.dumps({
        "results": [stale_row],
        "gate_m": 1.0,
        "goal_m": 0.1,
        "source_identity": {"sha256": "abc", "files": 1},
        "ab_eligible_runs": 0,
    }), encoding="utf-8")
    verdict = dict(stale_row)
    verdict["sim_cpa"] = _eligible_block()

    rescore.rebuild_summary(tmp_path, case_dir, verdict)

    rebuilt = json.loads(
        (tmp_path / "summary.json").read_text(encoding="utf-8")
    )
    assert rebuilt["schema_version"] == SUMMARY_SCHEMA_VERSION
    assert rebuilt["ab_eligible_runs"] == 1
    assert rebuilt["module_evidence_accepted_runs"] == 1
    assert rebuilt["results"][0]["sim_cpa"] == _eligible_block()
    assert rebuilt["results"][0]["repetition"] == 1


def test_rebuild_summary_refuses_an_unmatched_case(tmp_path) -> None:
    case_dir = _case_dir(tmp_path, name="speed-10-run-3")
    (tmp_path / "summary.json").write_text(json.dumps({
        "results": [{"speedup": 1.0, "repetition": 1}],
        "gate_m": 1.0,
        "goal_m": 0.1,
        "source_identity": {"sha256": "abc", "files": 1},
    }), encoding="utf-8")
    with pytest.raises(ValueError, match="no row"):
        rescore.rebuild_summary(tmp_path, case_dir, {"sim_cpa": None})
