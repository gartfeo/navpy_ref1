"""The leg driver ranks legs only by A/B-eligible certified truth rows."""

from __future__ import annotations

import json
from pathlib import Path

from scripts import eval_leg_independence as leg


def _eligible_block(disagreement: bool = False) -> dict:
    return {
        "schema_version": 1,
        "authority": "cross_check_only",
        "evidence": {"status": "accepted"},
        "comparison": {"status": "compared", "disagreement": disagreement},
    }


def test_report_reads_truth_and_surfaces_unscored_runs(tmp_path: Path) -> None:
    (tmp_path / "summary.json").write_text(json.dumps({
        "results": [
            {
                # The proof-pass configuration: snap and truth disagree by
                # design.  Only the certified truth number may be reported.
                "scoring_source": "sim_state_truth",
                "valid": True,
                "truth": {"dist_3d_m": 0.05},
                "child": {"snap_3d_m": 0.9},
                "sim_cpa": _eligible_block(),
            },
            {
                # Uncertified run: never fall back to the child's snap.
                "scoring_source": "none",
                "valid": False,
                "truth": {"dist_3d_m": None},
                "child": {"snap_3d_m": 0.02},
                "sim_cpa": _eligible_block(),
            },
        ],
    }), encoding="utf-8")

    report = leg._report(tmp_path, 700.0, 135.4)

    assert report["misses_m"] == [0.05]
    assert report["unscored_runs"] == 1
    assert report["deferral"] == []


def test_report_excludes_certified_truth_from_invalid_runs(
    tmp_path: Path,
) -> None:
    """The live 10x case (Codex round-5 finding 2): a freshness-starved run
    carries a perfectly certified truth CPA that measures host contention,
    not navigation. It must count as unscored, while a VALID over-gate miss
    stays usable -- navigation was measured and navigation missed."""
    (tmp_path / "summary.json").write_text(json.dumps({
        "results": [
            {
                "scoring_source": "sim_state_truth",
                "valid": False,
                "passed_accuracy": True,
                "truth": {"dist_3d_m": 0.003},
                "child": {"snap_3d_m": 0.2},
                "sim_cpa": _eligible_block(),
            },
            {
                "scoring_source": "sim_state_truth",
                "valid": True,
                "passed_accuracy": False,
                "truth": {"dist_3d_m": 1.5},
                "child": {"snap_3d_m": 0.2},
                "sim_cpa": _eligible_block(),
            },
        ],
    }), encoding="utf-8")

    report = leg._report(tmp_path, 700.0, 135.4)

    assert report["misses_m"] == [1.5]
    assert report["unscored_runs"] == 1


def test_report_excludes_disagreed_and_module_absent_rows(
    tmp_path: Path,
) -> None:
    """The cross-check's teeth (verdict-module-score R2): a module/stream
    disagreement or missing module evidence keeps a valid run OUT of the
    campaign statistics -- and pre-v3 rows with no block at all are not
    grandfathered in."""
    (tmp_path / "summary.json").write_text(json.dumps({
        "results": [
            {
                "scoring_source": "sim_state_truth",
                "valid": True,
                "truth": {"dist_3d_m": 0.04},
                "sim_cpa": _eligible_block(disagreement=True),
            },
            {
                "scoring_source": "sim_state_truth",
                "valid": True,
                "truth": {"dist_3d_m": 0.05},
                "sim_cpa": {
                    "schema_version": 1,
                    "authority": "cross_check_only",
                    "evidence": {"status": "no_bin"},
                    "comparison": {
                        "status": "not_attempted", "disagreement": None,
                    },
                },
            },
            {
                # A pre-integration (schema v2) row: no sim_cpa block.
                "scoring_source": "sim_state_truth",
                "valid": True,
                "truth": {"dist_3d_m": 0.06},
            },
            {
                "scoring_source": "sim_state_truth",
                "valid": True,
                "truth": {"dist_3d_m": 0.07},
                "sim_cpa": _eligible_block(),
            },
        ],
    }), encoding="utf-8")

    report = leg._report(tmp_path, 700.0, 135.4)

    assert report["misses_m"] == [0.07]
    assert report["unscored_runs"] == 3


def test_exit_code_fails_unscored_or_broken_sweeps() -> None:
    scored = {"misses_m": [0.05], "unscored_runs": 0}
    unscored = {"misses_m": [], "unscored_runs": 3}

    assert leg._exit_code([scored, scored], launch_failures=0) == 0
    assert leg._exit_code([scored, unscored], launch_failures=0) == 1
    assert leg._exit_code([scored], launch_failures=1) == 1
    assert leg._exit_code([], launch_failures=0) == 1


def test_report_rejects_boolean_truth_distance(tmp_path: Path) -> None:
    (tmp_path / "summary.json").write_text(json.dumps({
        "results": [
            {
                "scoring_source": "sim_state_truth",
                "valid": True,
                "truth": {"dist_3d_m": True},
                "child": {"snap_3d_m": 0.02},
                "sim_cpa": _eligible_block(),
            },
        ],
    }), encoding="utf-8")

    report = leg._report(tmp_path, 700.0, 135.4)

    assert report["misses_m"] == []
    assert report["unscored_runs"] == 1
