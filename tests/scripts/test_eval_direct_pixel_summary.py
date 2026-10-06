"""Sweep-summary counting: accuracy statistics never absorb validity."""

from __future__ import annotations

from scripts import eval_direct_pixel_summary as summary_module


def _row(meets_goal, passed=True, source="sim_state_truth",
         passed_accuracy=True, valid=True):
    return {
        "passed": passed,
        "valid": valid,
        "passed_accuracy": passed_accuracy,
        "meets_goal": meets_goal,
        "scoring_source": source,
    }


def test_summary_counts_goal_only_from_scored_runs() -> None:
    summary = summary_module.summarize(
        [
            _row(True),
            _row(False),
            _row(None, passed=False, passed_accuracy=None, source="none"),
        ],
        gate_m=1.0,
        goal_m=0.1,
        source_identity={"sha256": "abc", "files": 1},
        aborted_identity=None,
    )
    assert summary["schema_version"] == summary_module.SUMMARY_SCHEMA_VERSION
    assert summary["runs"] == 3
    assert summary["runs_within_gate"] == 2
    assert summary["runs_gate_unscored"] == 1
    assert summary["runs_within_goal"] == 1
    assert summary["runs_goal_unscored"] == 1
    assert summary["truth_scored_runs"] == 2
    assert summary["estimate_scored_runs"] == 0
    assert summary["passed"] is False


def test_summary_gate_counts_accuracy_not_overall_validity() -> None:
    """The live 10x case: certified truth CPA within the gate, but the run
    invalidated by the freshness floor. The gate counter is an accuracy
    statistic -- the miss distance was measured and was inside the gate --
    while overall `passed` stays False."""
    summary = summary_module.summarize(
        [_row(True, passed=False, valid=False)],
        gate_m=1.0,
        goal_m=0.1,
        source_identity={"sha256": "abc", "files": 1},
        aborted_identity=None,
    )
    assert summary["runs_within_gate"] == 1
    assert summary["runs_gate_unscored"] == 0
    assert summary["passed"] is False


def test_summary_counts_estimate_scored_runs_separately() -> None:
    summary = summary_module.summarize(
        [_row(True, source="ekf_snap_estimate"), _row(False, source="ekf_snap_estimate")],
        gate_m=1.0,
        goal_m=0.1,
        source_identity={"sha256": "abc", "files": 1},
        aborted_identity=None,
    )
    assert summary["truth_scored_runs"] == 0
    assert summary["estimate_scored_runs"] == 2
    assert summary["runs_goal_unscored"] == 0


def _module_row(config="configured", artifact="not_attempted",
                evidence="not_attempted", comparison="not_attempted",
                disagreement=None, **row_kwargs):
    row = _row(True, **row_kwargs)
    row["sim_cpa"] = {
        "schema_version": 1,
        "authority": "cross_check_only",
        "configuration": {"status": config},
        "artifact": {"status": artifact},
        "provenance": {},
        "evidence": {"status": evidence},
        "comparison": {"status": comparison, "disagreement": disagreement},
    }
    return row


def test_summary_v3_counters_cover_the_whole_failure_taxonomy() -> None:
    """R12 + 90_review finding 4: every stage failure has a counter, so a
    degraded batch is legible from the summary alone -- including the
    artifact stage and internal cross-check crashes."""
    rows = [
        # 1: the clean cross-checked run -- the only A/B-eligible row.
        _module_row(artifact="acquired", evidence="accepted",
                    comparison="compared", disagreement=False),
        # 2: compared but disagreeing -- counted, never eligible.
        _module_row(artifact="acquired", evidence="accepted",
                    comparison="compared", disagreement=True),
        # 3-5: the three no-evidence configuration outcomes.
        _module_row(config="disabled_by_operator"),
        _module_row(config="unsupported_binary"),
        _module_row(config="push_failed"),
        # 6: pre-flight cross-check crash (internal_error configuration).
        _module_row(config="internal_error"),
        # 7: configured but the BIN never became evidence.
        _module_row(artifact="no_bin"),
        # 8: bound BIN whose evidence failed certification.
        _module_row(artifact="acquired", evidence="bad_interval_arithmetic"),
        # 9: post-teardown module crash (internal_error evidence).
        _module_row(evidence="internal_error"),
        # 10: a pre-v3 row without any block counts nowhere.
        _row(True),
    ]
    summary = summary_module.summarize(
        rows,
        gate_m=1.0,
        goal_m=0.1,
        source_identity={"sha256": "abc", "files": 1},
        aborted_identity=None,
    )
    assert summary["module_configured_runs"] == 5
    assert summary["module_disabled_runs"] == 1
    assert summary["module_unsupported_runs"] == 1
    assert summary["module_config_failed_runs"] == 2
    assert summary["module_artifact_failed_runs"] == 1
    assert summary["module_evidence_accepted_runs"] == 2
    assert summary["module_evidence_rejected_runs"] == 2
    assert summary["module_compared_runs"] == 2
    assert summary["module_disagreed_runs"] == 1
    assert summary["ab_eligible_runs"] == 1


def test_summary_abort_never_reads_clean() -> None:
    summary = summary_module.summarize(
        [_row(True)],
        gate_m=1.0,
        goal_m=0.1,
        source_identity={"sha256": "abc", "files": 1},
        aborted_identity={"sha256": "def", "files": 1},
    )
    assert summary["passed"] is False
    assert summary["runs_within_gate"] == 1
