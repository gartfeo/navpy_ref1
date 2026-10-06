"""Schema and eligibility policy for the verdict's ``sim_cpa`` block.

One home for the block's shape so every producer (live driver, offline
rescore, early-failure paths) emits the same field set and every consumer
(summary counters, campaign batch tooling) reads one predicate instead of
reimplementing it.  The four evidence layers stay separate on purpose:
configuration says what the harness pushed, artifact says which BIN was
bound, evidence says what the module recorded, comparison says how it
relates to the certified stream score.  Collapsing them is how "absent"
and "rejected" become indistinguishable.

Authority is fixed at cross-check-only in this version: nothing in this
block may change ``valid``, ``passed``, ``passed_accuracy``,
``meets_goal``, or ``scoring_source``.  Its only downstream teeth are A/B
eligibility -- campaign statistics exclude rows without an accepted,
agreeing module cross-check (R2).
"""

from __future__ import annotations

import math
from typing import Any

BLOCK_SCHEMA_VERSION = 1
AUTHORITY_CROSS_CHECK = "cross_check_only"

# Versioned disagreement policy: compare d3 and CPA time only.  DH/DV
# deltas are recorded but never flagged -- two minima a few milliseconds
# apart can repartition components by more than a centimetre while the
# 3D distance agrees (observed 14 mm vertical delta on a 3.5 mm d3
# delta, 2026-09-03 validation).
COMPARISON_POLICY_ID = "sim-cpa-stream-crosscheck-v1"
DISAGREE_D3_M = 0.010
DISAGREE_TIME_S = 0.001

STATUS_NOT_ATTEMPTED = "not_attempted"

# configuration.status
CONFIG_CONFIGURED = "configured"
CONFIG_DISABLED = "disabled_by_operator"
CONFIG_UNSUPPORTED = "unsupported_binary"
CONFIG_FAILED = "push_failed"
# An unexpected crash inside the cross-check's own pre-flight code.  The
# flight continues stream-scored (cross-check-only authority); the status
# exists so counters can see the failure instead of a bare error string.
CONFIG_INTERNAL_ERROR = "internal_error"

# artifact.status
ARTIFACT_ACQUIRED = "acquired"
ARTIFACT_NO_BIN = "no_bin"
ARTIFACT_AMBIGUOUS = "ambiguous"
ARTIFACT_ROLLOVER = "rollover_window"
ARTIFACT_EXIT_TIMEOUT = "sitl_exit_timeout"
ARTIFACT_COPY_FAILED = "copy_failed"

# evidence.status
EVIDENCE_ACCEPTED = "accepted"
EVIDENCE_PROVENANCE = "provenance_mismatch"
EVIDENCE_PARSE_FAILED = "parse_failed"
EVIDENCE_NO_SCPC = "no_scpc"
EVIDENCE_TARGET_MISMATCH = "target_mismatch"
EVIDENCE_AMBIGUOUS_EPOCH = "ambiguous_epoch"
EVIDENCE_FAULT = "fault"
EVIDENCE_NO_ROWS = "no_rows"
EVIDENCE_BAD_ARITHMETIC = "bad_interval_arithmetic"
EVIDENCE_INCOMPLETE = "incomplete_coverage"
# An unexpected crash inside post-teardown module scoring: distinct from
# not_attempted so the rejected counter includes it (90_review finding 4).
EVIDENCE_INTERNAL_ERROR = "internal_error"

# comparison.status
COMPARISON_COMPARED = "compared"
COMPARISON_INADMISSIBLE = "inadmissible"
COMPARISON_NO_STREAM = "stream_uncertified"


def default_configuration() -> dict[str, Any]:
    return {
        "mode": None,
        "status": STATUS_NOT_ATTEMPTED,
        "expected_target": None,
        "requested_params": [],
        "acknowledged_params": [],
        "failed_param": None,
        "error": None,
    }


def default_artifact() -> dict[str, Any]:
    return {
        "status": STATUS_NOT_ATTEMPTED,
        "source_path": None,
        "copied_path": None,
        "size_bytes": None,
        "sha256": None,
        "binding": None,
        "error": None,
    }


def default_provenance() -> dict[str, Any]:
    return {
        "arduplane_sha256_before": None,
        "arduplane_sha256_after": None,
        "comparison_policy": COMPARISON_POLICY_ID,
    }


def default_evidence() -> dict[str, Any]:
    return {
        "status": STATUS_NOT_ATTEMPTED,
        "epoch": None,
        "epoch_us": None,
        "selected_rows": None,
        "first_seq": None,
        "last_seq": None,
        "common_episode": None,
        "score": None,
        "raw_epoch_global": None,
        "errors": [],
    }


def default_comparison() -> dict[str, Any]:
    return {
        "status": STATUS_NOT_ATTEMPTED,
        "stream_score": None,
        "deltas": None,
        "thresholds": {"d3_m": DISAGREE_D3_M, "cpa_time_s": DISAGREE_TIME_S},
        "disagreement": None,
        "error": None,
    }


def sim_cpa_block(
    *,
    configuration: dict[str, Any] | None = None,
    artifact: dict[str, Any] | None = None,
    provenance: dict[str, Any] | None = None,
    evidence: dict[str, Any] | None = None,
    comparison: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """The complete block; omitted layers get their not-attempted shape.

    Every verdict -- scored, unscored, failed during setup -- carries this
    block so the field set stays homogeneous (the schema-v2 lesson).
    """
    return {
        "schema_version": BLOCK_SCHEMA_VERSION,
        "authority": AUTHORITY_CROSS_CHECK,
        "configuration": configuration or default_configuration(),
        "artifact": artifact or default_artifact(),
        "provenance": provenance or default_provenance(),
        "evidence": evidence or default_evidence(),
        "comparison": comparison or default_comparison(),
    }


_SCORE_FIELDS = ("d3_m", "dh_m", "dv_m", "cpa_time_s")


def _non_finite_fields(score: dict[str, Any]) -> list[str]:
    return [
        field for field in _SCORE_FIELDS
        if not isinstance(score.get(field), (int, float))
        or isinstance(score.get(field), bool)
        or not math.isfinite(score[field])
    ]


def compare_scores(
    module_score: dict[str, Any],
    stream_score: dict[str, Any],
) -> dict[str, Any]:
    """The comparison layer for an admissible module/stream score pair.

    Signed deltas are module minus stream.  Exactly-at-threshold agrees.
    Non-finite values fail CLOSED here as the last line: ``abs(nan) >
    threshold`` is False, so without this guard a corrupted score would
    read as agreement and pass into A/B eligibility (90_review finding 1).
    """
    bad = [
        f"module.{field}" for field in _non_finite_fields(module_score)
    ] + [
        f"stream.{field}" for field in _non_finite_fields(stream_score)
    ]
    if bad:
        comparison = default_comparison()
        comparison["status"] = COMPARISON_INADMISSIBLE
        comparison["error"] = (
            f"non-finite score value(s) in comparison: {', '.join(bad)}"
        )
        return comparison
    deltas = {
        "d3_m": module_score["d3_m"] - stream_score["d3_m"],
        "dh_m": module_score["dh_m"] - stream_score["dh_m"],
        "dv_m": module_score["dv_m"] - stream_score["dv_m"],
        "cpa_time_s": module_score["cpa_time_s"] - stream_score["cpa_time_s"],
    }
    disagreement = (
        abs(deltas["d3_m"]) > DISAGREE_D3_M
        or abs(deltas["cpa_time_s"]) > DISAGREE_TIME_S
    )
    comparison = default_comparison()
    comparison.update({
        "status": COMPARISON_COMPARED,
        "stream_score": dict(stream_score),
        "deltas": deltas,
        "disagreement": disagreement,
    })
    return comparison


def ab_eligible(result: dict[str, Any]) -> bool:
    """Whether one verdict row may enter cross-checked A/B statistics.

    Requires everything the pre-module campaign required (a valid run
    scored by certified simulator truth) plus an accepted, agreeing module
    cross-check.  This predicate is what keeps a disagreement from being
    ornamental: a disagreed or module-absent row stays a valid run but
    never a campaign statistic.
    """
    if result.get("valid") is not True:
        return False
    if result.get("scoring_source") != "sim_state_truth":
        return False
    block = result.get("sim_cpa")
    if not isinstance(block, dict):
        return False
    evidence = block.get("evidence") or {}
    comparison = block.get("comparison") or {}
    return (
        evidence.get("status") == EVIDENCE_ACCEPTED
        and comparison.get("status") == COMPARISON_COMPARED
        and comparison.get("disagreement") is False
    )


__all__ = [
    "AUTHORITY_CROSS_CHECK",
    "BLOCK_SCHEMA_VERSION",
    "COMPARISON_COMPARED",
    "COMPARISON_INADMISSIBLE",
    "COMPARISON_NO_STREAM",
    "COMPARISON_POLICY_ID",
    "CONFIG_CONFIGURED",
    "CONFIG_DISABLED",
    "CONFIG_FAILED",
    "CONFIG_INTERNAL_ERROR",
    "CONFIG_UNSUPPORTED",
    "DISAGREE_D3_M",
    "DISAGREE_TIME_S",
    "ARTIFACT_ACQUIRED",
    "ARTIFACT_AMBIGUOUS",
    "ARTIFACT_COPY_FAILED",
    "ARTIFACT_EXIT_TIMEOUT",
    "ARTIFACT_NO_BIN",
    "ARTIFACT_ROLLOVER",
    "EVIDENCE_ACCEPTED",
    "EVIDENCE_AMBIGUOUS_EPOCH",
    "EVIDENCE_BAD_ARITHMETIC",
    "EVIDENCE_FAULT",
    "EVIDENCE_INCOMPLETE",
    "EVIDENCE_INTERNAL_ERROR",
    "EVIDENCE_NO_ROWS",
    "EVIDENCE_NO_SCPC",
    "EVIDENCE_PARSE_FAILED",
    "EVIDENCE_PROVENANCE",
    "EVIDENCE_TARGET_MISMATCH",
    "STATUS_NOT_ATTEMPTED",
    "ab_eligible",
    "compare_scores",
    "default_artifact",
    "default_comparison",
    "default_configuration",
    "default_evidence",
    "default_provenance",
    "sim_cpa_block",
]
