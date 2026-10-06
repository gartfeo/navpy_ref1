"""Sweep-level summary for the direct-pixel harness.

Split from ``eval_direct_pixel_verdict`` (which owns the PER-CASE scoring
policy) so each file keeps one concern: this module only counts and labels
finished case rows.  The counters follow the same separation the case verdict
records -- accuracy statistics never absorb validity: a run invalidated for a
non-accuracy reason (the live 10x freshness case, 2026-09-03) still hit or
missed the gate, and an unscored run counts as unscored, never as a miss.
Overall sweep health is the ``passed`` flag alone.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path (same standalone-import guard as the siblings).
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_direct_pixel_verdict import (  # noqa: E402
    SCORING_SOURCE_ESTIMATE, SCORING_SOURCE_TRUTH,
)
from eval_sim_cpa_block import (  # noqa: E402
    ARTIFACT_ACQUIRED, COMPARISON_COMPARED, CONFIG_CONFIGURED,
    CONFIG_DISABLED, CONFIG_FAILED, CONFIG_INTERNAL_ERROR,
    CONFIG_UNSUPPORTED, EVIDENCE_ACCEPTED, STATUS_NOT_ATTEMPTED, ab_eligible,
)

# v3: SIM_CPA cross-check coverage and disagreement counters are
# machine-significant -- campaign tooling reads them to decide whether a
# batch has admissible cross-checked coverage at all.
SUMMARY_SCHEMA_VERSION = 3


def _module_counters(results: list[dict[str, Any]]) -> dict[str, int]:
    """Coverage/disagreement counters over each row's ``sim_cpa`` block.

    Absent and rejected stay different states: a rejected-evidence row
    had a configured module and a bound BIN that failed certification,
    while an unsupported or disabled row never produced evidence at all.
    """
    def blocks() -> list[dict[str, Any]]:
        return [
            row["sim_cpa"] for row in results
            if isinstance(row.get("sim_cpa"), dict)
        ]

    def config_status(block: dict[str, Any]) -> Any:
        return (block.get("configuration") or {}).get("status")

    def evidence_status(block: dict[str, Any]) -> Any:
        return (block.get("evidence") or {}).get("status")

    def comparison(block: dict[str, Any]) -> dict[str, Any]:
        return block.get("comparison") or {}

    return {
        "module_configured_runs": sum(
            1 for block in blocks()
            if config_status(block) == CONFIG_CONFIGURED
        ),
        "module_disabled_runs": sum(
            1 for block in blocks()
            if config_status(block) == CONFIG_DISABLED
        ),
        "module_unsupported_runs": sum(
            1 for block in blocks()
            if config_status(block) == CONFIG_UNSUPPORTED
        ),
        "module_config_failed_runs": sum(
            1 for block in blocks()
            if config_status(block)
            in (CONFIG_FAILED, CONFIG_INTERNAL_ERROR)
        ),
        # Configured runs whose BIN never became evidence: binding lost,
        # SITL exit unconfirmed, no/ambiguous successor, or a failed copy
        # (R12's artifact-failed counter; 90_review finding 4).
        "module_artifact_failed_runs": sum(
            1 for block in blocks()
            if (block.get("artifact") or {}).get("status")
            not in (ARTIFACT_ACQUIRED, STATUS_NOT_ATTEMPTED)
        ),
        "module_evidence_accepted_runs": sum(
            1 for block in blocks()
            if evidence_status(block) == EVIDENCE_ACCEPTED
        ),
        "module_evidence_rejected_runs": sum(
            1 for block in blocks()
            if evidence_status(block)
            not in (EVIDENCE_ACCEPTED, STATUS_NOT_ATTEMPTED)
        ),
        "module_compared_runs": sum(
            1 for block in blocks()
            if comparison(block).get("status") == COMPARISON_COMPARED
        ),
        "module_disagreed_runs": sum(
            1 for block in blocks()
            if comparison(block).get("disagreement") is True
        ),
        "ab_eligible_runs": sum(
            1 for row in results if ab_eligible(row)
        ),
    }


def summarize(
    results: list[dict[str, Any]],
    *,
    gate_m: float,
    goal_m: float,
    source_identity: dict[str, Any],
    aborted_identity: dict[str, Any] | None,
) -> dict[str, Any]:
    """Sweep summary; a truncated or unscored sweep must never read clean."""
    return {
        "schema_version": SUMMARY_SCHEMA_VERSION,
        # A truncated sweep must never read as a clean one, so the abort is a
        # failure of the sweep even when every completed run passed.
        "passed": (
            aborted_identity is None
            and bool(results)
            and all(row["passed"] for row in results)
        ),
        "gate_m": gate_m,
        "goal_m": goal_m,
        "runs": len(results),
        "runs_within_gate": sum(
            1 for row in results if row.get("passed_accuracy") is True
        ),
        "runs_gate_unscored": sum(
            1 for row in results if row.get("passed_accuracy") is None
        ),
        "runs_within_goal": sum(
            1 for row in results if row.get("meets_goal") is True
        ),
        "runs_goal_unscored": sum(
            1 for row in results if row.get("meets_goal") is None
        ),
        "truth_scored_runs": sum(
            1 for row in results
            if row.get("scoring_source") == SCORING_SOURCE_TRUTH
        ),
        "estimate_scored_runs": sum(
            1 for row in results
            if row.get("scoring_source") == SCORING_SOURCE_ESTIMATE
        ),
        **_module_counters(results),
        "source_identity": source_identity,
        "aborted_source_identity": aborted_identity,
        "results": results,
    }


__all__ = [
    "SUMMARY_SCHEMA_VERSION",
    "summarize",
]
