"""Verdict assembly and scoring policy for the direct-pixel harness.

Split from ``eval_direct_pixel_pn.run_case`` so the accuracy policy has one
home: which source is authoritative for the score, what certifies it, and how
``passed`` / ``passed_accuracy`` / ``meets_goal`` derive from it.  The
2026-09-03 proof pass showed the EKF-derived
snap/coordinate numbers are the EKF's error projection and ranked runs
opposite to simulator truth, so under the default ``sitl-truth`` policy the
certified SIM_STATE truth CPA decides accuracy and the EKF numbers demote to
labeled diagnostics with recorded warnings.  ``vehicle-estimate`` preserves
the legacy EKF-gated behavior for runs with no simulator (real hardware).
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Do it here rather than relying on another script
# having been imported first: without this the module (and its test) fails
# standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_direct_pixel_validity import validity_errors  # noqa: E402
from eval_sim_cpa_block import sim_cpa_block  # noqa: E402


SCORING_POLICY_SITL_TRUTH = "sitl-truth"
SCORING_POLICY_VEHICLE_ESTIMATE = "vehicle-estimate"
SCORING_POLICIES = (SCORING_POLICY_SITL_TRUTH, SCORING_POLICY_VEHICLE_ESTIMATE)

SCORING_SOURCE_TRUTH = "sim_state_truth"
SCORING_SOURCE_ESTIMATE = "ekf_snap_estimate"
SCORING_SOURCE_NONE = "none"


def _finite_number(value: Any) -> float | None:
    """The value as a float when it is a real, finite number; else None.

    ``bool`` is an ``int`` subclass, so a bare isinstance check would accept
    ``True`` as a 1.0-metre miss.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _ekf_gate_texts(
    child_result: dict[str, Any],
    closest: Any,
    max_distance_m: float,
) -> list[str]:
    """The legacy snap/coordinate gate findings, source-labeled."""
    texts: list[str] = []
    snap = _finite_number(child_result.get("snap_3d_m"))
    if snap is None or snap > max_distance_m:
        raw = child_result.get("snap_3d_m")
        texts.append(f"SNAP {raw!r} exceeds {max_distance_m:g}m")
    if closest is None or closest.dist_3d_m > max_distance_m:
        measured = None if closest is None else closest.dist_3d_m
        texts.append(
            f"coordinate CPA {measured!r} exceeds {max_distance_m:g}m"
        )
    return texts


def _truth_accuracy(
    truth_block: dict[str, Any] | None,
    max_distance_m: float,
    goal_distance_m: float,
) -> tuple[list[str], bool | None, bool | None]:
    """Accuracy errors, passed_accuracy, and meets_goal under sitl-truth.

    The stream-request ACK is deliberately not consulted: the observed,
    certified stream is authoritative, and a lost ACK must not fail a run
    whose truth track certifies.  The ACK outcome is recorded as data on the
    verdict instead.
    """
    if truth_block is None:
        return (
            ["truth scoring unavailable: no SIM_STATE recorder was attached"],
            None,
            None,
        )
    certification = truth_block.get("certification_error")
    if certification:
        return [f"truth scoring uncertified: {certification}"], None, None
    distance = _finite_number(truth_block.get("dist_3d_m"))
    if distance is None:
        return ["truth scoring produced no closest approach"], None, None
    errors: list[str] = []
    if distance > max_distance_m:
        errors.append(
            f"truth CPA {distance:.3f}m exceeds {max_distance_m:g}m"
        )
    return errors, distance <= max_distance_m, distance < goal_distance_m


def scoring_decision(
    *,
    scoring_policy: str,
    child_result: dict[str, Any],
    closest: Any,
    truth_block: dict[str, Any] | None,
    max_distance_m: float,
    goal_distance_m: float,
) -> dict[str, Any]:
    """Accuracy outcome for one case under the selected policy."""
    ekf_texts = _ekf_gate_texts(child_result, closest, max_distance_m)
    if scoring_policy == SCORING_POLICY_SITL_TRUTH:
        accuracy, passed_accuracy, meets_goal = _truth_accuracy(
            truth_block, max_distance_m, goal_distance_m
        )
        return {
            "accuracy_errors": accuracy,
            "ekf_gate_warnings": ekf_texts,
            "passed_accuracy": passed_accuracy,
            "meets_goal": meets_goal,
            "scoring_source": (
                SCORING_SOURCE_TRUTH
                if passed_accuracy is not None
                else SCORING_SOURCE_NONE
            ),
        }
    snap = _finite_number(child_result.get("snap_3d_m"))
    return {
        "accuracy_errors": list(ekf_texts),
        "ekf_gate_warnings": [],
        "passed_accuracy": not ekf_texts,
        "meets_goal": snap is not None and snap < goal_distance_m,
        "scoring_source": SCORING_SOURCE_ESTIMATE,
    }


def persist_verdict(case_dir: Path, result: dict[str, Any]) -> None:
    """Atomically write ``verdict.json`` (write-then-replace).

    The verdict is now written after post-teardown module analysis, so a
    crash mid-write must never leave a truncated file where a complete
    stream verdict could have stood.
    """
    import os

    path = case_dir / "verdict.json"
    staging = case_dir / "verdict.json.tmp"
    staging.write_text(json.dumps(result, indent=2), encoding="utf-8")
    os.replace(staging, path)


def unscored_result(
    errors: list[str],
    *,
    scoring_policy: str,
    truth_stream_acknowledged: bool | None = None,
    truth_block: dict[str, Any] | None = None,
    child_result: dict[str, Any] | None = None,
    scorer: Any | None = None,
    case_dir: Path | None = None,
    sim_cpa: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """A schema-complete result for a case that failed before scoring.

    Early bailouts (invalid configuration, setup exceptions) must produce the
    core ``case_verdict`` field set so the summary stays structurally
    homogeneous and consumers can tell "failed before scoring" from an
    old-schema row. Flight-evidence blocks (ground track, stability,
    freshness, causality) exist only when a flight produced them.

    A late failure must not erase what was observed: the caller passes the
    real stream ACK and whatever the flight already produced (recorder
    verdict, child result, coordinate scorer) -- ``None`` still means
    "never requested" / "never produced". The row stays unscored either
    way; these fields are audit data, labeled by ``scoring_source: none``.
    With ``case_dir`` the result is also persisted as ``verdict.json``, so
    a crashed case leaves the same audit artifact a scored one does.
    """
    closest = None if scorer is None else scorer.result
    result = {
        "passed": False,
        "valid": False,
        "passed_accuracy": None,
        "meets_goal": None,
        "scoring_policy": scoring_policy,
        "scoring_source": SCORING_SOURCE_NONE,
        "errors": list(errors),
        "ekf_gate_warnings": [],
        "truth_stream_acknowledged": truth_stream_acknowledged,
        "child": child_result,
        "coordinate": None if closest is None else asdict(closest),
        "coordinate_samples": 0 if scorer is None else scorer.sample_count,
        "truth": truth_block,
        "ekf_truth_divergence_m": None,
        # Present on EVERY verdict, early failures included, so the field
        # set stays homogeneous; a default block reads as not-attempted.
        "sim_cpa": sim_cpa or sim_cpa_block(),
    }
    if case_dir is not None:
        persist_verdict(case_dir, result)
    return result


def case_verdict(
    case_dir: Path,
    *,
    child_result: dict[str, Any],
    scorer: Any,
    track: Any,
    truth: Any | None,
    truth_stream_accepted: bool | None,
    scoring_policy: str,
    max_distance_m: float,
    goal_distance_m: float,
    wind_speed: float,
    wind_dir_deg: float,
    speedup: float,
    sim_cpa: dict[str, Any] | None = None,
    persist: bool = True,
) -> dict[str, Any]:
    """Assemble one case's verdict; persist unless the caller defers.

    The single-UAV driver defers persistence (``persist=False``) so the
    SIM_CPA cross-check -- which must only read a BIN after the SITL
    process is gone -- can be folded in before the one atomic write.  The
    module block never changes any other field here: cross-check-only
    authority is enforced by construction.
    """
    closest = scorer.result
    coordinate = None if closest is None else asdict(closest)
    validity, evidence = validity_errors(
        case_dir,
        child_result=child_result,
        scorer=scorer,
        track=track,
        wind_speed=wind_speed,
        wind_dir_deg=wind_dir_deg,
        speedup=speedup,
    )
    truth_block = None if truth is None else truth.verdict()
    decision = scoring_decision(
        scoring_policy=scoring_policy,
        child_result=child_result,
        closest=closest,
        truth_block=truth_block,
        max_distance_m=max_distance_m,
        goal_distance_m=goal_distance_m,
    )
    truth_distance = (
        None if truth_block is None
        else _finite_number(truth_block.get("dist_3d_m"))
    )
    coordinate_distance = None if closest is None else closest.dist_3d_m
    errors = validity + decision["accuracy_errors"]
    result = {
        "passed": not errors,
        # An authoritative score must exist for the run to be valid: a
        # certified over-gate miss is valid (and fails accuracy), but an
        # uncertified or missing score is not.
        "valid": not validity and decision["passed_accuracy"] is not None,
        "passed_accuracy": decision["passed_accuracy"],
        "meets_goal": decision["meets_goal"],
        "scoring_policy": scoring_policy,
        "scoring_source": decision["scoring_source"],
        "errors": errors,
        "ekf_gate_warnings": decision["ekf_gate_warnings"],
        "truth_stream_acknowledged": truth_stream_accepted,
        "child": child_result,
        "coordinate": coordinate,
        "coordinate_samples": scorer.sample_count,
        "truth": truth_block,
        "ekf_truth_divergence_m": (
            None
            if truth_distance is None or coordinate_distance is None
            else coordinate_distance - truth_distance
        ),
        "sim_cpa": sim_cpa or sim_cpa_block(),
        **evidence,
    }
    if persist:
        persist_verdict(case_dir, result)
    return result


__all__ = [
    "SCORING_POLICIES",
    "SCORING_POLICY_SITL_TRUTH",
    "SCORING_POLICY_VEHICLE_ESTIMATE",
    "case_verdict",
    "persist_verdict",
    "scoring_decision",
    "unscored_result",
]
