"""Post-flight metric collection and stable evaluator result rows."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from scripts import eval_certificate as cert

from eval_navigation_case_state import CaseState
from eval_navigation_logs import (
    parse_snap_components,
    parse_snap_summary,
    post_snap_poi_seen,
    sidecar_paths,
    snap_from_compact,
)
from eval_navigation_models import MatrixCase
from eval_navigation_vehicle_config import (
    certificate_invalid_reason_for,
    identity_fingerprint,
)


def finalize_case(
    state: CaseState,
    case: MatrixCase,
    index: int,
    attempt: int,
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Collect all descriptive evidence and form one stable CSV/JSON row."""
    metrics = _collect_metrics(state, case, args)
    row = _base_row(state, case, index, attempt, args, metrics)
    row.update(_identity_and_selection_fields(state, metrics))
    row.update(_coordinate_fields(state, metrics))
    row.update(cert.source_time_row_metrics(metrics["source_time"]))
    row.update(metrics["snap"])
    return row


def _collect_metrics(
    state: CaseState,
    case: MatrixCase,
    args: argparse.Namespace,
) -> dict[str, Any]:
    compact = state.paths.compact
    if compact is not None:
        state.paths.debug, sidecar_navigation = sidecar_paths(compact)
        if state.paths.navigation is None:
            state.paths.navigation = sidecar_navigation
    source_time = cert.source_time_summary(
        state.paths.case_dir / cert.SOURCE_TIME_SUBDIR
    )
    try:
        (state.paths.case_dir / "source_time_summary.json").write_text(
            json.dumps(source_time, indent=2), encoding="utf-8"
        )
    except OSError:
        pass
    snap = parse_snap_components(state.paths.debug)
    if not snap:
        snap = parse_snap_summary(snap_from_compact(compact))
    scoring_interval = state.evidence.scoring_interval
    scorer = scoring_interval.scorer if scoring_interval else None
    coordinate_score = scorer.result if scorer else None
    coordinate_error = (
        scorer.certification_error
        if scorer is not None
        else "coordinate scorer did not start"
    )
    if not state.error and coordinate_error:
        state.error = coordinate_error
    measured_clock = (
        scoring_interval.rate_tracker.result
        if scoring_interval is not None
        else cert.ClockRate(None, 0.0, 0, "scoring window never started")
    )
    invalid_reason = certificate_invalid_reason_for(
        state.certificate,
        measured_rate=measured_clock.rate,
        requested_speedup=case.speedup,
        identity=state.evidence.identity,
    )
    if invalid_reason and not state.error:
        state.error = invalid_reason
    snap_distance = snap.get("dist_3d_m")
    snap_passed = isinstance(snap_distance, float) and snap_distance < args.max_distance
    coordinate_passed = bool(
        coordinate_score is not None
        and coordinate_error is None
        and coordinate_score.dist_3d_m < args.max_distance
    )
    post_poi = post_snap_poi_seen(state.paths.navigation)
    gate = state.evidence.gate
    passed = bool(
        not state.error
        and not invalid_reason
        and gate is not None
        and gate.passed
        and snap_passed
        and coordinate_passed
        and post_poi is False
    )
    return {
        "source_time": source_time,
        "snap": snap,
        "scorer": scorer,
        "coordinate_score": coordinate_score,
        "coordinate_error": coordinate_error,
        "measured_clock": measured_clock,
        "invalid_reason": invalid_reason,
        "snap_passed": snap_passed,
        "coordinate_passed": coordinate_passed,
        "post_poi": post_poi,
        "passed": passed,
    }


def _base_row(
    state: CaseState,
    case: MatrixCase,
    index: int,
    attempt: int,
    args: argparse.Namespace,
    metrics: dict[str, Any],
) -> dict[str, Any]:
    expectation = state.evidence.expectation
    verdict = state.processes.launch_verdict
    clock = metrics["measured_clock"]
    return {
        "index": index,
        "attempt": attempt,
        "repetition": attempt if state.certificate else "",
        "certificate_mode": state.certificate,
        "poi_alt_m": case.poi_alt_m,
        "vision_profile": args.vision_profile,
        "detector_type": args.detector_type,
        "poi_wp": args.poi_wp,
        "poi_mission_seq": expectation.mission_seq if expectation else "",
        "expected_lat_deg": expectation.location.lat_deg if expectation else "",
        "expected_lon_deg": expectation.location.lon_deg if expectation else "",
        "expected_rel_alt_m": expectation.location.rel_alt_m if expectation else "",
        "expected_abs_alt_m": expectation.location.abs_alt_m if expectation else "",
        "speedup": case.speedup,
        "launch_measured_clock_rate": verdict.measured_rate if verdict else "",
        "measured_clock_rate": clock.rate if clock.rate is not None else "",
        "measured_clock_span_s": clock.span_s,
        "measured_clock_samples": clock.sample_count,
        "measured_clock_error": clock.error or "",
        "certificate_invalid_reason": metrics["invalid_reason"],
        "navigation_speedup": case.navigation_speedup,
        "wind_speed": case.wind_speed_mps,
        "wind_dir": case.wind_direction_deg,
        "chat": state.processes.chat,
        "sysid": state.processes.sysid,
        "passed": metrics["passed"],
        "coordinate_stream_acknowledged": (
            state.evidence.coordinate_stream_acknowledged
        ),
        "coordinate_stream_live_at_snap": (
            state.evidence.coordinate_stream_live_at_snap
        ),
        "passed_snap_accuracy": metrics["snap_passed"],
        "passed_coordinate_accuracy": metrics["coordinate_passed"],
        "poi_episode_binding_error": state.evidence.episode_error or "",
        "post_snap_poi": metrics["post_poi"],
        "error": state.error,
        "name": state.name,
        "compact": str(state.paths.compact) if state.paths.compact else "",
        "debug": str(state.paths.debug) if state.paths.debug else "",
        "navigation_log": str(state.paths.navigation) if state.paths.navigation else "",
        "case_dir": str(state.paths.case_dir),
    }


def _identity_and_selection_fields(
    state: CaseState,
    metrics: dict[str, Any],
) -> dict[str, Any]:
    selection = state.evidence.selection
    gate = state.evidence.gate
    return {
        **identity_fingerprint(state.evidence.identity),
        "identity_gate_passed": bool(gate and gate.passed),
        "selected_task_id": selection.task_id if selection else "",
        "selected_obj_id": selection.obj_id if selection else "",
        "selected_poi_lat_deg": selection.poi_lat_deg if selection else "",
        "selected_poi_lon_deg": selection.poi_lon_deg if selection else "",
        "selected_poi_abs_alt_m": (
            selection.poi_abs_alt_m if selection else ""
        ),
        "default_ooi_registered": (
            selection.default_ooi_registered if selection else ""
        ),
        "poi_coordinate_error_m": gate.coordinate_error_m if gate else "",
        "poi_altitude_error_m": gate.altitude_error_m if gate else "",
    }


def _coordinate_fields(
    state: CaseState,
    metrics: dict[str, Any],
) -> dict[str, Any]:
    score = metrics["coordinate_score"]
    scorer = metrics["scorer"]
    return {
        "coordinate_dist_3d_m": score.dist_3d_m if score else "",
        "coordinate_h_m": score.horizontal_m if score else "",
        "coordinate_v_m": score.vertical_m if score else "",
        "coordinate_score_error": metrics["coordinate_error"] or "",
        "coordinate_sample_count": scorer.sample_count if scorer else 0,
        "coordinate_received_samples": (
            scorer.received_sample_count if scorer else 0
        ),
        "coordinate_reordered_samples": (
            scorer.reordered_sample_count if scorer else 0
        ),
        "coordinate_duplicate_samples": (
            scorer.duplicate_sample_count if scorer else 0
        ),
        "coordinate_late_samples": scorer.late_sample_count if scorer else 0,
        "coordinate_max_source_gap_s": (
            scorer.max_source_gap_s if scorer else ""
        ),
        "coordinate_max_spatial_gap_m": (
            scorer.max_spatial_gap_m if scorer else ""
        ),
    }
