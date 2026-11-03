"""Step 12's check of a held-out run against its frozen constants (check_integration_timing.py).

- attitude skew = navpy's associated ATTITUDE time_boot - the frame's represented board time B;
- gimbal skew = the plant time of navpy's associated readbacks (by weight) - the rendered plant time T_r, on the
  emulator's clock. Until step 18 every zoom-capable mount is refused before the gimbal lookup, so for those frames
  the check replays navpy's own ``interpolate_gimbal_at`` on the snapshot the attempt logged (``replayed``); a live
  association must equal its replay.
Every mapped frame must have both results. A pass needs max |skew| + uncertainty <= SKEW_BUDGET_S per source, with
uncertainty = the fit's interval + the join resolution, and the smallest represented interval >= 10 x the largest
PTS interval error plus its resolution. ``observation`` adds the held-out 10-minute rule: >= 600 s of scored frames
per source and no gap between scored frames over COVERAGE_GAP_S.
"""
from __future__ import annotations

import bisect
import math
from collections.abc import Sequence
from typing import Any

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig
from navpy.core.perception.pipeline.integration_association import STATUS_ZOOM_UNSETTLED, interpolate_gimbal_at
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory, GimbalHistorySample
from tools.hitl.encoded_capture_score import INTERVAL_TOLERANCE_NS
from tools.hitl.timing_inputs import Association
from tools.hitl.timing_joins import MISSING, JoinError
from tools.hitl.timing_run import CORRECTION_MATCH_S, Joined
from tools.hitl.timing_stats import summarize

REPORT_SCHEMA = "navpy.hitl.integration_timing_check.v1"
SKEW_BUDGET_S = 0.008                # plan: max skew + join/fit uncertainty <= 8 ms
INTERVAL_RATIO = 10.0                # plan: minimum represented interval >= 10 x max PTS interval error
OBSERVATION_S = 600.0                # plan: held-out 10-minute observation
# My values, not set by the plan:
# - ArduPilot's ATTITUDE time_boot_ms is AP_HAL::millis(), whole milliseconds cut down from the board's micros, so an
#   associated time_boot can sit up to 1 ms below the board time it stands for (one-sided; taken as 1 ms);
ATTITUDE_JOIN_RESOLUTION_S = 0.001
# - the emulator's report and step times are floats on one clock: no resolution term;
GIMBAL_JOIN_RESOLUTION_S = 0.0
# - a hole between scored frames longer than this breaks the observation's coverage (2 x the 0.5 s PTS gap limit).
COVERAGE_GAP_S = 1.0
NS_PER_S = 1e9

GimbalResult = tuple[int, int, float]       # earlier and later joined readback index, weight


def _replay_gimbal(joined: Joined, item: Association, lookup_s: float) -> tuple[GimbalResult | None, str]:
    """navpy's own gimbal lookup on the snapshot this attempt logged, or None, and how it ended."""
    snapshot = item.get("gimbal_history")
    if not isinstance(snapshot, dict) or not snapshot.get("count"):
        return None, "replay_no_history"
    if snapshot.get("session") != joined.gimbal.session:
        return None, "replay_other_session"
    first, last = snapshot["first_receipt_s"], snapshot["last_receipt_s"]
    receipts = joined.gimbal_receipts
    indexes = [index for index in range(bisect.bisect_left(receipts, first), bisect.bisect_right(receipts, last))
               if joined.gimbal.samples[index].order < item.order]
    if len(indexes) != snapshot["count"]:
        raise JoinError(MISSING, f"log line {item.order}: {len(indexes)} logged gimbal receipts, the attempt saw "
                                 f"{snapshot['count']}")
    if item.get("now_s") - last > joined.config.limits.max_source_age_s:
        return None, "replay_source_stale"
    still = Attitude(0.0, 0.0, 0.0)
    samples = tuple(GimbalHistorySample(joined.gimbal.session, joined.gimbal.samples[index].seq,
                                        joined.gimbal.samples[index].receipt_s, still) for index in indexes)
    history = GimbalCaptureHistory(joined.gimbal.session, samples, GimbalData(att=still))
    found, status = interpolate_gimbal_at(history, lookup_s, joined.config.limits.gimbal_bracket_max_s)
    if found is None:
        return None, f"replay_{status}"
    by_receipt = joined.gimbal_index
    return (by_receipt[found.earlier.receipt_s], by_receipt[found.later.receipt_s], found.fraction), "replayed"


def _corrections_match(item: Association, mapped_s: float, config: IntegrationTimingConfig) -> list[str]:
    problems = []
    for name, correction in (("attitude_lookup_s", config.attitude_correction_s),
                             ("gimbal_lookup_s", config.gimbal_correction_s)):
        value = item.get(name)
        if value is not None and abs(float(value) - mapped_s - correction) > CORRECTION_MATCH_S:
            problems.append(f"log line {item.order}: {name} - mapped_s is not the frozen {correction}")
    return problems


def _constants_failures(joined: Joined, constants: dict[str, Any]) -> list[str]:
    failures = []
    if set(constants.get("run_ids", [])) & joined.log.run_ids:
        failures.append("the checked run was used to fit the constants (not held out)")
    for name, key in (("attitude", "attitude_correction_s"), ("gimbal", "gimbal_correction_s")):
        section = constants.get(name, {})
        if section.get("correction_s") != getattr(joined.config, key):
            failures.append(f"run record {key} {getattr(joined.config, key)} is not the fitted constant")
        uncertainty = section.get("uncertainty_s")
        if isinstance(uncertainty, bool) or not isinstance(uncertainty, (int, float)) \
                or not math.isfinite(uncertainty) or uncertainty < 0.0:
            failures.append(f"{name}: fitted uncertainty {uncertainty!r} is not a finite number >= 0")
    return failures


def _gimbal_result(joined: Joined, item: Association, mapped_s: float,
                   failures: list[str]) -> tuple[GimbalResult | None, str]:
    live = item.get("gimbal_receipts_s")
    if live is None and item.status != STATUS_ZOOM_UNSETTLED:
        return None, f"not_looked_up:{item.status}"
    replay, how = _replay_gimbal(joined, item, mapped_s + joined.config.gimbal_correction_s)
    if live is None:
        return replay, how
    try:
        found = (joined.gimbal_index[live[0]], joined.gimbal_index[live[1]], float(item.get("gimbal_weight")))
    except (KeyError, TypeError):
        raise JoinError(MISSING, f"log line {item.order}: associated gimbal receipts or weight not logged")
    if replay is None or replay[:2] != found[:2] or abs(replay[2] - found[2]) > 1e-9:
        failures.append(f"log line {item.order}: live gimbal association differs from its replay")
    return found, "live"


def check(joined: Joined, constants: dict[str, Any], observation: bool) -> dict[str, Any]:
    config = joined.config
    failures: list[str] = [f"map interrupted: {text}" for text in joined.interrupted]
    failures += _constants_failures(joined, constants)
    plant = joined.gimbal.plant_t
    attitude_skew, attitude_t, gimbal_skew, gimbal_t = [], [], [], []
    counts: dict[str, int] = {}
    unscored = {"attitude": 0, "gimbal": 0}
    for item in joined.frames:
        sequence = int(item.get("frame_sequence"))
        mapped_s = float(item.get("mapped_s"))
        failures += _corrections_match(item, mapped_s, config)
        boot = item.get("attitude_time_boot_s")
        if boot is None:
            counts[f"attitude:{item.status}"] = counts.get(f"attitude:{item.status}", 0) + 1
            unscored["attitude"] += 1
            unscored["gimbal"] += 1
            continue
        attitude_skew.append(float(boot) - joined.board_s[sequence])
        attitude_t.append(joined.board_s[sequence])
        result, how = _gimbal_result(joined, item, mapped_s, failures)
        counts[f"gimbal:{how}"] = counts.get(f"gimbal:{how}", 0) + 1
        if result is None:
            unscored["gimbal"] += 1
            continue
        earlier, later, weight = result
        associated = plant[earlier] + weight * (plant[later] - plant[earlier])
        gimbal_skew.append(associated - joined.rendered_t[sequence])
        gimbal_t.append(joined.board_s[sequence])
    report: dict[str, Any] = {"schema": REPORT_SCHEMA, "run_ids": sorted(joined.log.run_ids),
                              "frames_mapped": len(joined.frames), "counts": counts,
                              "sender_frames_unlogged": joined.sender_frames_unlogged,
                              "budget_s": SKEW_BUDGET_S, "observation": observation}
    for name, skews, times, resolution in (("attitude", attitude_skew, attitude_t, ATTITUDE_JOIN_RESOLUTION_S),
                                            ("gimbal", gimbal_skew, gimbal_t, GIMBAL_JOIN_RESOLUTION_S)):
        if unscored[name]:
            failures.append(f"{name}: {unscored[name]} of {len(joined.frames)} mapped frames have no result")
        if not skews:
            failures.append(f"{name}: no associated frames")
            report[name] = {"count": 0, "unscored": unscored[name]}
            continue
        summary = summarize(skews)
        fitted = constants.get(name, {}).get("uncertainty_s")
        uncertainty = (float(fitted) if isinstance(fitted, (int, float)) else math.inf) + resolution
        section = {**summary.record(), "unscored": unscored[name], "uncertainty_s": uncertainty,
                   "span_s": max(times) - min(times), "max_gap_s": _max_gap(times)}
        if not summary.max_abs_s + uncertainty <= SKEW_BUDGET_S:
            failures.append(f"{name}: max |skew| {summary.max_abs_s * 1e3:.3f} ms + uncertainty "
                            f"{uncertainty * 1e3:.3f} ms is over {SKEW_BUDGET_S * 1e3:g} ms")
        if observation:
            if section["span_s"] < OBSERVATION_S:
                failures.append(f"{name}: scored frames span {section['span_s']:.1f} s, under {OBSERVATION_S:g} s")
            if section["max_gap_s"] > COVERAGE_GAP_S:
                failures.append(f"{name}: a {section['max_gap_s']:.3f} s hole between scored frames")
        report[name] = section
    report["intervals"] = _intervals(joined, failures)
    report["failures"] = failures
    report["passed"] = not failures
    return report


def _max_gap(times: Sequence[float]) -> float:
    ordered = sorted(times)
    return max((later - earlier for earlier, later in zip(ordered, ordered[1:])), default=0.0)


def _intervals(joined: Joined, failures: list[str]) -> dict[str, float]:
    """Consecutive mapped frames: navpy's PTS interval against the represented board-time interval."""
    ordered = sorted(joined.frames, key=lambda item: int(item.get("frame_sequence")))
    represented, errors = [], []
    for earlier, later in zip(ordered, ordered[1:]):
        first, second = int(earlier.get("frame_sequence")), int(later.get("frame_sequence"))
        board = joined.board_s[second] - joined.board_s[first]
        pts = (int(later.get("pts_ns")) - int(earlier.get("pts_ns"))) / NS_PER_S
        represented.append(board)
        errors.append(abs(pts - board))
    if not represented:
        failures.append("intervals: fewer than two mapped frames")
        return {}
    resolution = INTERVAL_TOLERANCE_NS / NS_PER_S
    worst = max(errors) + resolution
    smallest = min(represented)
    if smallest < INTERVAL_RATIO * worst:
        failures.append(f"intervals: smallest represented {smallest * 1e3:.3f} ms is under {INTERVAL_RATIO:g} x "
                        f"{worst * 1e3:.4f} ms")
    return {"min_represented_s": smallest, "max_pts_error_s": max(errors), "resolution_s": resolution}


__all__ = ["COVERAGE_GAP_S", "OBSERVATION_S", "REPORT_SCHEMA", "SKEW_BUDGET_S", "check"]
