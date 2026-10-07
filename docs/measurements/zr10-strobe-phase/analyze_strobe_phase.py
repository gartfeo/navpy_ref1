#!/usr/bin/env python3
"""Turn strobe-phase JSONL into an offset estimate with a paper trail.

Reads the JSONL that the EXISTING `measure_zr10_latency.py` writes -
this bench adds no acquisition code. Writes one summary JSON per
session and FAILS CLOSED: the exit code is nonzero unless a valid
primary estimate exists, so a wasted Jetson session is discovered at
analysis time, not after the rig is torn down.

The reported quantity is the effective optical-response offset
relative to THIS session's fitted PTS->publication affine map, in ONE
canonical representation: wrapped modulo the session's median frame
period, alongside that period (review finding - the split crossing is
naturally unwrapped and the band center naturally wrapped, and
subtracting mixed representations faked whole-period jumps). Wrapping
also means a between-condition shift of EXACTLY one frame period would
alias to zero; PROTOCOL.md records why that is physically implausible
here.

`--compare` reads several summaries: per-session canonical offsets,
CIRCULAR deltas against the first VALID session, and the A-B-A
contrast for a three-block S3 run. Invalid sessions are listed but
excluded from every contrast.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from strobe_phase_estimator import (
    AffineFit,
    Frame,
    InsufficientData,
    Pulse,
    affine_fit,
    classify_pulses,
    linear_split_crossing_ns,
    median_period_ns,
    miss_band,
    split_crossing_ns,
)

# Coarse prior from the accepted 08-30 run: light lands in the frame
# PUBLISHED ~100 ms after the GPIO edge. A wrong prior surfaces as
# unexpected_lit and lost splits, never as a shifted answer.
DEFAULT_PRIOR_OFFSET_NS = 100_000_000
TERMINAL_GUARD_FRAMES = 8
# Predeclared sensitivity scan around the chosen prior. The scan NEVER
# picks the best-looking prior; it demands the active estimator hold
# still across every admissible one. It probes PRIOR-ASSIGNMENT
# sensitivity only - a genuine temporal drift common to every prior
# passes it (review finding); session drift needs time blocks.
PRIOR_SCAN_STEPS_MS = (-20, -15, -10, -5, 0, 5, 10, 15, 20)
PLATEAU_TOLERANCE_NS = 1_000_000
ESTIMATOR_AGREEMENT_NS = 1_000_000
CROSS_REGIME_TOLERANCE_NS = 1_500_000
# Worst per-block map divergence, in TIME. 1 ms matches the phase
# budget of the estimator gates; a ppm slope bound would allow ~20 ms
# of phase drift across a 20 s block (review finding).
MAP_DIVERGENCE_TOLERANCE_NS = 1_000_000
SATURATION_RED_P95 = 250.0
MAX_ROI_HEIGHT_PX = 40
# Topological, not statistical: one dead-time interval per period admits
# exactly one circular miss run, so a coherent band contains every miss.
# Zero is the model's own answer - any other number would be tuning.
MAX_MISSES_OUTSIDE_BAND = 0


def _wrap(value_ns: float, period_ns: float) -> float:
    return value_ns % period_ns


def _circular_delta(a_ns: float, b_ns: float, period_ns: float) -> float:
    """Shortest signed b-minus-a on the circle."""
    delta = (b_ns - a_ns) % period_ns
    return delta - period_ns if delta > period_ns / 2 else delta


def load_session(path: Path) -> tuple[list[Frame], list[Pulse], dict]:
    frames, on_edges, off_edges, meta = [], {}, {}, {}
    rejected = malformed = saturated = 0
    edge_spans = []
    last_pts = None
    with path.open(encoding="utf-8") as fp:
        for line in fp:
            if not line.strip():
                continue
            row = json.loads(line)
            kind = row.get("event")
            if kind == "frame":
                pts = row.get("pts_ns")
                if pts is None or (last_pts is not None and pts <= last_pts):
                    rejected += 1  # duplicate or regressing PTS
                    continue
                last_pts = pts
                if row.get("roi_red_p95", 0.0) >= SATURATION_RED_P95:
                    saturated += 1
                frames.append(
                    Frame(pts, row["publication_mono_ns"], row["roi_score"])
                )
            elif kind == "gpio":
                target = on_edges if row["value"] == 1 else off_edges
                target[row["event_id"]] = row
                edge_spans.append(
                    row["post_write_mono_ns"] - row["pre_write_mono_ns"]
                )
            elif kind == "run_start":
                meta = {"argv": row.get("argv"), "roi": row.get("roi"),
                        "versions": row.get("versions"),
                        "random_seed": row.get("random_seed")}
    pulses = []
    for event_id, on in sorted(on_edges.items()):
        off = off_edges.get(event_id)
        if off is None or off["pre_write_mono_ns"] <= on["post_write_mono_ns"]:
            malformed += 1
            continue
        pulses.append(
            Pulse(event_id, on["post_write_mono_ns"], off["pre_write_mono_ns"])
        )
    malformed += sum(1 for e in off_edges if e not in on_edges)
    edge_spans.sort()
    diagnostics = {
        "frames_rejected_bad_pts": rejected,
        "frames_saturated": saturated,
        "gpio_pairs_malformed": malformed,
        "gpio_edge_span_us": {
            "median": edge_spans[len(edge_spans) // 2] / 1e3,
            "max": edge_spans[-1] / 1e3,
        } if edge_spans else None,
    }
    return frames, pulses, {"meta": meta, "diagnostics": diagnostics}


def drop_terminal_pulses(
    frames: list[Frame], pulses: list[Pulse], fit: AffineFit,
    prior_offset_ns: float,
) -> list[Pulse]:
    """Acquisition stops right after the last OFF edge (review finding):
    the final pulses may lack the frames the baseline and pairing need."""
    if not frames:
        return []
    horizon = fit.map_pts(frames[-1].pts_ns)
    period = ((horizon - fit.map_pts(frames[0].pts_ns))
              / max(1, len(frames) - 1))
    cutoff = horizon - TERMINAL_GUARD_FRAMES * period
    return [p for p in pulses if p.center_ns + prior_offset_ns < cutoff]


def _estimate(frames, pulses, fit, prior_ns, lit_threshold):
    classified = classify_pulses(frames, pulses, fit, prior_ns, lit_threshold)
    counts: dict[str, int] = {}
    for c in classified:
        counts[c.kind] = counts.get(c.kind, 0) + 1
    return classified, counts


def _band_is_coherent(band) -> bool:
    """One dead-time interval per period admits ONE circular miss run,
    so a coherent band holds every miss and no lit sample."""
    return (band.misses_outside <= MAX_MISSES_OUTSIDE_BAND
            and band.lit_inside == 0)


def _primary(classified, pulse_width_ns: float, period_ns: float):
    """The active estimator's wrapped M, with its name. None if neither
    reaches its declared minimum."""
    try:
        crossing, lo, hi = split_crossing_ns(classified)
        return _wrap(crossing, period_ns), "split", (lo, hi)
    except InsufficientData:
        pass
    try:
        band = miss_band(classified, pulse_width_ns, period_ns)
        # An incoherent band is forensic evidence, never an estimate: it
        # must not reach canonical, the half-session drift, or the prior
        # scan's admissible set, or a rejected session would still be
        # publishing a number (review finding).
        if not _band_is_coherent(band):
            return None, None, None
        return _wrap(band.center_ns, period_ns), "miss_band", None
    except InsufficientData:
        return None, None, None


def _prior_scan(
    frames, pulses, fit, prior_ns, lit_threshold, pulse_width_ns, period_ns
) -> dict:
    rows, values = [], []
    for step_ms in PRIOR_SCAN_STEPS_MS:
        prior = prior_ns + step_ms * 1_000_000
        classified, counts = _estimate(
            frames, pulses, fit, prior, lit_threshold
        )
        row = {"step_ms": step_ms, "classes": counts}
        if counts.get("unexpected_lit", 0) == 0:
            value, source, _ = _primary(classified, pulse_width_ns, period_ns)
            row["offset_wrapped_ms"] = (
                value / 1e6 if value is not None else None
            )
            row["source"] = source
            if value is not None:
                values.append(value)
        rows.append(row)
    # Whole-frame pair reassignment between priors is legal, so the
    # spread is circular: the shortest arc containing every estimate.
    plateau = None
    if len(values) > 1:
        wrapped = sorted(v % period_ns for v in values)
        gaps = [b - a for a, b in zip(wrapped, wrapped[1:])]
        gaps.append(wrapped[0] + period_ns - wrapped[-1])
        plateau = period_ns - max(gaps)
    return {
        "rows": rows,
        "admissible_estimates": len(values),
        "plateau_spread_ms": plateau / 1e6 if plateau is not None else None,
        "stable": plateau is not None and plateau <= PLATEAU_TOLERANCE_NS,
        "note": ("prior-assignment sensitivity only; a temporal drift "
                 "common to every prior passes this - it is not a "
                 "session-drift test"),
    }


def _physical_checks(result: dict, failures: list) -> None:
    roi = (result.get("meta") or {}).get("roi")
    if roi and len(roi) == 4 and roi[3] > MAX_ROI_HEIGHT_PX:
        failures.append(
            f"ROI height {roi[3]} px exceeds {MAX_ROI_HEIGHT_PX}: "
            "rolling-shutter smear voids the phase boundary (PROTOCOL)"
        )
    saturated = result["diagnostics"]["frames_saturated"]
    if saturated:
        failures.append(
            f"{saturated} frame(s) saturated (roi_red_p95 >= "
            f"{SATURATION_RED_P95}): a clipped ROI destroys the score "
            "split; reduce LED current or diffuse (PROTOCOL)"
        )


def analyze(path: Path, lit_threshold: float, prior_offset_ns: float) -> dict:
    frames, pulses, extra = load_session(path)
    fit = affine_fit(frames)
    period_ns = median_period_ns(frames, fit)
    kept = drop_terminal_pulses(frames, pulses, fit, prior_offset_ns)
    classified, counts = _estimate(
        frames, kept, fit, prior_offset_ns, lit_threshold
    )
    widths = sorted(p.width_ns for p in kept)
    median_width = float(widths[len(widths) // 2]) if widths else 0.0
    failures: list[str] = []
    result = {
        "input": str(path), **extra,
        "frames": len(frames), "pulses_total": len(pulses),
        "pulses_analyzed": len(kept),
        "pulses_terminal_excluded": len(pulses) - len(kept),
        "frame_period_ms": period_ns / 1e6,
        "affine": {"alpha": fit.alpha, "beta_ns": fit.beta,
                   "residual_sd_ms": fit.residual_sd_ns / 1e6,
                   "block_alphas": list(fit.block_alphas),
                   "block_map_divergence_ms": fit.block_divergence_ns / 1e6},
        "classes": counts, "lit_threshold": lit_threshold,
        "prior_offset_ms": prior_offset_ns / 1e6,
    }
    lit = [c for c in classified if c.kind in ("split", "single")]
    if lit:
        # Median event-to-publication latency of the LIT frame. Coarse
        # (carries the pipeline), but good to a fraction of a period -
        # exactly what compare() needs to restore the integer-period
        # branch a wrapped offset cannot carry (review finding: without
        # it a whole-period shift between conditions aliases to zero).
        lat = sorted(
            (c.earlier if c.deposit_earlier >= c.deposit_later else c.later
             ).publication_mono_ns - c.pulse.center_ns
            for c in lit
        )
        result["coarse_latency_ms"] = lat[len(lat) // 2] / 1e6
    else:
        result["coarse_latency_ms"] = None
    _physical_checks(result, failures)
    if counts.get("unexpected_lit", 0):
        failures.append(f"unexpected_lit={counts['unexpected_lit']}: light "
                        "outside the candidate pair; retune --prior-offset-ms")
    if fit.block_divergence_ns > MAP_DIVERGENCE_TOLERANCE_NS:
        failures.append(
            f"block map divergence {fit.block_divergence_ns / 1e6:.3f} ms "
            "exceeds 1 ms: the affine map is not stable within the session"
        )
    crossing_ns = band_center_ns = None
    try:
        crossing_ns, lo, hi = split_crossing_ns(classified)
        result["split_crossing"] = {
            "offset_ms": crossing_ns / 1e6, "ci95_ms": [lo / 1e6, hi / 1e6],
            "ci_note": ("conditional on the fitted affine map; pulses "
                        "resampled as independent"),
        }
        try:
            linear_ns = linear_split_crossing_ns(classified)
            result["linear_split_diagnostic_ms"] = linear_ns / 1e6
            if abs(linear_ns - crossing_ns) > ESTIMATOR_AGREEMENT_NS:
                failures.append(
                    "split estimators disagree "
                    f"({abs(linear_ns - crossing_ns) / 1e6:.3f} ms)"
                )
        except InsufficientData as exc:
            result["linear_split_diagnostic_ms"] = str(exc)
        phases = sorted(c.phase_ns for c in classified if c.kind == "split")
        below = sum(1 for x in phases if x < crossing_ns)
        result["split_phase_balance"] = {
            "below_crossing": below, "above_crossing": len(phases) - below,
        }
    except InsufficientData as exc:
        result["split_crossing"] = {"insufficient": str(exc)}
    try:
        band = miss_band(classified, median_width, period_ns)
        coherent = _band_is_coherent(band)
        if coherent:
            # An incoherent center is meaningless, not noisy, so it is
            # not carried forward into the cross-regime contrast either
            # (review finding: that was the last path where a rejected
            # band still derived a number of its own).
            band_center_ns = band.center_ns
        band_width_ns = (
            band.band_end_ns - band.band_start_ns
        ) % band.period_ns
        sampled = band.misses + band.lit_samples
        result["miss_band"] = {
            "band_start_ms": band.band_start_ns / 1e6,
            "band_end_ms": band.band_end_ns / 1e6,
            "center_ms": band.center_ns / 1e6,
            "exposure_width_ms": band.exposure_width_ns / 1e6,
            "misses": band.misses, "lit_samples": band.lit_samples,
            "misses_inside_band": band.misses_inside,
            "misses_outside_band": band.misses_outside,
            "lit_samples_inside_band": band.lit_inside,
            "coherence_fraction": band.misses_inside / band.misses,
            "observed_miss_fraction": band.misses / sampled,
            "uniform_expected_miss_fraction": band_width_ns / band.period_ns,
            "consistent": coherent,
        }
        # The model admits ONE dead-time run per period, so a miss
        # outside the band contradicts the population that set it. No
        # tolerance: a fraction here would be a tuned number standing in
        # for a classification-error model nobody has measured.
        if not coherent:
            failures.append(
                f"miss band incoherent: {band.misses_outside} of "
                f"{band.misses} misses lie OUTSIDE the "
                f"{band_width_ns / 1e6:.3f} ms band they set "
                f"(coherence {band.misses_inside / band.misses:.3f}, "
                f"lit samples inside {band.lit_inside}); this miss "
                "population cannot be one stationary dead-time band - "
                "suspect sub-threshold deposits from LED misalignment "
                "or a response that changed mid-session (PROTOCOL)"
            )
    except InsufficientData as exc:
        result["miss_band"] = {"insufficient": str(exc)}
    if crossing_ns is not None and band_center_ns is not None:
        gap = abs(_circular_delta(crossing_ns, band_center_ns, period_ns))
        result["cross_regime_gap_ms"] = gap / 1e6
        if gap > CROSS_REGIME_TOLERANCE_NS:
            failures.append(
                f"split and miss-band centers {gap / 1e6:.3f} ms apart"
            )
    # DIAGNOSTIC ONLY: optical-offset drift relative to PTS is invisible
    # to the affine-map divergence check (that bounds the PTS->publication
    # map, nothing optical). First-half vs second-half estimates catch a
    # gross within-session drift; None when either half is too thin.
    half = len(classified) // 2
    early_ns, _, _ = _primary(classified[:half], median_width, period_ns)
    late_ns, _, _ = _primary(classified[half:], median_width, period_ns)
    result["half_session_drift_ms"] = (
        _circular_delta(early_ns, late_ns, period_ns) / 1e6
        if early_ns is not None and late_ns is not None else None
    )
    canonical_ns, source, _ = _primary(classified, median_width, period_ns)
    if canonical_ns is not None:
        result["canonical"] = {
            "offset_wrapped_ms": canonical_ns / 1e6, "source": source,
        }
        scan = _prior_scan(frames, kept, fit, prior_offset_ns,
                           lit_threshold, median_width, period_ns)
        result["prior_scan"] = scan
        if not scan["stable"]:
            failures.append(
                "estimate not stable across the prior scan plateau"
            )
    else:
        failures.append("no primary estimate: neither split crossing nor "
                        "miss band reached its declared minimum")
    result["verdict"] = {"valid": not failures, "failures": failures}
    return result


def compare(labeled: list[tuple[str, Path]]) -> dict:
    sessions, known = [], []
    for label, path in labeled:
        summary = json.loads(path.read_text(encoding="utf-8"))
        canonical = summary.get("canonical") or {}
        entry = {
            "label": label, "input": summary["input"],
            "roi": (summary.get("meta") or {}).get("roi"),
            "alpha": summary["affine"]["alpha"],
            "offset_wrapped_ms": canonical.get("offset_wrapped_ms"),
            "source": canonical.get("source"),
            "frame_period_ms": summary.get("frame_period_ms"),
            "coarse_latency_ms": summary.get("coarse_latency_ms"),
            "prior_offset_ms": summary.get("prior_offset_ms"),
            "valid": summary.get("verdict", {}).get("valid"),
        }
        sessions.append(entry)
        # Only sessions that PASSED their own verdict participate in
        # contrasts (review finding: None must not count as valid).
        if entry["valid"] is True and entry["offset_wrapped_ms"] is not None:
            known.append(entry)
    result: dict = {
        "sessions": sessions,
        "excluded_from_contrasts": [
            s["label"] for s in sessions if s not in known
        ],
    }
    if len(known) < 2:
        result["comparison_invalid"] = (
            "fewer than two valid sessions; nothing to contrast"
        )
        return result
    deltas: list[float] = []
    if len(known) > 1:
        periods = [s["frame_period_ms"] for s in known]
        period_ms = sum(periods) / len(periods)
        if max(periods) - min(periods) > 0.005 * period_ms:
            result["period_mismatch_ms"] = max(periods) - min(periods)
            result["comparison_invalid"] = (
                "frame periods differ by more than 0.5%; wrapped offsets"
                " are not on a common circle - no contrasts computed"
            )
            return result
        if any(s["coarse_latency_ms"] is None for s in known):
            result["comparison_invalid"] = (
                "coarse_latency_ms missing from a session; the"
                " integer-period branch cannot be restored - no contrasts"
            )
            return result
        first = known[0]
        rows = []
        for s in known[1:]:
            circ = _circular_delta(first["offset_wrapped_ms"],
                                   s["offset_wrapped_ms"], period_ms)
            # The wrapped offset carries phase; the coarse latency
            # carries the integer-period branch. Together the delta is
            # absolute, so a whole-period shift between conditions
            # reads ~40 ms instead of aliasing to zero. SIGN: exposure
            # moving LATER against fixed PTS/publication makes the
            # covering frame index DROP, so publication-of-lit-frame
            # minus pulse-center SHRINKS - coarse latency moves opposite
            # to the optical phase (pinned by the end-to-end test).
            branch = round(
                (first["coarse_latency_ms"] - s["coarse_latency_ms"] - circ)
                / period_ms
            )
            rows.append({"label": s["label"],
                         "delta_ms": circ + branch * period_ms,
                         "circular_delta_ms": circ,
                         "branch_periods": branch})
        deltas = [0.0] + [r["delta_ms"] for r in rows]
        result["deltas_vs_first_ms"] = rows
        result["offset_spread_ms"] = max(deltas) - min(deltas)
    if len(known) == 3:
        # A-B-A: the middle block against the mean of its brackets, in
        # delta space so wrapping cannot fake a period jump.
        result["aba_contrast"] = {
            "design": [s["label"] for s in known],
            "effect_ms": deltas[1] - (deltas[0] + deltas[2]) / 2.0,
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+",
                        help="raw JSONL; or label=summary.json with --compare")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--lit-threshold", type=float, default=12.0)
    parser.add_argument("--prior-offset-ms", type=float,
                        default=DEFAULT_PRIOR_OFFSET_NS / 1e6)
    parser.add_argument("--compare", action="store_true")
    args = parser.parse_args()
    if args.compare:
        labeled = []
        for item in args.logs:
            label, _, raw = item.rpartition("=")
            labeled.append((label or Path(raw).stem, Path(raw)))
        result = compare(labeled)
        ok = (not result["excluded_from_contrasts"]
              and "comparison_invalid" not in result)
    else:
        if len(args.logs) != 1:
            parser.error("one raw JSONL per run; use --compare for many")
        result = analyze(Path(args.logs[0]), args.lit_threshold,
                         args.prior_offset_ms * 1e6)
        ok = result["verdict"]["valid"]
    args.report.write_text(json.dumps(result, indent=2) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
