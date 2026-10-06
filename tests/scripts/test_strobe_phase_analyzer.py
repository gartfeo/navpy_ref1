"""The analyzer's own obligations: fail closed, honor the prior, and
survive real-stream defects the estimator refuses to guess about.

Split from `test_strobe_phase_estimator` because these tests exercise
the IO/verdict layer, not the estimation math.
"""
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "docs" / "measurements" / "zr10-strobe-phase"
    ),
)

import analyze_strobe_phase as analyzer  # noqa: E402
from strobe_phase_estimator import (  # noqa: E402
    affine_fit,
    classify_pulses,
    miss_band,
)
from tests.scripts.test_strobe_phase_estimator import (  # noqa: E402
    BAND_GEOM,
    LINEAR,
    PRIOR_NS,
    PULSE_NS,
    SPLIT_GEOM,
    classify_on_grid,
    synth_session,
)


def write_jsonl(path: Path, frames, pulses, drop_off_for=(),
                roi=(10, 10, 40, 30), saturate=()):
    rows = [{"event": "run_start", "mono_ns": 0, "argv": ["synthetic"],
             "roi": list(roi), "versions": {}, "random_seed": 7}]
    for index, frame in enumerate(frames):
        rows.append({"event": "frame", "frame_index": index,
                     "publication_mono_ns": frame.publication_mono_ns,
                     "pts_ns": frame.pts_ns, "roi_score": frame.roi_score,
                     "roi_red_p95": 254.0 if index in saturate else 120.0,
                     "led_on": False, "source": None})
    for pulse in pulses:
        rows.append({"event": "gpio", "event_id": pulse.event_id, "value": 1,
                     "pre_write_mono_ns": pulse.lit_from_ns - 40_000,
                     "post_write_mono_ns": pulse.lit_from_ns})
        if pulse.event_id not in drop_off_for:
            rows.append({"event": "gpio", "event_id": pulse.event_id,
                         "value": 0,
                         "pre_write_mono_ns": pulse.lit_until_ns,
                         "post_write_mono_ns": pulse.lit_until_ns + 40_000})
    # Interleaving does not matter to the loader; keep insertion order.
    path.write_text(
        "\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8"
    )


def test_a_valid_synthetic_session_passes_the_verdict(tmp_path):
    frames, pulses = synth_session(*SPLIT_GEOM, 800, LINEAR)
    log = tmp_path / "session.jsonl"
    write_jsonl(log, frames, pulses)
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert result["verdict"]["valid"], result["verdict"]["failures"]
    assert "offset_ms" in result["split_crossing"]
    assert result["canonical"]["source"] == "split"
    assert 0 <= result["canonical"]["offset_wrapped_ms"] < result[
        "frame_period_ms"
    ]
    assert result["prior_scan"]["stable"]
    assert result["coarse_latency_ms"] is not None
    assert "half_session_drift_ms" in result
    assert result["meta"]["roi"] == [10, 10, 40, 30]
    assert result["diagnostics"]["gpio_edge_span_us"]["median"] == 40.0


def test_a_session_with_no_estimate_fails_closed(tmp_path):
    """PROTOCOL promises the analyzer refuses; the exit path must too."""
    frames, pulses = synth_session(*SPLIT_GEOM, 12, LINEAR)  # too few
    log = tmp_path / "thin.jsonl"
    write_jsonl(log, frames, pulses)
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert not result["verdict"]["valid"]
    assert any("no primary estimate" in f for f in result["verdict"]["failures"])


def test_a_wrong_prior_fails_the_verdict_instead_of_shifting_the_answer(
    tmp_path,
):
    frames, pulses = synth_session(*SPLIT_GEOM, 400, LINEAR)
    log = tmp_path / "wrongprior.jsonl"
    write_jsonl(log, frames, pulses)
    result = analyzer.analyze(log, 12.0, PRIOR_NS - 60_000_000)
    assert not result["verdict"]["valid"]
    assert result["classes"].get("unexpected_lit", 0) > 0


def test_final_approach_exclusion_honors_a_nondefault_prior():
    """The cutoff must move with the prior the operator chose (review
    finding: it silently used the default)."""
    frames, pulses = synth_session(*SPLIT_GEOM, 100, LINEAR)
    fit = affine_fit(frames)
    strict = analyzer.drop_final_approach_pulses(frames, pulses, fit, 300_000_000)
    lax = analyzer.drop_final_approach_pulses(frames, pulses, fit, 0)
    assert len(strict) <= len(lax)
    assert len(lax) == len(pulses), "zero prior excludes nothing here"


def test_malformed_gpio_pairs_are_counted_and_skipped(tmp_path):
    frames, pulses = synth_session(*SPLIT_GEOM, 60, LINEAR)
    log = tmp_path / "malformed.jsonl"
    write_jsonl(log, frames, pulses, drop_off_for={3, 7})
    loaded_frames, loaded_pulses, extra = analyzer.load_session(log)
    assert len(loaded_pulses) == len(pulses) - 2
    assert extra["diagnostics"]["gpio_pairs_malformed"] == 2


def test_duplicate_pts_frames_are_rejected_not_estimated(tmp_path):
    frames, pulses = synth_session(*SPLIT_GEOM, 60, LINEAR)
    frames = list(frames)
    frames.insert(50, replace(frames[50]))  # exact duplicate PTS
    log = tmp_path / "dup.jsonl"
    write_jsonl(log, frames, pulses)
    loaded_frames, _, extra = analyzer.load_session(log)
    assert extra["diagnostics"]["frames_rejected_bad_pts"] == 1
    assert len(loaded_frames) == len(frames) - 1


def test_a_dropped_frame_classifies_cadence_gap_not_miss():
    """A hole in the stream is one long local period; pooling its
    wrapped phase with the others would mix circular coordinates."""
    frames, pulses = synth_session(*SPLIT_GEOM, 200, LINEAR)
    fit = affine_fit(frames)
    reference = classify_pulses(frames, pulses, fit, PRIOR_NS, 12.0)
    target = next(c for c in reference if c.kind == "split")
    victim = frames.index(target.later)
    gapped = frames[:victim] + frames[victim + 1:]
    classified = classify_pulses(gapped, pulses, affine_fit(gapped),
                                 PRIOR_NS, 12.0)
    by_id = {c.pulse.event_id: c for c in classified}
    assert by_id[target.pulse.event_id].kind == "cadence_gap"


def _summary(tmp_path, name, offset, valid=True, coarse=100.0,
             period=40.09):
    path = tmp_path / f"{name}.json"
    path.write_text(json.dumps({
        "input": name, "meta": {"roi": [0, 0, 1, 1]},
        "affine": {"alpha": 1.2},
        "frame_period_ms": period,
        "coarse_latency_ms": coarse,
        "prior_offset_ms": 100.0,
        "canonical": {"offset_wrapped_ms": offset, "source": "split"},
        "verdict": {"valid": valid},
    }), encoding="utf-8")
    return path


def test_compare_reports_circular_deltas_and_the_aba_contrast(tmp_path):
    result = analyzer.compare([
        ("top", _summary(tmp_path, "a", 5.0)),
        ("bottom", _summary(tmp_path, "b", 7.0)),
        ("top2", _summary(tmp_path, "c", 5.4)),
    ])
    assert result["excluded_from_contrasts"] == []
    assert round(result["offset_spread_ms"], 9) == 2.0
    deltas = [d["delta_ms"] for d in result["deltas_vs_first_ms"]]
    assert [round(x, 9) for x in deltas] == [2.0, 0.4]
    # B against the mean of its brackets, so session drift cancels.
    assert abs(result["aba_contrast"]["effect_ms"] - 1.8) < 1e-9


def test_compare_wraps_a_delta_across_the_period_seam(tmp_path):
    """39.8 vs 0.3 ms is a 0.59 ms step across the seam, not a 39.5 ms
    jump - the exact defect a linear subtraction of wrapped values had."""
    result = analyzer.compare([
        ("a", _summary(tmp_path, "a", 39.8)),
        ("b", _summary(tmp_path, "b", 0.3)),
    ])
    row = result["deltas_vs_first_ms"][0]
    assert row["branch_periods"] == 0
    assert abs(row["delta_ms"] - 0.59) < 1e-9


def test_compare_restores_the_integer_period_branch_end_to_end(tmp_path):
    # Two RAW sessions from the real generator (review finding: a
    # hand-moved coarse latency validated the wrong sign). Exposure
    # start shifted by +10 ms and by +10 ms + one period; the absolute
    # delta must read ~+10 and ~+50.09, not alias or flip sign.
    period_ms = 40.09
    labeled = []
    # A full-period shift under the FIXED prior fails loudly as
    # unexpected_lit (verified) - the alias risk is the operator then
    # RETUNING the prior by one period, which the wrapped offsets can
    # no longer see. That retuned session is the one compared here.
    for name, shift_ns, prior_ns in (
        ("base", 0, PRIOR_NS),
        ("late", 10_000_000, PRIOR_NS),
        ("wrapped", 10_000_000 + 40_090_000, PRIOR_NS - 40_090_000),
    ):
        frames, pulses = synth_session(
            SPLIT_GEOM[0] + shift_ns, SPLIT_GEOM[1], 800, LINEAR
        )
        log = tmp_path / f"{name}.jsonl"
        write_jsonl(log, frames, pulses)
        summary = analyzer.analyze(log, 12.0, prior_ns)
        assert summary["verdict"]["valid"], (name,
                                             summary["verdict"]["failures"])
        out = tmp_path / f"{name}.json"
        out.write_text(json.dumps(summary), encoding="utf-8")
        labeled.append((name, out))
    result = analyzer.compare(labeled)
    late, wrapped = result["deltas_vs_first_ms"]
    assert abs(late["delta_ms"] - 10.0) < 1.5, late
    assert late["branch_periods"] == 0
    assert abs(wrapped["delta_ms"] - (10.0 + period_ms)) < 1.5, wrapped
    assert wrapped["branch_periods"] == 1


def test_compare_refuses_a_single_valid_session(tmp_path):
    result = analyzer.compare([
        ("only", _summary(tmp_path, "a", 5.0)),
        ("bad", _summary(tmp_path, "b", 7.0, valid=False)),
    ])
    assert "comparison_invalid" in result
    assert "deltas_vs_first_ms" not in result


def test_compare_period_mismatch_suppresses_contrasts(tmp_path):
    result = analyzer.compare([
        ("a", _summary(tmp_path, "a", 5.0)),
        ("b", _summary(tmp_path, "b", 7.0, period=41.0)),
    ])
    assert "comparison_invalid" in result
    assert "deltas_vs_first_ms" not in result
    assert "aba_contrast" not in result


def test_a_publication_step_fails_the_map_divergence_gate(tmp_path):
    # +5 ms publication step halfway: PTS->publication map unstable.
    frames, pulses = synth_session(*SPLIT_GEOM, 800, LINEAR)
    frames = list(frames)
    half = len(frames) // 2
    frames[half:] = [
        replace(f, publication_mono_ns=f.publication_mono_ns + 5_000_000)
        for f in frames[half:]
    ]
    log = tmp_path / "step.jsonl"
    write_jsonl(log, frames, pulses)
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert not result["verdict"]["valid"]
    assert any("affine map is not stable" in f
               for f in result["verdict"]["failures"])


def test_compare_excludes_invalid_sessions_from_contrasts(tmp_path):
    """valid=None and valid=False must NOT count as valid."""
    result = analyzer.compare([
        ("good", _summary(tmp_path, "a", 5.0)),
        ("bad", _summary(tmp_path, "b", 7.0, valid=False)),
        ("good2", _summary(tmp_path, "c", 5.4)),
    ])
    assert result["excluded_from_contrasts"] == ["bad"]
    assert "aba_contrast" not in result, "an invalid middle voids A-B-A"


def test_a_saturated_session_fails_the_verdict(tmp_path):
    frames, pulses = synth_session(*SPLIT_GEOM, 800, LINEAR)
    log = tmp_path / "sat.jsonl"
    write_jsonl(log, frames, pulses, saturate={60, 61})
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert not result["verdict"]["valid"]
    assert any("saturated" in f for f in result["verdict"]["failures"])


def test_a_too_tall_roi_fails_the_verdict(tmp_path):
    frames, pulses = synth_session(*SPLIT_GEOM, 800, LINEAR)
    log = tmp_path / "tall.jsonl"
    write_jsonl(log, frames, pulses, roi=(10, 10, 40, 130))
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert not result["verdict"]["valid"]
    assert any("ROI height" in f for f in result["verdict"]["failures"])


def test_a_pure_miss_band_session_gets_a_prior_gated_verdict(tmp_path):
    """The scan must gate the ACTIVE estimator (review finding: a
    band-only session skipped the stability rule entirely)."""
    from tests.scripts.test_strobe_phase_estimator import BAND_GEOM

    frames, pulses = synth_session(*BAND_GEOM, 400, LINEAR)
    log = tmp_path / "band.jsonl"
    write_jsonl(log, frames, pulses)
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert result["canonical"]["source"] == "miss_band"
    assert result["prior_scan"]["admissible_estimates"] > 1
    assert result["verdict"]["valid"], result["verdict"]["failures"]
    band = result["miss_band"]
    assert band["consistent"] is True
    assert band["misses_outside_band"] == 0
    assert band["coherence_fraction"] == 1.0
    # Audit identities: the reported fractions must match their counts.
    assert band["misses_inside_band"] + band["misses_outside_band"] == (
        band["misses"]
    )
    sampled = band["misses"] + band["lit_samples"]
    assert abs(
        band["observed_miss_fraction"] - band["misses"] / sampled
    ) < 1e-12


def test_an_incoherent_miss_band_fails_the_verdict(tmp_path):
    """The 2026-09-01 bench defect: misses scattered across the whole
    phase circle still yield a narrow, confident-looking band. A green
    verdict on that data is what this gate exists to stop."""
    frames, pulses = synth_session(*BAND_GEOM, 400, LINEAR)
    log = tmp_path / "incoherent.jsonl"
    # Blind a scattered sample of frames: their pulses become misses at
    # phases having nothing to do with the dead band.
    write_jsonl(log, frames, pulses)
    rows = [json.loads(line) for line in
            log.read_text(encoding="utf-8").splitlines()]
    blinded = 0
    for row in rows:
        if row["event"] == "frame" and row["frame_index"] % 3 == 0:
            if row["roi_score"] > 20.0:
                row["roi_score"] = 3.0
                blinded += 1
    log.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    assert blinded > 20, "fixture must actually blind lit frames"
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    assert not result["verdict"]["valid"]
    assert any("miss band incoherent" in f
               for f in result["verdict"]["failures"]), result["verdict"]
    assert result["miss_band"]["consistent"] is False
    assert result["miss_band"]["misses_outside_band"] > 0
    # An incoherent band must not be PROMOTED to an estimate: no
    # canonical, no prior-scan plateau built from it. Its own band and
    # center stay in the report as forensics, which is why this is not
    # "publishes no number" (review finding).
    assert "canonical" not in result
    assert result["half_session_drift_ms"] is None
    assert any("no primary estimate" in f
               for f in result["verdict"]["failures"])


def test_a_split_session_with_scattered_misses_still_fails(tmp_path):
    """Winning estimator precedence does not certify the population: a
    split crossing drawn from data that also carries an incoherent miss
    band comes from a session whose response was not stationary."""
    frames, pulses = synth_session(*SPLIT_GEOM, 900, LINEAR)
    log = tmp_path / "mixed.jsonl"
    write_jsonl(log, frames, pulses)
    rows = [json.loads(line) for line in
            log.read_text(encoding="utf-8").splitlines()]
    # Blind ADJACENT frames: a miss needs both frames of a pair dark,
    # while isolated blinding would only destroy splits.
    blinded = 0
    for row in rows:
        if row["event"] == "frame" and row["frame_index"] % 41 in (0, 1):
            if row["roi_score"] > 20.0:
                row["roi_score"] = 3.0
                blinded += 1
    log.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    result = analyzer.analyze(log, 12.0, PRIOR_NS)
    # The scenario must actually materialize, or the test proves nothing.
    assert blinded > 20
    assert "offset_ms" in result["split_crossing"], result["split_crossing"]
    assert "insufficient" not in result["miss_band"], result["miss_band"]
    assert result["canonical"]["source"] == "split"
    assert result["miss_band"]["misses_outside_band"] > 0
    assert not result["verdict"]["valid"]
    assert any("miss band incoherent" in f
               for f in result["verdict"]["failures"]), result["verdict"]
    # This is the ONLY shape that reaches the cross-regime contrast with
    # a band in hand, and an incoherent center is meaningless rather
    # than noisy: contrasting it would publish a derived number from a
    # rejected band (review finding).
    assert "cross_regime_gap_ms" not in result


def test_a_lit_sample_tied_to_a_miss_phase_fails_the_gate():
    """The other half of the rule, which the miss count cannot see.

    An exact tie between a lit sample and a miss phase is measure-zero
    in recorded data, so it is constructed at the estimator and judged
    here: the band below has NO miss outside it and would pass on that
    count alone.
    """
    frames, pulses = synth_session(*BAND_GEOM, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    clean = miss_band(classified, PULSE_NS, period)
    assert analyzer._band_is_coherent(clean)
    edge_miss = max(
        (c for c in classified if c.kind == "miss"),
        key=lambda c: (c.phase_ns % period - clean.band_start_ns) % period,
    )
    victim = next(c for c in classified if c.kind == "single")
    tied = [
        replace(c, phase_ns=edge_miss.phase_ns % period) if c is victim else c
        for c in classified
    ]
    band = miss_band(tied, PULSE_NS, period)
    assert band.misses_outside == 0, "the miss count alone would pass this"
    assert band.lit_inside == 1
    assert not analyzer._band_is_coherent(band)


def test_the_recorded_bench_defect_session_is_now_rejected():
    """Regression fixture: the real session that returned VALID before
    this gate existed (ledger 66). 305 of its 317 misses contradict the
    2.258 ms band they set."""
    session = (
        Path(__file__).resolve().parents[2]
        / "docs" / "measurements" / "zr10-strobe-phase" / "sessions"
        / "s1_dim_restart.jsonl"
    )
    assert session.exists(), f"tracked bench fixture missing: {session}"
    result = analyzer.analyze(session, 12.0, 99.8 * 1_000_000)
    band = result["miss_band"]
    assert band["misses"] == 317
    assert band["misses_inside_band"] + band["misses_outside_band"] == 317
    assert band["misses_outside_band"] > 300
    assert band["consistent"] is False
    assert not result["verdict"]["valid"]
    assert any("miss band incoherent" in f
               for f in result["verdict"]["failures"]), result["verdict"]
    assert "canonical" not in result, "a rejected session published a number"
