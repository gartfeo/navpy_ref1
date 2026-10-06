"""The strobe-phase estimator against synthetic truth.

Ground truth: a rectangular integration window of width E starting at
offset A after each mapped PTS tick. Pulses deposit light proportional
to overlap; the sensor response wraps the deposit. Both estimators
target the same invariant M = A + (E + P) / 2 - the midpoint between
one frame's exposure end and the next frame's exposure start - so the
tests pin that number, not internal geometry.

The tests that matter most:
- the sign crossing recovers M under a LINEAR response AND under a
  GAMMA-warped one (that invariance is why the estimator exists);
- the linear-split diagnostic AGREES with the crossing under any
  shared monotone response (so on real data a gap between them flags
  a response differing between frames, not nonlinearity);
- the miss-band CENTER survives a detection threshold that displaces
  both band edges (the regime the real camera lands in if AE picks a
  short shutter), and equals the same M;
- too little data raises instead of reporting.
"""
from __future__ import annotations

import random
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "docs" / "measurements" / "zr10-strobe-phase"
    ),
)

from strobe_phase_estimator import (  # noqa: E402
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

PERIOD_NS = 40_090_000
ALPHA = 1.2026908            # measured slope, 08-30 dataset
PIPE_DELAY_NS = 100_000_000  # exposure -> publication, absorbed by beta
PULSE_NS = 5_000_000
# The mapped-PTS axis is publication-anchored (beta absorbs the
# pipeline delay), so the RECEIVING frame's mapped PTS sits about
# PIPE_DELAY - offset - [0..E] AFTER the pulse. The prior must place
# the predicted landing MID-window: at the window edge the pair flips
# one frame late exactly around the handoff, silently discarding the
# split population (observed: 48 -> 18 splits at prior 94.5 ms). A
# wrong prior shows up as lost splits and "unexpected_lit", never as a
# shifted answer.
PRIOR_NS = 75_000_000


def handoff_midpoint(offset_ns: int, width_ns: int) -> float:
    """M: where exposure end and next exposure start meet, phase terms.

    The estimator's phase axis is publication-anchored (beta absorbs
    the pipeline delay), so M carries -PIPE_DELAY: exposure opens
    PIPE_DELAY before the frame's mapped PTS, plus the true offset.
    """
    return offset_ns - PIPE_DELAY_NS + (width_ns + PERIOD_NS) / 2.0


def synth_session(
    exposure_offset_ns: int,
    exposure_width_ns: int,
    events: int,
    response,
    seed: int = 7,
    baseline: float = 3.0,
    gain: float = 60.0,
    noise_ns: int = 300_000,
):
    """Frames on a PTS grid + pulses at uniform phase, honest response."""
    rng = random.Random(seed)
    pts_step = int(PERIOD_NS / ALPHA)
    count = events * 20 + 120
    frames = []
    for k in range(count):
        pts = 1_000_000 + k * pts_step
        frames.append(
            Frame(
                pts_ns=pts,
                publication_mono_ns=int(
                    ALPHA * pts + PIPE_DELAY_NS + rng.gauss(0.0, noise_ns)
                ),
                roi_score=baseline,
            )
        )
    scores = dict.fromkeys(range(count), baseline)
    pulses = []
    cursor = ALPHA * frames[40].pts_ns
    for event_id in range(events):
        cursor += rng.uniform(0.31e9, 0.73e9)
        start = int(cursor)
        pulses.append(Pulse(event_id, start, start + PULSE_NS))
        near = int((start - ALPHA * frames[0].pts_ns) / PERIOD_NS)
        for k in range(max(0, near - 2), min(count, near + 3)):
            open_ns = ALPHA * frames[k].pts_ns + exposure_offset_ns
            close_ns = open_ns + exposure_width_ns
            overlap = max(
                0.0, min(close_ns, start + PULSE_NS) - max(open_ns, start)
            )
            if overlap > 0.0:
                scores[k] = baseline + gain * response(overlap / PULSE_NS)
    frames = [
        Frame(f.pts_ns, f.publication_mono_ns, scores[i])
        for i, f in enumerate(frames)
    ]
    return frames, pulses


def run_classify(frames, pulses, lit_threshold=12.0):
    fit = affine_fit(frames)
    return classify_pulses(frames, pulses, fit, PRIOR_NS, lit_threshold), fit


def classify_on_grid(frames, pulses, lit_threshold=12.0):
    """Classified pulses plus the frame-grid clock they are judged on -
    the pair `miss_band` takes, so no test lets the classifications pick
    their own circle."""
    classified, fit = run_classify(frames, pulses, lit_threshold)
    return classified, median_period_ns(frames, fit)


LINEAR = lambda deposit: deposit         # noqa: E731
GAMMA = lambda deposit: deposit ** 0.45  # noqa: E731

SPLIT_GEOM = (6_000_000, 39_500_000)   # E + w > P: splits exist
BAND_GEOM = (6_000_000, 20_000_000)    # E + w < P: miss band instead


def test_split_crossing_recovers_the_handoff_under_linear_response():
    offset, width = SPLIT_GEOM
    frames, pulses = synth_session(offset, width, 800, LINEAR)
    classified, _ = run_classify(frames, pulses)
    crossing, lo, hi = split_crossing_ns(classified)
    expected = handoff_midpoint(offset, width)
    assert abs(crossing - expected) < 1_000_000, (crossing, expected)
    assert lo <= crossing <= hi


def test_split_crossing_survives_a_gamma_warped_response():
    """THE reason this estimator exists."""
    offset, width = SPLIT_GEOM
    frames, pulses = synth_session(offset, width, 800, GAMMA)
    classified, _ = run_classify(frames, pulses)
    crossing, _, _ = split_crossing_ns(classified)
    assert abs(crossing - handoff_midpoint(offset, width)) < 1_000_000


def test_the_two_split_estimators_agree_under_a_shared_response():
    """Consistency, not bias - a claim this test CORRECTED.

    With a symmetric pulse and one response shared by both frames, the
    split fraction is antisymmetric about the crossing under ANY
    monotone warp, so the linear fit is unbiased there too - the
    original "linear is biased under gamma" test failed by agreeing to
    3 us. The estimators must therefore AGREE on clean data; their
    disagreement on real data flags a response that differs BETWEEN
    the two frames (AGC step, clipping) - a data-quality alarm, not a
    choice between numbers.
    """
    offset, width = SPLIT_GEOM
    for response in (LINEAR, GAMMA):
        frames, pulses = synth_session(offset, width, 800, response)
        classified, _ = run_classify(frames, pulses)
        sign_based, _, _ = split_crossing_ns(classified)
        linear_based = linear_split_crossing_ns(classified)
        assert abs(linear_based - sign_based) < 500_000, (
            linear_based,
            sign_based,
        )


def test_miss_band_center_is_threshold_invariant_and_is_the_same_m():
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    expected = handoff_midpoint(offset, width) % PERIOD_NS
    centers = []
    for threshold in (8.0, 25.0):
        classified, period = classify_on_grid(
            frames, pulses, lit_threshold=threshold
        )
        band = miss_band(classified, PULSE_NS, period)
        centers.append(band.center_ns)
        assert abs(band.center_ns - expected) < 1_200_000, (
            band.center_ns,
            expected,
        )
    assert abs(centers[0] - centers[1]) < 1_000_000, centers


def test_miss_band_recovers_the_exposure_width():
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    band = miss_band(classified, PULSE_NS, period)
    # Documented bias: low by up to 2 * detection-threshold overlap.
    assert width - 5_000_000 < band.exposure_width_ns < width + 2_500_000


def test_a_clean_band_contains_every_miss_it_was_fitted_from():
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    band = miss_band(classified, PULSE_NS, period)
    assert band.misses_outside == 0
    assert band.misses_inside == band.misses
    assert band.misses_inside + band.misses_outside == band.misses
    # lit_samples counts single AND split, which the old name denied.
    lit = sum(1 for c in classified if c.kind in ("single", "split"))
    assert band.lit_samples == lit


def test_one_stray_miss_outside_the_band_is_counted():
    """Pins the zero rule: a single contradicting miss must show up, so
    the gate cannot be satisfied by a mostly-coherent population."""
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    clean = miss_band(classified, PULSE_NS, period)
    # REPLACE one lit sample (do not append a duplicate: the population
    # must stay a valid session), half a period away from the band.
    stray_phase = (clean.center_ns + period / 2) % period
    victim = next(c for c in classified if c.kind == "single")
    planted = [
        replace(c, kind="miss", phase_ns=stray_phase) if c is victim else c
        for c in classified
    ]
    band = miss_band(planted, PULSE_NS, period)
    assert band.misses_outside == 1
    assert band.misses == clean.misses + 1
    assert band.lit_samples == clean.lit_samples - 1


def test_scattered_misses_cannot_pose_as_a_narrow_band():
    """The real defect: sub-threshold deposits scatter misses over the
    whole circle, the widest accidental cluster still yields a band, and
    only the coherence count exposes it."""
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    rng = random.Random(11)
    scattered = [
        replace(c, kind="miss", phase_ns=rng.uniform(0.0, period))
        if c.kind in ("single", "split") and rng.random() < 0.7 else c
        for c in classified
    ]
    band = miss_band(scattered, PULSE_NS, period)
    # A band still comes back - that is the trap. Only the count of
    # contradicting misses reveals it. (The real fixture in the analyzer
    # tests carries the severity: 305 of 317 outside.)
    assert band.misses_outside > 0
    assert band.misses_inside < band.misses


def test_a_band_wrapping_phase_zero_still_contains_its_misses():
    # Exposure late in the period puts the dead zone across the seam.
    frames, pulses = synth_session(34_000_000, 20_000_000, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    band = miss_band(classified, PULSE_NS, period)
    assert band.band_start_ns > band.band_end_ns, "band must cross zero"
    assert band.misses_outside == 0


def test_the_supplied_circle_is_the_only_one_used():
    """Review finding, and the reason the period is a PARAMETER.

    The superseded code wrapped each sample by its OWN local period and
    fitted on the median of those, so local-period spread displaced
    phases against each other and one artifact would fail an honest
    session under a zero-tolerance rule. Reproducing that algorithm
    (seed 8, +-50 us) puts two misses outside a band that has none, and
    its period misses the grid by hundreds of ns at every seed tried -
    so both assertions below discriminate against it.
    """
    frames, pulses = synth_session(34_000_000, 20_000_000, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    clean = miss_band(classified, PULSE_NS, period)
    assert clean.band_start_ns > clean.band_end_ns, "band must cross zero"
    rng = random.Random(8)
    jittered = [
        replace(c, local_period_ns=c.local_period_ns + rng.uniform(-5e4, 5e4))
        for c in classified
    ]
    band = miss_band(jittered, PULSE_NS, period)
    assert band.period_ns == period
    assert band.misses_outside == 0
    assert (band.band_start_ns, band.band_end_ns) == (
        clean.band_start_ns, clean.band_end_ns
    )


def test_only_phase_bearing_kinds_reach_the_circle():
    """A cadence_gap spans a dropped frame and an unexpected_lit sits
    outside its candidate pair, so neither has a phase measured against
    this circle. Planting both INSIDE the band must change nothing:
    counting them would invent dead time, or would put a lit sample in
    the band and fail an honest session."""
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    clean = miss_band(classified, PULSE_NS, period)
    intruders = [
        replace(classified[i], kind=kind, phase_ns=clean.center_ns,
                local_period_ns=classified[i].local_period_ns * 2)
        for i, kind in enumerate(("cadence_gap", "unexpected_lit") * 20)
    ]
    band = miss_band(classified + intruders, PULSE_NS, period)
    assert band.period_ns == clean.period_ns
    assert (band.misses, band.lit_samples) == (clean.misses, clean.lit_samples)
    assert band.misses_outside == 0
    assert band.lit_inside == 0


def test_a_lit_sample_sharing_a_miss_phase_is_not_swallowed():
    """A miss and a lit at the SAME phase contradict each other. The
    band edge is a midpoint, so an exact tie lands the lit sample on the
    boundary - it must be counted, not silently absorbed."""
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    clean = miss_band(classified, PULSE_NS, period)
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
    assert band.lit_inside == 1
    assert clean.lit_inside == 0


def test_nonuniform_phase_sampling_stays_coherent():
    """Coherence is topological, so a lopsided phase distribution must
    not trip it - that is why the gate is not a uniform-phase fraction."""
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 400, LINEAR)
    classified, period = classify_on_grid(frames, pulses)
    lopsided = [c for c in classified
                if c.kind != "miss" or c.phase_ns % period < period * 0.75]
    band = miss_band(lopsided, PULSE_NS, period)
    assert band.misses_outside == 0


@pytest.mark.parametrize(
    "period", [0.0, -40_000_000.0, float("nan"), float("inf")]
)
def test_a_malformed_circle_fails_loud_not_as_insufficient_data(period):
    """The analyzer catches InsufficientData around `miss_band` and
    files it as "insufficient" in the report, so a bad circle must not
    raise that type - it would hide a caller bug as thin data. NaN and
    +inf clear a bare `<= 0` test, and a NaN circle would otherwise read
    as a maximally incoherent band rather than a malformed call."""
    offset, width = BAND_GEOM
    frames, pulses = synth_session(offset, width, 150, LINEAR)
    classified, _ = run_classify(frames, pulses)
    with pytest.raises(ValueError) as excinfo:
        miss_band(classified, PULSE_NS, period)
    assert not isinstance(excinfo.value, InsufficientData)


def test_too_few_splits_raises_instead_of_reporting():
    offset, width = BAND_GEOM  # no splits possible in this regime
    frames, pulses = synth_session(offset, width, 150, LINEAR)
    classified, _ = run_classify(frames, pulses)
    with pytest.raises(InsufficientData):
        split_crossing_ns(classified)


def test_block_divergence_flags_an_intercept_step():
    # A +5 ms publication step is invisible to a slope-only ppm figure
    # (review finding) but must show as time-domain block divergence.
    from dataclasses import replace

    frames, _ = synth_session(*SPLIT_GEOM, 200, LINEAR)
    clean = affine_fit(frames)
    assert clean.block_divergence_ns < 500_000
    frames = list(frames)
    half = len(frames) // 2
    frames[half:] = [
        replace(f, publication_mono_ns=f.publication_mono_ns + 5_000_000)
        for f in frames[half:]
    ]
    stepped = affine_fit(frames)
    assert stepped.block_divergence_ns > 2_000_000


def test_affine_fit_recovers_the_planted_slope():
    frames, _ = synth_session(*SPLIT_GEOM, 60, LINEAR)
    fit = affine_fit(frames)
    assert abs(fit.alpha - ALPHA) < 1e-4
    assert fit.block_alphas, "blockwise diagnostics must be populated"


def test_one_sided_split_signs_raise_instead_of_extrapolating():
    """A crossing outside the sampled phase range is not estimable."""
    offset, width = SPLIT_GEOM
    frames, pulses = synth_session(offset, width, 800, LINEAR)
    classified, _ = run_classify(frames, pulses)
    crossing, _, _ = split_crossing_ns(classified)
    one_sided = [
        c if c.kind != "split" or c.phase_ns >= crossing
        else replace(c, kind="single")
        for c in classified
    ]
    with pytest.raises(InsufficientData):
        split_crossing_ns(one_sided)


def test_asymmetric_phase_sampling_bends_the_linear_root_not_the_crossing():
    """Why the linear fit stays a diagnostic even under a shared
    response: truncated one-sided coverage bends its root by hundreds
    of microseconds (reviewer reproduced 0.4-0.7 ms), while the sign
    crossing holds."""
    offset, width = SPLIT_GEOM
    frames, pulses = synth_session(offset, width, 1600, GAMMA)
    classified, _ = run_classify(frames, pulses)
    crossing, _, _ = split_crossing_ns(classified)
    truncated = [
        c for c in classified
        if c.kind != "split" or c.phase_ns > crossing - 400_000
    ]
    truncated_crossing, _, _ = split_crossing_ns(truncated)
    linear_root = linear_split_crossing_ns(truncated)
    assert abs(truncated_crossing - crossing) < 700_000
    assert abs(linear_root - crossing) > abs(truncated_crossing - crossing), (
        linear_root, truncated_crossing, crossing,
    )
