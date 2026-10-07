#!/usr/bin/env python3
"""Pure estimation for the strobe phase bench. No IO, no hardware.

The question: where, relative to affine-mapped PTS, does the camera
actually respond to light? A short strobe at known monotonic time
deposits light into whichever frame's integration window overlaps it.
Sweeping strobe phase across the frame period and reading the
continuous `roi_score` locates the window WITHOUT a photodiode.

What this measures is the EFFECTIVE OPTICAL-RESPONSE OFFSET relative
to the fitted PTS->publication affine map (review finding): GPIO
timestamps bound the electrical command, not the emitted light, and
`beta` is publication-anchored, so the offset is defined against that
map, not as an intrinsic camera constant. The LED's own
electrical-to-optical delay must be bounded separately (PROTOCOL.md).

Two regimes, decided by the data, not by us:

Both regimes, with phase measured at the PULSE CENTER, estimate the
same invariant M: the midpoint between one frame's exposure END and
the next frame's exposure START. That identity is the cross-regime
consistency check.

- SPLIT (exposure width E + pulse width w > period P): some pulses
  straddle the handoff between two adjacent integrations and light
  both frames. Equal overlap deposits equal light, so the phase where
  the two frames score EQUALLY is invariant under any single monotone
  sensor response - and by symmetry that phase is M, independent of
  the pulse width. Estimated distribution-free as the 50% point of
  "later frame scored higher" vs phase.
- MISS-BAND (E + w < P): no pulse can light two frames; instead a
  band of center-phases lights none. The detection threshold WIDENS
  the observed band equally on both sides, so the band CENTER is
  threshold-invariant and again equals M, while the width (and so the
  derived E = P - w - band_width) is biased by up to 2*delta.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Frame:
    pts_ns: int
    publication_mono_ns: int
    roi_score: float


@dataclass(frozen=True, slots=True)
class Pulse:
    """One strobe. Light is guaranteed on within [lit_from, lit_until]."""

    event_id: int
    lit_from_ns: int    # post-write stamp of the ON edge
    lit_until_ns: int   # pre-write stamp of the OFF edge

    @property
    def center_ns(self) -> int:
        return (self.lit_from_ns + self.lit_until_ns) // 2

    @property
    def width_ns(self) -> int:
        return self.lit_until_ns - self.lit_from_ns


@dataclass(frozen=True, slots=True)
class AffineFit:
    """publication ~= alpha * pts + beta, with honesty diagnostics."""

    alpha: float
    beta: float
    residual_sd_ns: float
    n: int
    block_alphas: tuple[float, ...]
    # Worst disagreement, in ns, between any per-block fit and the
    # global fit, evaluated at that block's endpoints. A slope-only
    # ppm figure hides intercept steps and understates the PHASE cost
    # of a slope error over a 20 s block (review finding); this is the
    # number the verdict can bound in milliseconds.
    block_divergence_ns: float

    def map_pts(self, pts_ns: int) -> float:
        return self.alpha * pts_ns + self.beta


@dataclass(frozen=True, slots=True)
class Classified:
    """One pulse against its timestamp-chosen candidate frame pair."""

    pulse: Pulse
    earlier: Frame
    later: Frame
    deposit_earlier: float
    deposit_later: float
    # "split" | "single" | "miss" | "unexpected_lit" | "cadence_gap"
    kind: str
    phase_ns: float      # pulse center minus mapped pts of `earlier`
    local_period_ns: float


class InsufficientData(ValueError):
    """Raised instead of reporting an estimate from too few pulses."""


def affine_fit(
    frames: list[Frame], skip_startup: int = 3, blocks: int = 500
) -> AffineFit:
    """Least squares over (pts, publication). Startup frames excluded.

    Refit PER SESSION, never carried between sessions (review finding):
    only alpha stability within one session justifies pooling phases.
    """
    usable = frames[skip_startup:]
    if len(usable) < 32:
        raise InsufficientData(f"{len(usable)} frames after startup skip")
    alpha, beta = _least_squares(
        [f.pts_ns for f in usable], [f.publication_mono_ns for f in usable]
    )
    residuals = [
        f.publication_mono_ns - (alpha * f.pts_ns + beta) for f in usable
    ]
    sd = _stdev(residuals)
    block_alphas = []
    divergence = 0.0
    for start in range(0, len(usable) - blocks + 1, blocks):
        chunk = usable[start : start + blocks]
        block_alpha, block_beta = _least_squares(
            [f.pts_ns for f in chunk],
            [f.publication_mono_ns for f in chunk],
        )
        block_alphas.append(block_alpha)
        for edge in (chunk[0].pts_ns, chunk[-1].pts_ns):
            gap = abs(
                (block_alpha * edge + block_beta) - (alpha * edge + beta)
            )
            divergence = max(divergence, gap)
    return AffineFit(
        alpha, beta, sd, len(usable), tuple(block_alphas), divergence
    )


def median_period_ns(frames: list[Frame], fit: AffineFit) -> float:
    """The frame-grid clock: median mapped-PTS step over every RETAINED
    frame.

    This is the one circle phases are wrapped on. It reads the frame
    grid alone, so no classification outcome can move it - which is
    exactly what `miss_band` needs, since it judges a population that
    would otherwise be choosing its own clock.

    Retained, not every frame emitted: the loader has already dropped
    rows with missing, duplicate or regressing PTS, and each gap that
    leaves behind survives as one long step. The median absorbs those
    unless they outnumber the good intervals - at which point the
    session has no dominant frame period to find (review finding).
    """
    if len(frames) < 2:
        raise InsufficientData("need two frames for a period")
    steps = sorted(
        fit.map_pts(b.pts_ns) - fit.map_pts(a.pts_ns)
        for a, b in zip(frames, frames[1:])
    )
    return steps[len(steps) // 2]


def classify_pulses(
    frames: list[Frame],
    pulses: list[Pulse],
    fit: AffineFit,
    prior_offset_ns: float,
    lit_threshold: float,
    baseline_span: int = 4,
    guard_frames: int = 2,
) -> list[Classified]:
    """Timestamp-first pairing, then scores - never the other way.

    The candidate pair is the two frames whose mapped PTS straddle
    (pulse center + prior_offset); choosing "the lit frames" instead
    would select on the very scores being tested (review finding).
    `prior_offset_ns` comes from the coarse 08-30 measurement (~+100 ms:
    light lands in the frame PUBLISHED about that long after the GPIO
    edge). Its tolerance shrinks as the exposure fills the period, so
    a wrong prior is DETECTED, not silently absorbed: light outside the
    chosen pair classifies as "unexpected_lit" - adjust the prior until
    that count vanishes before trusting anything else.
    """
    mapped = [fit.map_pts(f.pts_ns) for f in frames]
    for a, b in zip(mapped, mapped[1:]):
        if b <= a:
            raise InsufficientData(
                "mapped PTS not strictly increasing; the loader must "
                "reject duplicate or regressing PTS before estimation"
            )
    median_period = median_period_ns(frames, fit)
    out: list[Classified] = []
    for pulse in pulses:
        predicted = pulse.center_ns + prior_offset_ns
        index = _straddle_index(mapped, predicted)
        if index is None:
            continue  # off the ends: terminal pulses handled by caller
        earlier, later = frames[index], frames[index + 1]
        baseline = _local_baseline(
            frames, index, baseline_span, guard_frames
        )
        if baseline is None:
            continue
        dep_e = earlier.roi_score - baseline
        dep_l = later.roi_score - baseline
        lit_e, lit_l = dep_e >= lit_threshold, dep_l >= lit_threshold
        neighbors_lit = _neighbors_lit(
            frames, index, baseline, lit_threshold, guard_frames
        )
        local_period = mapped[index + 1] - mapped[index]
        if not 0.8 * median_period < local_period < 1.2 * median_period:
            # A dropped frame or an exceptional PTS step is one long
            # "local period"; wrapping phases by it and pooling against
            # the others would mix two circular coordinates (review
            # finding). Named and excluded, not silently pooled.
            kind = "cadence_gap"
        elif neighbors_lit:
            kind = "unexpected_lit"
        elif lit_e and lit_l:
            kind = "split"
        elif lit_e or lit_l:
            kind = "single"
        else:
            kind = "miss"
        out.append(
            Classified(
                pulse,
                earlier,
                later,
                dep_e,
                dep_l,
                kind,
                pulse.center_ns - mapped[index],
                mapped[index + 1] - mapped[index],
            )
        )
    return out


def split_crossing_ns(
    classified: list[Classified],
    min_splits: int = 25,
    bootstrap: int = 400,
    seed: int = 20260901,
) -> tuple[float, float, float]:
    """The 50% point of "later frame brighter" vs phase, with a CI.

    Distribution-free: the threshold minimizing sign misclassification,
    reported as the midpoint of the optimal interval. Invariant under
    any single monotone response shared by the two frames - NOT under a
    response that changes between them, which is a stated assumption,
    not a theorem (review finding).

    Returns (crossing_ns, ci_low_ns, ci_high_ns).
    """
    splits = [c for c in classified if c.kind == "split"]
    if len(splits) < min_splits:
        raise InsufficientData(
            f"{len(splits)} split pulses; {min_splits} required"
        )
    points = sorted(
        (c.phase_ns, c.deposit_later > c.deposit_earlier) for c in splits
    )
    later_wins = sum(1 for _, later in points if later)
    if min(later_wins, len(points) - later_wins) < 5:
        raise InsufficientData(
            f"one-sided split signs ({later_wins} of {len(points)} "
            "later-brighter); the crossing is not inside the sampled "
            "phase range"
        )
    center = _best_threshold(points)
    rng = random.Random(seed)
    resampled = []
    for _ in range(bootstrap):
        draw = sorted(
            points[rng.randrange(len(points))] for _ in range(len(points))
        )
        resampled.append(_best_threshold(draw))
    resampled.sort()
    lo = resampled[int(0.025 * (len(resampled) - 1))]
    hi = resampled[int(0.975 * (len(resampled) - 1))]
    return center, lo, hi


def linear_split_crossing_ns(classified: list[Classified]) -> float:
    """DIAGNOSTIC ONLY, and what its disagreement actually means.

    Fits fraction = deposit_later / total against phase and solves for
    0.5. Under a symmetric pulse and ONE response shared by both
    frames the fraction is antisymmetric about the crossing for any
    monotone warp, so this agrees with `split_crossing_ns` even under
    gamma (a bias claim the unit tests corrected). The antisymmetry
    argument needs the sampled phases WEIGHTED symmetrically around
    the crossing, though - one-sided phase coverage or prior-induced
    truncation bends the linear root by several hundred microseconds
    even under a shared response (review finding). Disagreement on
    real data therefore flags EITHER a response differing between the
    two frames (AGC step, clipping, asymmetric pulse) OR asymmetric
    phase sampling - a data-quality alarm either way, never an
    alternative answer.
    """
    xs, fs = [], []
    for c in classified:
        if c.kind != "split":
            continue
        total = c.deposit_earlier + c.deposit_later
        if total <= 0.0:
            continue
        xs.append(c.phase_ns)
        fs.append(c.deposit_later / total)
    if len(xs) < 8:
        raise InsufficientData(f"{len(xs)} usable splits for linear fit")
    slope, intercept = _least_squares(xs, fs)
    if slope == 0.0:
        raise InsufficientData("flat linear split fit")
    return (0.5 - intercept) / slope


@dataclass(frozen=True, slots=True)
class MissBand:
    band_start_ns: float      # lit-to-miss edge, pushed EARLIER by delta
    band_end_ns: float        # miss-to-lit edge, pushed LATER by delta
    center_ns: float          # threshold-invariant: the handoff midpoint M
    exposure_width_ns: float  # P - w - band_width, biased low by 2*delta
    misses: int               # every miss, wherever its phase landed
    lit_samples: int          # single + split, NOT singles alone
    misses_inside: int        # misses whose phase lies in the fitted band
    misses_outside: int       # misses contradicting the band they helped set
    lit_inside: int           # lit samples in the band: 0 unless a tie
    period_ns: float          # the one circle every phase is wrapped on


def miss_band(
    classified: list[Classified],
    pulse_width_ns: float,
    period_ns: float,
    min_misses: int = 15,
    min_singles: int = 30,
) -> MissBand:
    """The no-light center-phase band, when E + w < P leaves one.

    The detection threshold delta WIDENS the observed band by delta on
    each side (a pulse only counts as lit once its overlap exceeds
    delta, so the last detected pulse sits delta short of each true
    edge). The displacements are equal and opposite, so the plain band
    midpoint cancels delta exactly under the rectangular model - and
    measured at pulse centers it needs no pulse-width correction: it
    IS the invariant M shared with the split crossing. The width, by
    the same token, is biased LOW by 2*delta.

    COHERENCE. The fit takes the largest contiguous miss run, so a band
    always comes back - even when the misses are scattered noise and the
    "run" is the widest accidental cluster. The model says otherwise:
    one dead-time interval per period admits exactly ONE circular run,
    so every miss must lie inside the band its own population set.
    `misses_outside` counts the ones that do not. It is measured here
    and judged by the caller; a nonzero count means this miss population
    cannot be one stationary dead-time band (sub-threshold deposits, a
    response that changed mid-session), and the center is then
    meaningless rather than noisy. It does not say which misses are
    real - only that the model behind the estimate does not hold.

    `period_ns` is the circle every phase is wrapped on and every
    returned phase lives on. Pass `median_period_ns(frames, fit)`.
    """
    # ONE circle for everything, and the CALLER supplies it. Wrapping
    # each sample by its own local period and then fitting on some
    # pooled circle displaces phases against each other (measured:
    # 13.4 us of local-period spread on real data), and under a
    # zero-tolerance coherence rule one such artifact would fail an
    # honest session. Deriving the circle from the classified pulses
    # instead would let the population being judged choose the clock
    # that judges it - the same circularity this gate exists to close.
    # `median_period_ns` reads the frame grid, which no classification
    # can move. Only kinds that contribute a phase are wrapped:
    # cadence_gap spans a dropped frame and unexpected_lit sits outside
    # its candidate pair, so neither places a point on this circle.
    period = float(period_ns)
    if not math.isfinite(period) or period <= 0.0:
        # NaN and +inf clear a bare `<= 0` test (review finding), and a
        # NaN circle makes every membership comparison False - the band
        # would look maximally incoherent instead of malformed. NOT
        # InsufficientData either: the caller catches that and would
        # file a bad circle under "too few pulses" in the report.
        raise ValueError(
            f"period_ns must be finite and positive, got {period_ns}"
        )
    lit_phases, miss_phases = [], []
    for c in classified:
        if c.kind not in ("single", "split", "miss"):
            continue
        wrapped = c.phase_ns % period
        if c.kind == "miss":
            miss_phases.append(wrapped)
        else:
            lit_phases.append(wrapped)
    if len(miss_phases) < min_misses or len(lit_phases) < min_singles:
        raise InsufficientData(
            f"{len(miss_phases)} misses / {len(lit_phases)} lit; "
            f"need {min_misses} / {min_singles}"
        )
    start, end = _largest_gap_of(miss_phases, lit_phases, period)
    width = (end - start) % period
    # Every phase now lives on the one circle the band was fitted on, so
    # membership is exact. A LIT sample inside the band is impossible by
    # construction except when a miss and a lit share a phase exactly -
    # counting them closes that tie rather than trusting it.
    inside = sum(1 for phase in miss_phases
                 if ((phase - start) % period) <= width)
    lit_inside = sum(1 for phase in lit_phases
                     if ((phase - start) % period) <= width)
    return MissBand(
        band_start_ns=start,
        band_end_ns=end,
        center_ns=(start + ((end - start) % period) / 2.0) % period,
        exposure_width_ns=period - pulse_width_ns - width,
        misses=len(miss_phases),
        lit_samples=len(lit_phases),
        misses_inside=inside,
        misses_outside=len(miss_phases) - inside,
        lit_inside=lit_inside,
        period_ns=period,
    )


# --- internals -------------------------------------------------------------

def _least_squares(xs: list, ys: list) -> tuple[float, float]:
    n = len(xs)
    mean_x, mean_y = sum(xs) / n, sum(ys) / n
    var = sum((x - mean_x) ** 2 for x in xs)
    if var == 0.0:
        raise InsufficientData("degenerate fit: no spread in x")
    cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = cov / var
    return slope, mean_y - slope * mean_x


def _stdev(values: list[float]) -> float:
    n = len(values)
    mean = sum(values) / n
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def _straddle_index(mapped: list[float], instant: float) -> int | None:
    """Index i with mapped[i] <= instant < mapped[i+1], else None."""
    lo, hi = 0, len(mapped) - 1
    if hi < 1 or instant < mapped[0] or instant >= mapped[hi]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if mapped[mid] <= instant:
            lo = mid
        else:
            hi = mid
    return lo


def _local_baseline(
    frames: list[Frame], index: int, span: int, guard: int
) -> float | None:
    """Median score of frames near the pair but outside the lit zone."""
    scores = []
    for offset in range(guard + 1, guard + 1 + span):
        for j in (index - offset, index + 1 + offset):
            if 0 <= j < len(frames):
                scores.append(frames[j].roi_score)
    return _median(scores) if len(scores) >= span else None


def _neighbors_lit(
    frames: list[Frame],
    index: int,
    baseline: float,
    threshold: float,
    guard: int,
) -> bool:
    """Light outside the candidate pair means the pairing prior is off."""
    for offset in range(1, guard + 1):
        for j in (index - offset, index + 1 + offset):
            if 0 <= j < len(frames):
                if frames[j].roi_score - baseline >= threshold:
                    return True
    return False


def _best_threshold(points: list[tuple[float, bool]]) -> float:
    """Midpoint of the interval minimizing sign misclassification."""
    n = len(points)
    later_left = 0
    later_total = sum(1 for _, later in points)
    best_errors, best_low, best_high = n + 1, points[0][0], points[0][0]
    # Cut before index i: predict "later brighter" for x >= cut.
    for i in range(n + 1):
        errors = later_left + ((n - i) - (later_total - later_left))
        if errors < best_errors:
            best_errors = errors
            best_low = points[i - 1][0] if i > 0 else points[0][0]
            best_high = points[i][0] if i < n else points[-1][0]
        if i < n and points[i][1]:
            later_left += 1
    return (best_low + best_high) / 2.0


def _largest_gap_of(
    misses: list[float], lits: list[float], period: float
) -> tuple[float, float]:
    """Circular band of misses with no lit phase inside it.

    Edges are midpoints between the outermost miss and its nearest lit
    neighbor, so both carry the same threshold displacement inward.
    """
    tagged = sorted(
        [(phase, False) for phase in misses] + [(phase, True) for phase in lits]
    )
    runs = []
    n = len(tagged)
    i = 0
    while i < n:
        if tagged[i][1]:
            i += 1
            continue
        j = i
        while j < n and not tagged[j][1]:
            j += 1
        runs.append((i, j - 1))
        i = j
    if not runs:
        raise InsufficientData("no contiguous miss run")
    # Circular merge: a run touching the end joins one touching the start.
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == n - 1:
        first, last = runs.pop(0), runs.pop(-1)
        runs.append((last[0], first[1]))
    def run_width(run):
        lo, hi = tagged[run[0]][0], tagged[run[1]][0]
        return (hi - lo) % period
    widest = max(runs, key=run_width)
    lo_idx, hi_idx = widest
    prev_lit = tagged[(lo_idx - 1) % n][0]
    next_lit = tagged[(hi_idx + 1) % n][0]
    start = (prev_lit + ((tagged[lo_idx][0] - prev_lit) % period) / 2.0) % period
    end = (tagged[hi_idx][0] + ((next_lit - tagged[hi_idx][0]) % period) / 2.0) % period
    return start, end
