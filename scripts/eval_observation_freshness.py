"""Whether a run fed the navigation law enough fresh observations to be scored.

This is a VALIDITY gate, not a navigation result: it decides whether a run
measured navigation quality at all, or merely measured host contention. It lived
inside `eval_direct_pixel_pn.py` while three evaluators reached across into that
module for it -- the single-vehicle path, the three-vehicle path and
`scratch_direct_geo_eval.py`. Shared logic owned by one caller is how the paths
drift apart on what counts as a scoreable run, so it owns itself here.
"""

from __future__ import annotations

import csv
from pathlib import Path

# Measured three-UAV baseline: 1x delivers 88.4-89.8% of projected frames across
# 48 vehicles with no spread.  Degraded 10x runs deliver 57.0-74.0%, and their
# 3-D CPA rises monotonically as the fraction falls (74.0% -> 0.35 m,
# 57.0% -> 8.42 m).  0.80 sits below every healthy run and above every degraded
# one, so a run under it is reporting host contention rather than navigation
# quality and must not be scored as a navigation result.
MIN_FRESH_OBSERVATION_FRACTION = 0.80
# Absolute floor, in vehicle time, on fresh observations reaching the law.
#
# The fraction alone is not sufficient: if host load also starves the pose
# stream, projections fall with deliveries and the ratio stays high while the
# law sees almost nothing.  Ten frames delivered out of ten projected is 100%
# and still worthless.  Conversely a run whose sensor produced MORE than usual
# could dip below the fraction while still feeding the law plenty.  Gate both.
#
# The sim pose stream runs at 40 Hz and healthy 1x runs deliver 35.3-36.1 Hz of
# it; degraded runs deliver 21.0-26.5 Hz.  30 Hz sits between the two bands.
MIN_FRESH_OBSERVATION_RATE_HZ = 30.0


def scoring_interval_span_s(case_dir: Path, speedup: float) -> float:
    """Guided-flight duration in vehicle time.

    Only the first and last timestamps of the actuator trace are used.  The
    trace is written by a 10 ms polling loop in ``direct_pixel_pn_child``, so
    its row spacing measures that poller and says nothing about command or
    observation cadence; its span is still an honest wall duration of the
    guided leg, which is all this needs.
    """
    path = case_dir / "flight_control_trace.csv"
    stamps: list[float] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                stamps.append(float(row["wall_s"]))
            except (KeyError, TypeError, ValueError):
                continue
    if len(stamps) < 10:
        raise RuntimeError(f"insufficient scoring-window samples: {len(stamps)}")
    span_s = (stamps[-1] - stamps[0]) * speedup
    if span_s <= 0.0:
        raise RuntimeError("command trace has no advancing timestamps")
    return span_s


def observation_freshness(
    case_dir: Path,
    speedup: float,
    source: dict[str, object],
) -> dict[str, float | int]:
    """Rate of fresh observations that actually reached the navigation law.

    ``projected_frames`` counts what the sim sensor produced; ``delivered_frames``
    counts what navigation consumed.  The difference is frames overwritten in the
    source's newest-only slot before any command slot drained them, so the law
    never saw them.  Rates are reported in vehicle time because that is the
    timebase the navigation law and the airframe share.
    """
    projected = source.get("projected_frames")
    delivered = source.get("delivered_frames")
    if not isinstance(projected, int) or not isinstance(delivered, int):
        raise RuntimeError(
            f"source frame counts missing: projected={projected!r} "
            f"delivered={delivered!r}"
        )
    if projected <= 0:
        raise RuntimeError(f"no projected frames: {projected}")
    span_s = scoring_interval_span_s(case_dir, speedup)
    rejections = source.get("delivery_rejections")
    return {
        "engagement_s": span_s,
        "projected_frames": projected,
        "delivered_frames": delivered,
        "overwritten_frames": projected
        - delivered
        - (rejections if isinstance(rejections, int) else 0),
        "projected_rate_hz": projected / span_s,
        "delivered_rate_hz": delivered / span_s,
        "fresh_fraction": delivered / projected,
        "minimum_fresh_fraction": MIN_FRESH_OBSERVATION_FRACTION,
    }


def freshness_errors(freshness: dict[str, float | int]) -> list[str]:
    """Validity errors for one run's observation freshness.

    Shared by every evaluator so the single-vehicle and three-vehicle paths
    cannot drift apart on what counts as a scoreable run.
    """
    errors: list[str] = []
    fraction = float(freshness["fresh_fraction"])
    delivered_hz = float(freshness["delivered_rate_hz"])
    if fraction < MIN_FRESH_OBSERVATION_FRACTION:
        errors.append(
            f"fresh observations {fraction * 100.0:.1f}% below "
            f"{MIN_FRESH_OBSERVATION_FRACTION * 100.0:.1f}%: the law saw "
            f"{delivered_hz:.1f}Hz of "
            f"{float(freshness['projected_rate_hz']):.1f}Hz produced; run "
            f"measures host contention, not navigation"
        )
    if delivered_hz < MIN_FRESH_OBSERVATION_RATE_HZ:
        errors.append(
            f"fresh observation rate {delivered_hz:.1f}Hz below "
            f"{MIN_FRESH_OBSERVATION_RATE_HZ:.1f}Hz: the law was starved "
            f"regardless of the delivered fraction"
        )
    return errors


__all__ = [
    "MIN_FRESH_OBSERVATION_FRACTION",
    "MIN_FRESH_OBSERVATION_RATE_HZ",
    'scoring_interval_span_s',
    "freshness_errors",
    "observation_freshness",
]
