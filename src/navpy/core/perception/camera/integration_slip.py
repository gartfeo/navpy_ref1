"""Slip checks for the integration timing map (HITL stage 1 step 11).

Slip checks use the floor of ``read - pts``: its rate is fitted over the
``rate_window_s`` that ends where the two newest slip windows begin (so a slip
inside them cannot bend it), its level is the rate-corrected minimum of that
same window: the fitted hull line itself, so jitter does not lift it the way
the minimum of one short window can. A slip is:
- a frame read more than ``slip_limit_s`` below the floor (no jitter is
  negative);
- every frame of the newest ``slip_window_s`` more than ``slip_limit_s``
  above the floor (a longer video delay). Frames of that window published
  before the abort were mapped with up to the jump as error: a stated limit
  (owner ruling 2026-11-02).
"""

from __future__ import annotations

from collections.abc import Sequence

from navpy.core.perception.camera.integration_timing import NS_PER_S, IntegrationTimingLimits

Sample = tuple[int, float]


class FloorSlipDetector:
    """Keeps the drift-corrected floor and names a frame that shows a slip against it."""

    def __init__(self, limits: IntegrationTimingLimits) -> None:
        self._slip_window_ns = round(limits.slip_window_s * NS_PER_S)
        self._rate_window_ns = round(limits.rate_window_s * NS_PER_S)
        self._slip_limit_s = limits.slip_limit_s
        self._rate: float | None = None
        self._rate_fitted_at_ns: int | None = None

    @property
    def rate(self) -> float | None:
        return self._rate

    def check(self, samples: Sequence[Sample], pts_ns: int, sample: float) -> str | None:
        """Why this frame shows a slip, against the drift-corrected floor; None if it does not.

        Checked before the frame joins ``samples``, which hold every frame of
        the open within the history the map keeps (a PTS gap aborts it).
        """
        fit_end_ns = pts_ns - 2 * self._slip_window_ns
        if self._rate_fitted_at_ns is None or fit_end_ns - self._rate_fitted_at_ns >= self._slip_window_ns:
            # The rate is a slow property: refit once per slip window.
            rate = _floor_rate(samples, fit_end_ns - self._rate_window_ns, fit_end_ns)
            if rate is not None:
                self._rate = rate
                self._rate_fitted_at_ns = fit_end_ns
        rate = self._rate
        level = None if rate is None else _minimum(samples, fit_end_ns - self._rate_window_ns, fit_end_ns, rate=rate)
        if level is None:
            return None
        floor = level[1]
        limit = self._slip_limit_s
        if sample - rate * pts_ns / NS_PER_S < floor - limit:
            return "slip: a frame read earlier than the recent floor allows"
        current = _minimum(
            samples, pts_ns - self._slip_window_ns, pts_ns, rate=rate, extra=(pts_ns, sample),
        )
        if current is not None and current[1] - floor > limit:
            return "slip: the slip window's floor moved"
        return None


def _floor_rate(samples: Sequence[Sample], start_ns: int, end_ns: int) -> float | None:
    """Slope (s/s) of the line under every sample of [start, end), or None unless frames cover it.

    The linear-programming estimate of clock skew from one-way delays
    (Moon, Skelly and Towsley, 1999): of the lines below every sample, the
    one closest to them on average is the lower convex hull edge that
    spans the samples' mean PTS.
    """
    if _minimum(samples, start_ns, end_ns) is None:
        return None
    points = [(pts, value) for pts, value in samples if start_ns <= pts < end_ns]
    hull: list[Sample] = []
    for point in points:
        while len(hull) >= 2 and _turn(hull[-2], hull[-1], point) <= 0.0:
            hull.pop()
        hull.append(point)
    mean_ns = sum(pts for pts, _ in points) / len(points)
    if len(hull) < 2:
        return None
    for left, right in zip(hull, hull[1:]):
        if right[0] >= mean_ns:
            break
    return (right[1] - left[1]) / ((right[0] - left[0]) / NS_PER_S)


def _minimum(
    samples: Sequence[Sample],
    start_ns: int,
    end_ns: int,
    rate: float = 0.0,
    extra: Sample | None = None,
) -> Sample | None:
    """(pts, sample - rate * pts) of the smallest such value with start <= pts < end, or None unless frames cover all of it."""
    if not samples or samples[0][0] > start_ns:
        return None
    found = [(pts, value) for pts, value in samples if start_ns <= pts < end_ns]
    if extra is not None:
        found.append(extra)
    detrended = [(pts, value - rate * pts / NS_PER_S) for pts, value in found]
    return min(detrended, key=lambda item: item[1]) if detrended else None


def _turn(first: Sample, middle: Sample, last: Sample) -> float:
    """Positive when middle lies below the chord from first to last (a lower hull keeps it)."""
    return (middle[0] - first[0]) * (last[1] - first[1]) - (middle[1] - first[1]) * (last[0] - first[0])


__all__ = ["FloorSlipDetector"]
