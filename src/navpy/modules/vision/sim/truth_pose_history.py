"""Two-deep truth-pose history that interpolates the pose at a clock.

WHY THIS EXISTS
---------------
Stock SIM_STATE carries no clock, so the direct-pixel source pairs each truth
sample with the NEXT ATTITUDE receipt. Rendering from the raw held sample
means the ray uses a pose up to the association gate (two pose periods) OLDER
than the attitude that de-rotates it, and the age CHANGES from frame to
frame; the navigation law differentiates bearing, so the changing age lands as
line-of-sight rate noise. (The ``source`` axis from ``truth_pose_time_axis``
positions samples by the SIM_STATE ``time_us`` stamp instead, making the
pairing exact.)

The fix is to answer at the clock itself -- and only from INSIDE a bracketing
pair: extrapolating past the newest sample measurably was not good enough
(any angular ACCELERATION leaves ~0.07 deg of bearing residual per sample,
which the law's 27 ms differentiator turns back into rate noise). So
``pose_at`` refuses (returns ``None``) rather than guessing, and the direct
source DROPS a refused clock instead of rendering the raw held sample.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.sim.truth_pose_time_axis import (
    SOURCE_CLOCK_QUANTUM_S,
    TIME_AXES,
    TIME_AXIS_RECEIPT,
    TIME_AXIS_SOURCE,
    TRUTH_POSE_TIME_AXIS_ENV,
    resolve_truth_pose_time_axis,
)


def wrap_degrees(angle_deg: float) -> float:
    """Map an angle onto [-180, 180) so axis crossings stay small."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


# Receipts are HOST wall stamps that TRACK the true emission spacing of
# SIM_STATE rather than jittering around a fixed pose period (scored-leg
# diagnostics 2026-08-24). The pair's OWN gap therefore positions the clock
# — the truth-scored A/B put it at 0.058/0.088 m median vs 0.133 m for a
# fixed one-period denominator (detour: commits 4cbede1e5..45fd13c0d). A
# wider-than-gate pair is stale and refuses; equal or regressing stamps fail
# closed in note_current; interpolation bounds the answer between measured
# samples however small the gap, so the tiny-gap transport artifact
# (commit b0a69fe18) cannot fabricate motion.
MAX_GAP_SPAN_FRACTION = 1.0


@dataclass(frozen=True)
class _TruthSample:
    location: Location
    attitude: Attitude
    receipt_s: float
    # FDM state-sample time (autopilot clock) from the SIM_STATE ``time_us``
    # extension; None on stock firmware that does not fill it.
    source_s: float | None


def _interpolated_angle_deg(
    newest_deg: float, previous_deg: float, fraction: float
) -> float:
    """Walk back from the newest sample by ``fraction`` (0 = newest, -1 =
    previous) of the pair's step; never leaves the arc the samples span."""
    step = wrap_degrees(newest_deg - previous_deg) * fraction
    return wrap_degrees(newest_deg + step)


def _new_tallies() -> dict[str, float]:
    return {
        "accepted": 0.0,
        "unpaired": 0.0,
        "gap_wide": 0.0,
        "gap_wide_sum_s": 0.0,
        "gap_wide_min_s": math.inf,
        "gap_wide_max_s": 0.0,
        "forward_refused": 0.0,
        "source_axis_fallback": 0.0,
    }


def _read_source_time(pose: object) -> float | None:
    """The pose's source stamp, or None when absent or unusable."""
    raw = getattr(pose, "source_time_s", None)
    if raw is None:
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


class TruthPoseHistory:
    """Keep the two newest truth samples and interpolate between them.

    Not thread-safe by itself: the owning source holds its lock around every
    call that touches the pair or the tallies, because ``reset_pair`` runs on
    the activation thread while samples arrive on the message-dispatch
    thread.
    """

    def __init__(
        self,
        truth_pose: Callable[[], object | None],
        max_span_s: Callable[[], float],
        *,
        time_axis: str = TIME_AXIS_RECEIPT,
    ) -> None:
        if time_axis not in TIME_AXES:
            raise ValueError(
                f"time_axis must be one of {TIME_AXES}, got {time_axis!r}"
            )
        self._truth_pose = truth_pose
        self._max_span_s = max_span_s
        self._axis_in_use = time_axis
        self._previous: _TruthSample | None = None
        self._newest: _TruthSample | None = None
        self._tallies = _new_tallies()

    @property
    def axis_in_use(self) -> str:
        """The clock currently positioning samples and queries.

        Starts as the requested axis; drops to ``receipt`` (sticky) the first
        time a truth sample arrives without a source stamp. The caller must
        hand ``pose_at`` clocks from the SAME axis, so it reads this.
        """
        return self._axis_in_use

    def _sample_time(self, sample: _TruthSample) -> float:
        if self._axis_in_use == TIME_AXIS_SOURCE:
            # note_current falls back to receipt before storing a sample
            # without a source stamp, so on this axis the stamp exists.
            return float(sample.source_s)  # type: ignore[arg-type]
        return sample.receipt_s

    def _time_units(self, time_s: float) -> float:
        """Ordering units: integer microseconds on the source axis, so two
        conversions of one autopilot tick compare EQUAL (see
        ``SOURCE_CLOCK_QUANTUM_S``); raw seconds on the receipt axis."""
        if self._axis_in_use == TIME_AXIS_SOURCE:
            return float(round(time_s / SOURCE_CLOCK_QUANTUM_S))
        return time_s

    def _sample_units(self, sample: _TruthSample) -> float:
        return self._time_units(self._sample_time(sample))

    def _units_scale_s(self) -> float:
        if self._axis_in_use == TIME_AXIS_SOURCE:
            return SOURCE_CLOCK_QUANTUM_S
        return 1.0

    def reset_pair(self) -> None:
        """Drop the held pair so no rate ever spans a cadence change."""
        self._previous = None
        self._newest = None

    def note_current(self) -> bool:
        """Record the newest truth sample; False if there was nothing to add
        (the caller renders on the truth message that CLOSES a bracket, and a
        clock past an unchanged newest sample would be extrapolation)."""
        pose = self._truth_pose()
        if pose is None:
            return False
        sample = _TruthSample(
            location=pose.location,
            attitude=pose.attitude,
            receipt_s=float(pose.receipt_time_s),
            source_s=_read_source_time(pose),
        )
        if self._axis_in_use == TIME_AXIS_SOURCE and sample.source_s is None:
            # Stock firmware: no time_us. Fall back STICKY to receipt stamps
            # (per-sample flips would mix clocks inside one pair) and drop
            # the held pair outright — it must not straddle the flip.
            self._axis_in_use = TIME_AXIS_RECEIPT
            self._tallies["source_axis_fallback"] += 1.0
            logging.getLogger(__name__).warning(
                "truth-pose time axis 'source' requested but SIM_STATE has "
                "no time_us stamp; falling back to receipt stamps"
            )
            self._previous = None
            self._newest = sample
            return True
        newest = self._newest
        if newest is not None and self._sample_units(
            sample
        ) <= self._sample_units(newest):
            # Equal stamps can be two DISTINCT packets in one clock quantum;
            # a regressing stamp means the axis cannot be trusted. Fail
            # CLOSED: keep the content, drop the pair, wait for clean order.
            self._previous = None
            self._newest = sample
            return True
        self._previous = newest
        self._newest = sample
        return True

    def pose_at(self, time_s: float) -> tuple[Location, Attitude] | None:
        previous, newest = self._previous, self._newest
        if previous is None or newest is None:
            self._tallies["unpaired"] += 1.0
            return None
        span_s = self._max_span_s()
        gap_units = self._sample_units(newest) - self._sample_units(previous)
        forward_units = self._time_units(float(time_s)) - self._sample_units(
            newest
        )
        gap_s = gap_units * self._units_scale_s()
        if gap_s > span_s * MAX_GAP_SPAN_FRACTION:
            tallies = self._tallies
            tallies["gap_wide"] += 1.0
            tallies["gap_wide_sum_s"] += gap_s
            tallies["gap_wide_min_s"] = min(tallies["gap_wide_min_s"], gap_s)
            tallies["gap_wide_max_s"] = max(tallies["gap_wide_max_s"], gap_s)
            return None
        if not -gap_units <= forward_units <= 0.0:
            # INSIDE the pair or nothing: past the newest sample would be
            # extrapolation, the defect this class was written to remove.
            self._tallies["forward_refused"] += 1.0
            return None
        # 0 at the newest sample, -1 at the previous one; no denominator
        # floor (it would walk the answer off the clock it was asked for).
        fraction = forward_units / gap_units
        location = Location(
            lat=newest.location.lat
            + (newest.location.lat - previous.location.lat) * fraction,
            lng=newest.location.lng
            + wrap_degrees(newest.location.lng - previous.location.lng)
            * fraction,
            alt=newest.location.alt
            + (newest.location.alt - previous.location.alt) * fraction,
            is_absolute=newest.location.is_absolute,
        )
        attitude = Attitude(
            pitch=_interpolated_angle_deg(
                newest.attitude.pitch, previous.attitude.pitch, fraction
            ),
            yaw=_interpolated_angle_deg(
                newest.attitude.yaw, previous.attitude.yaw, fraction
            ),
            roll=_interpolated_angle_deg(
                newest.attitude.roll, previous.attitude.roll, fraction
            ),
        )
        self._tallies["accepted"] += 1.0
        return location, attitude

    @property
    def diagnostics(self) -> dict[str, float]:
        """Refusal/acceptance tallies for auditing advance conditioning."""
        tallies = self._tallies
        wide = tallies["gap_wide"]
        return {
            "accepted": tallies["accepted"],
            "unpaired": tallies["unpaired"],
            "gap_wide": wide,
            "gap_wide_min_ms": (
                tallies["gap_wide_min_s"] * 1000.0 if wide else 0.0
            ),
            "gap_wide_mean_ms": (
                tallies["gap_wide_sum_s"] / wide * 1000.0 if wide else 0.0
            ),
            "gap_wide_max_ms": tallies["gap_wide_max_s"] * 1000.0,
            "forward_refused": tallies["forward_refused"],
            "source_axis_fallback": tallies["source_axis_fallback"],
        }

    def reset_diagnostics(self) -> None:
        """Restart the tallies; the owner calls this when scoring begins.

        ``source_axis_fallback`` survives on purpose: a fallback is a
        configuration fact about the whole run (firmware without stamps),
        and clearing it at scoring start would hide that the scored leg is
        NOT flying the axis the A/B arm asked for.
        """
        fallback = self._tallies["source_axis_fallback"]
        self._tallies = _new_tallies()
        self._tallies["source_axis_fallback"] = fallback


__all__ = [
    "MAX_GAP_SPAN_FRACTION",
    "SOURCE_CLOCK_QUANTUM_S",
    "TIME_AXIS_RECEIPT",
    "TIME_AXIS_SOURCE",
    "TRUTH_POSE_TIME_AXIS_ENV",
    "TruthPoseHistory",
    "resolve_truth_pose_time_axis",
    "wrap_degrees",
]
