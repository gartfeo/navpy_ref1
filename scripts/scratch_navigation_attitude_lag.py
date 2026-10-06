"""Attitude sampled at a stated instant, instead of whichever sample is newest.

WHY THIS EXISTS
---------------
The de-rotating pitch/roll and the visual ray come from two different MAVLink
messages. The ray is built from SIM_STATE, which carries no clock; the pose is
stamped with the ATTITUDE that closes SIM_STATE's interval. So the attitude
paired with a ray is systematically NEWER than the ray, and the error that
produces grows with roll rate.

An earlier harness answered this with an integer pose delay: keep a queue N poses
deep and read the oldest. That found a real effect -- misses improved several
fold -- but it cannot be the mechanism, and the numbers say so. The pairing skew
was MEASURED at 25 ms per interval with a host-side truth-to-attitude gap of
16 ms median, while the delay that wins is 50-75 ms. A correction cannot be two
to three times the error it corrects.

So this module does two separate things, and keeping them separate is the point:

  * It samples attitude at `t - lag`, INTERPOLATED between the two bracketing
    samples rather than snapped to one of them. An integer pose delay can only
    express multiples of the 25 ms interval, which is coarser than the effect
    being measured, and the quantisation is indistinguishable from the lag
    itself in a sweep.
  * It says nothing about what the lag MEANS. If the win is a pairing fix, the
    best lag is the pairing skew and it disappears when the ray and attitude
    come from one message. If the win is loop phase, the best lag survives that
    and is a property of the control loop, not the sensor. Those are different
    defects with different fixes, and one sweep cannot tell them apart unless
    the same lag can be applied to BOTH sources.

WHAT IT MAY READ
----------------
The autopilot's own clock, and nothing else. Host arrival times are recorded
elsewhere as a diagnostic and are deliberately not reachable from here: a
command path that consults the host clock produces a different run on a loaded
machine, which is the failure this whole harness is built to avoid.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

# Two samples bracket the request; more than a handful means the lag is being
# asked to reach further back than the stream can honestly support.
HISTORY_DEPTH = 64


@dataclass(frozen=True)
class AttitudeSample:
    """Attitude state at one instant on the AUTOPILOT's clock.

    Carries the body rates beside the angles so a lag can be applied to the
    WHOLE state. Lagging the angles alone leaves the law's rate filter mixing
    two time bases -- delayed angles differentiated against a current yaw
    rate -- and whether that mixture or pure loop phase carries the measured
    win is exactly the open question; a sampler that cannot lag the rates
    cannot ask it. Rates default to zero so angle-only uses stay unchanged.
    """

    t_s: float
    pitch_deg: float
    roll_deg: float
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0


def _interpolate_angle(earlier: float, later: float, fraction: float) -> float:
    """Shortest-arc interpolation, so a wrap does not invent a whole rotation.

    Roll stays well inside +-180 in this regime, but interpolating the raw
    difference would turn a -179 to +179 step into a 358 degree sweep, and the
    resulting command would be a full-authority reversal at exactly the moment
    the aircraft is most sensitive to one.
    """
    delta = math.degrees(
        math.atan2(
            math.sin(math.radians(later - earlier)),
            math.cos(math.radians(later - earlier)),
        )
    )
    return earlier + delta * fraction


class AttitudeLagSampler:
    """Attitude at `t - lag`, or nothing.

    Returns None rather than a nearest sample when the request cannot be
    bracketed. That matters more than it looks: substituting the nearest sample
    is exactly the defect this class exists to remove, and doing it silently
    during the first `lag` seconds of every scoring interval would put the worst
    pairing error precisely where the geometry is least forgiving.
    """

    def __init__(self, lag_s: float = 0.0, depth: int = HISTORY_DEPTH) -> None:
        if lag_s < 0.0:
            raise ValueError(
                f"lag must not be negative, got {lag_s}: a negative lag asks "
                "for attitude the aircraft has not reported yet"
            )
        self._lag_s = float(lag_s)
        self._history: deque[AttitudeSample] = deque(maxlen=depth)
        self._unbracketed = 0

    @property
    def lag_s(self) -> float:
        return self._lag_s

    @property
    def unbracketed(self) -> int:
        """Requests refused for want of a bracket, reported not hidden."""
        return self._unbracketed

    def observe(self, sample: AttitudeSample) -> None:
        # Out-of-order or repeated stamps are dropped rather than sorted in: the
        # stream is monotonic on the autopilot clock, so a backward step means
        # something upstream is wrong and interpolating across it would average
        # two different instants into one plausible-looking attitude.
        if self._history and sample.t_s <= self._history[-1].t_s:
            return
        self._history.append(sample)

    def at(self, t_s: float) -> AttitudeSample | None:
        """Attitude `lag` seconds before `t_s`, strictly bracketed."""
        wanted = t_s - self._lag_s
        if len(self._history) < 2:
            self._unbracketed += 1
            return None
        # Zero lag is the ordinary case and must stay exact: the newest sample IS
        # the request, and running it through interpolation would depend on
        # floating-point luck to reproduce the undelayed arm.
        if wanted >= self._history[-1].t_s:
            if self._lag_s == 0.0:
                return self._history[-1]
            # A lag that lands in the FUTURE of the stream is a stalled feed,
            # not a rounding error. Refused, because the alternative is to keep
            # flying on an attitude whose age nobody knows.
            self._unbracketed += 1
            return None
        if wanted < self._history[0].t_s:
            self._unbracketed += 1
            return None
        for earlier, later in zip(self._history, list(self._history)[1:]):
            if earlier.t_s <= wanted <= later.t_s:
                span = later.t_s - earlier.t_s
                fraction = 0.0 if span <= 0.0 else (wanted - earlier.t_s) / span
                return AttitudeSample(
                    wanted,
                    _interpolate_angle(
                        earlier.pitch_deg, later.pitch_deg, fraction),
                    _interpolate_angle(
                        earlier.roll_deg, later.roll_deg, fraction),
                    # Rates interpolate linearly: they are not periodic, so
                    # there is no wrap to take the short way around.
                    earlier.roll_rate_rad_s + fraction * (
                        later.roll_rate_rad_s - earlier.roll_rate_rad_s),
                    earlier.pitch_rate_rad_s + fraction * (
                        later.pitch_rate_rad_s - earlier.pitch_rate_rad_s),
                    earlier.yaw_rate_rad_s + fraction * (
                        later.yaw_rate_rad_s - earlier.yaw_rate_rad_s),
                )
        self._unbracketed += 1
        return None


def check_lag_options(options: "object") -> None:
    """Refuse a lag that cannot mean anything, before the aircraft flies.

    The two-knob refusal is the important one. `--estimate-delay-poses` and
    `--estimate-lag-s` are the SAME axis measured two ways, so a run carrying
    both produces a number neither knob can claim -- and it looks like a valid
    result for whichever one the reader happens to be studying.
    """
    lag_s = getattr(options, "estimate_lag_s", 0.0)
    delay = getattr(options, "estimate_delay_poses", 0)
    if lag_s < 0.0:
        raise SystemExit(
            f"--estimate-lag-s must not be negative, got {lag_s}: a negative "
            "lag asks for attitude the aircraft has not reported yet"
        )
    if delay and lag_s:
        raise SystemExit(
            "give --estimate-delay-poses OR --estimate-lag-s, not both: they "
            "are the same axis measured two ways, and a run carrying both "
            "cannot say which produced its number"
        )
