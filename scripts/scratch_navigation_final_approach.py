"""What the roll command did, and how the aircraft answered.

Split out of the scoring interval loop because it is a whole concern rather than a
few variables: six series, two windows and one report section, all answering
the same question and none of them part of flying the aircraft. Held inline,
they were six separate locals threaded through three call sites and six result
keys, and every addition to them grew the one function that already does the
flying.

The split is also the only thing that keeps command and response comparable.
Reading the aircraft's actual roll is worth nothing except AGAINST the command
that produced it, so both go through the same summariser here rather than
through two that could drift apart -- a different sign band, a different step
convention -- and quietly stop meaning the same thing.
"""

from __future__ import annotations

import math
from typing import Protocol, Sequence

from scripts.scratch_navigation_result import RollSeries, _spread


class LateralRate(Protocol):
    """The two terms the law's roll command is built from.

    Structural, not imported: this module reports on the law, and importing
    its types would make the report a dependency of the thing it observes.
    """

    visual_rate_rad_s: float
    raw_inertial_rate_rad_s: float


class FinalApproachGeometry:
    """Is the final-approach path STRAIGHT, and is the command that flies it steady?

    An approach error cannot answer either question. Two runs that both stop 3 cm
    from the POI are indistinguishable in the score while one flew a settled
    straight line and the other spiralled in on a command reversing three times
    a second -- and only the first of those transfers to an airframe that rolls
    faster than this one does.

    Whole-run statistics cannot answer them either, which is why this window
    exists at all: a median roll taken over the whole scoring interval mixes the turn
    onto the constant-bearing course with the settled run that follows it, so a run
    that never settles and one that settles immediately report the same number.

    Straightness is measured in METRES, as the furthest the flown path departs
    from the straight line joining the ends of the closing window. A sustained
    bank appears as its own sagitta, which is what makes the number readable: a
    3.4 degree bank at 27 m/s bends 56 m over a 750 m window.

    LOS BEARING ROTATION was tried first and abandoned, because it looks like
    the natural measure and is not. Against a static POI an aircraft headed
    straight for it does hold a constant bearing, so rotation of that bearing
    is the path bending -- but the bearing also sweeps through half a turn as
    the POI is passed, ~90 degrees of it BEFORE the closest approach and the
    rest immediately after. That sweep is geometry, not flying, and it swamps
    any real curvature: every cell of a 30-aircraft matrix reported 170-195
    degrees, the dead-straight ones included. No boundary excludes it without
    inventing a threshold, and the metre figure needs none.

    The window is gated on RANGE, not time. "Final approach" has to mean close to the
    POI: a time gate covers a different part of the geometry at every
    airspeed, which is the confound the throttle sweep exists to avoid.

    Report only. Positions and ranges here are simulator truth, and none of it
    reaches a frame.
    """

    def __init__(self, range_fraction: float = 0.25) -> None:
        self._fraction = range_fraction
        self._gate_m: float | None = None
        self._points: list[tuple[float, float]] = []
        self._ranges: list[float] = []
        self._inside = False
        self.commanded = RollSeries()
        self.actual = RollSeries()
        # The law's lateral rate, split into the two terms it is built from.
        # The roll command is a direct function of the second, so if the
        # command dithers, one of these two is where the dither comes from --
        # and they have different fixes. The visual rate is a two-point bearing
        # difference; the inertial rate adds the gyro's yaw rate to it.
        self._visual: list[float] = []
        self._inertial: list[float] = []

    def observe_geometry(
        self, range_m: float, offset_ned_m: Sequence[float]
    ) -> None:
        """One truth pose: where the POI sits, and how far away it is."""
        if self._gate_m is None:
            # The FIRST range seen is the scoring interval range, so the gate is a
            # property of the run rather than a constant that would cover a
            # different fraction of every geometry.
            self._gate_m = range_m * self._fraction
        # Recomputed from the CURRENT range every pose, never latched. This is
        # a window, not a trigger: the range is not monotonic -- it grows again
        # once the POI is passed, and a wandering scoring interval can cross the
        # gate more than once -- so a flag that only ever turned on would keep
        # charging commands and rates to a window the aircraft had already
        # left, while the geometry series correctly stopped. That mismatch is
        # invisible in the artifact, because the two are summarised separately.
        self._inside = range_m <= self._gate_m
        if not self._inside:
            return
        north, east = float(offset_ned_m[0]), float(offset_ned_m[1])
        self._ranges.append(range_m)
        # Aircraft position in a POI-CENTRED frame: the offset points from
        # the aircraft to the POI, so the aircraft sits at its negation.
        self._points.append((-north, -east))

    def observe_command(
        self, commanded_deg: float, actual_deg: float, rate: LateralRate
    ) -> None:
        """One command instant, and the rate that command was built from.

        Taken together because they ARE one event: the rate is why the command
        moved, and recording them through separate calls let one be gated and
        the other not.
        """
        if not self._inside:
            return
        self.commanded.observe(commanded_deg)
        self.actual.observe(actual_deg)
        self._visual.append(math.degrees(rate.visual_rate_rad_s))
        self._inertial.append(math.degrees(rate.raw_inertial_rate_rad_s))

    @staticmethod
    def _jitter(series: list[float]) -> dict | None:
        """Magnitude and SAMPLE-TO-SAMPLE STEP of a rate series, deg/s.

        The step is the number that matters. A rate holding a steady 0.4 deg/s
        and a rate jumping between +0.4 and -0.4 have the same magnitude, and
        the roll command they produce is a steady bank in the first case and a
        reversal every sample in the second.
        """
        if not series:
            return None
        steps = [abs(b - a) for a, b in zip(series, series[1:])]
        return {
            "magnitude": _spread([abs(v) for v in series]),
            "step": _spread(steps),
        }

    def _closing(self) -> int:
        """How many samples belong to the APPROACH, not to the fly-away.

        Everything after the closest approach is the aircraft leaving, and
        including it destroys the measure. No threshold is involved: the
        closest approach IS the boundary, and it is read off the range series
        rather than assumed.
        """
        if not self._ranges:
            return 0
        return self._ranges.index(min(self._ranges)) + 1

    def _chord_deviation(self) -> float | None:
        """How far the path bends, or None when it was never measured.

        None rather than 0.0, because zero is not a neutral value here: it is
        the reading a DEAD-STRAIGHT run produces, so returning it for a run
        that supplied no points reports the best possible result for missing
        data. A run that passes wide enough never to enter the range gate --
        the exact case where straightness is most worth knowing -- would have
        been published as a flawless straight line, and nothing downstream
        would contradict it, because a wide pass is still a valid miss and so
        raises no classification error.

        Two ways the measure has no answer: fewer than three points, which
        leaves no interior point to be off the chord, and a chord of zero
        length, whose ends coincide so there is no line to be off.
        """
        points = self._points[:self._closing()]
        if len(points) < 3:
            return None
        (x0, y0), (x1, y1) = points[0], points[-1]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length <= 0.0:
            return None
        worst = 0.0
        for x, y in points[1:-1]:
            worst = max(worst, abs(dy * (x - x0) - dx * (y - y0)) / length)
        return worst

    def summary(self) -> dict:
        bend = self._chord_deviation()
        return {
            "range_gate_m": (None if self._gate_m is None
                             else round(self._gate_m, 1)),
            "samples": len(self._ranges),
            # Of those, how many were still closing. The rest are the fly-away,
            # excluded from the straightness measure below.
            "closing_samples": self._closing(),
            # STRAIGHTNESS, in metres. Zero is a dead-straight run at the
            # POI; a held bank reads as its own sagitta. None means the
            # window held too few points to say, which is NOT zero.
            "chord_deviation_m": (None if bend is None else round(bend, 3)),
            # DETERMINISM. The command and the roll the airframe actually flew,
            # over the SAME window, so a command that dithers while the airframe
            # filters it cannot read as a steady one.
            "roll_commanded": self.commanded.summary(),
            "roll_actual": self.actual.summary(),
            # WHERE a dithering command comes from. The lateral channel runs its
            # rate through no filter at all while the vertical channel filters
            # at VERTICAL_RATE_FILTER_TAU_S, so these two series are the
            # evidence for whether that asymmetry is what shakes the roll
            # command -- and which of the two terms is responsible.
            "lateral_rate_visual_deg_s": self._jitter(self._visual),
            "lateral_rate_inertial_deg_s": self._jitter(self._inertial),
        }


class RollRecord:
    """Every roll series the run reports, and the one window that matters.

    Owns the whole-run summaries, the final-approach-window ones, the command-vs-
    response tracking error, and when the ill-conditioning guard held the
    command. They are grouped because they are read together and meaningless
    apart: a tracking error without the command it was measured against says
    nothing, and a guard that fired says nothing without when it fired.
    """

    def __init__(self) -> None:
        self.commanded = RollSeries()
        self.actual = RollSeries()
        # Recorded ONLY into the report, never into a frame. The attenuation
        # the other two series measure would be overstated if the aircraft's
        # own estimate smooths or lags its roll, and nothing here could tell:
        # the estimate is the only actual-roll source. Simulator truth answers
        # it directly, and it is safe to read for a REPORT because two other
        # guards already pin the law's only input to `latest_frame` and the
        # frame's only truth arguments to the ray.
        self.truth = RollSeries()
        self.final_approach = FinalApproachGeometry()
        self._tracking_errors: list[float] = []
        # When the law HELD the roll command (ill-conditioning guard).
        # Timestamps, not a count: the guard matters most near closest
        # approach, and only the times can say whether it fired there.
        self._held_t_s: list[float] = []

    def observe_geometry(
        self, range_m: float, offset_ned_m: Sequence[float]
    ) -> None:
        self.final_approach.observe_geometry(range_m, offset_ned_m)

    def held(self, t_s: float) -> None:
        self._held_t_s.append(t_s)

    def observe_command(self, commanded_deg: float, estimated_deg: float,
                        truth_deg: float, rate: LateralRate) -> None:
        self.commanded.observe(commanded_deg)
        self.actual.observe(estimated_deg)
        self.truth.observe(truth_deg)
        self._tracking_errors.append(abs(commanded_deg - estimated_deg))
        self.final_approach.observe_command(commanded_deg, estimated_deg, rate)

    def summary(self, at_t_s: float) -> dict:
        """The report section, keyed as the artifacts already expect."""
        return {
            "lateral_held_commands": len(self._held_t_s),
            "lateral_held_final_3s": sum(
                1 for t in self._held_t_s if t >= at_t_s - 3.0),
            # Kept at the top level under its original name: every artifact and
            # every comparison in this investigation is keyed on it.
            "roll_reversals": self.commanded.reversals,
            "roll_commanded": self.commanded.summary(),
            "roll_actual": self.actual.summary(),
            "roll_truth": self.truth.summary(),
            "roll_tracking_error": _spread(self._tracking_errors),
            # Straightness of the final-approach path and steadiness of the command
            # that flew it -- neither is visible in a miss.
            "final_approach_geometry": self.final_approach.summary(),
        }


__all__ = ["LateralRate", "RollRecord", "FinalApproachGeometry"]
