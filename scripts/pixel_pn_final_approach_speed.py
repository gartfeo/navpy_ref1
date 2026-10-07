"""Drop SITL from cruise speed to final-approach speed before the scored leg.

Cruising out to the gate is dead time: nothing is scored until the companion
starts navigation, so flying it at 1x costs minutes per case for no evidence. The scored
leg is the opposite -- `eval_observation_freshness` shows a fast clock starves
the law of fresh frames, and the held-command reissue hides it, so the final approach
leg has to run at the speed the run claims to measure.

So the run does both: launch fast, then step down to the final-approach speed while
the aircraft is still on the gate leg, before navigation is activated.

The step down is not trusted on the parameter echo alone. The echo only proves
the autopilot STORED the value; only the clock proves it ACTED on it. A scored
leg accidentally flown at cruise speed looks like an ordinary run right up
until the freshness gate rejects it, minutes later -- so this measures the
autopilot's own boot clock against the wall clock and refuses to continue if
they disagree.

Tolerances and the parameter name come from `swarm_run_verification_model`,
which is where the launcher's own clock gate already keeps them.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Importable as `pixel_pn_final_approach_speed` (how the harnesses run) and as
# `scripts.pixel_pn_final_approach_speed` (how the tests import it). Only the first
# puts this directory on the path, and the sibling import below needs it.
SCRIPTS = str(Path(__file__).resolve().parent)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import swarm_run_verification_model as model  # noqa: E402

# Long enough for two ATTITUDE messages to arrive even on a slow stream; the
# measurement itself only needs `model.CLOCK_MIN_SPAN_S` of wall time.
CLOCK_MEASURE_TIMEOUT_S = 10.0
CLOCK_POLL_S = 0.02
# The autopilot does not change its clock rate the instant it stores the
# parameter, and a window opened immediately straddles the change: stepping
# 20x -> 1x, a straddling window reads somewhere BETWEEN the two speeds, so an
# aircraft that settled perfectly well is rejected for a rate it was only
# passing through. Measured on a nine-aircraft fleet: six read 0.999-1.080 and
# three read 1.280-1.560, all running the same step at the same moment.
#
# So re-measure until the clock matches, rather than trusting one window. The
# bound keeps a clock that never follows a failure, and is small next to the
# gate leg this settles in -- that leg runs at final-approach speed and lasts tens of
# seconds, against the few windows spent here.
SETTLE_TIMEOUT_S = 6.0


def launch_speedup(cruise_speedup: float, scored_speedup: float) -> float:
    """The speed SITL boots at: the cruise speed only when it is the faster one."""
    cruise = float(cruise_speedup or 0.0)
    return cruise if cruise > scored_speedup else float(scored_speedup)


def step_down_ordinal(scoring_start_wp: int) -> int:
    """The NAV_WAYPOINT ordinal to step the speed down on: the final-approach start gate.

    Stepping down AT scoring interval would be too late to be free. At 20x the
    aircraft covers several hundred metres per wall second, so the seconds
    spent settling the clock would come out of the ~900 m scored leg. The
    ordinal before the scoring interval one is the gate, which leaves the whole gate
    leg to settle in and costs cruise distance only.
    """
    if scoring_start_wp < 2:
        raise ValueError(
            "a cruise speed needs a NAV_WAYPOINT before the scoring-start one to "
            f"step down on; the scoring-start ordinal is {scoring_start_wp}"
        )
    return scoring_start_wp - 1


def boot_time_s(vehicle: Any) -> float | None:
    """The autopilot's own clock, as carried on ATTITUDE."""
    sample = getattr(vehicle, "attitude_sample", None)
    if sample is None:
        return None
    value = getattr(sample, "time_boot_s", None)
    return None if value is None else float(value)


def measure_clock_rate(
    vehicle: Any,
    *,
    span_s: float = model.CLOCK_MIN_SPAN_S,
    timeout_s: float = CLOCK_MEASURE_TIMEOUT_S,
    poll_s: float = CLOCK_POLL_S,
) -> float | None:
    """Sim seconds per wall second, or None if the clock never advanced."""
    deadline = time.monotonic() + timeout_s
    first: float | None = None
    while first is None and time.monotonic() < deadline:
        first = boot_time_s(vehicle)
        if first is None:
            time.sleep(poll_s)
    if first is None:
        return None
    started = time.monotonic()
    while time.monotonic() < deadline:
        time.sleep(poll_s)
        last = boot_time_s(vehicle)
        wall_s = time.monotonic() - started
        # `last > first` matters as much as the span: a stalled stream would
        # otherwise report a rate of exactly zero, which reads as a measured
        # result rather than as no measurement at all.
        if last is not None and last > first and wall_s >= span_s:
            return (last - first) / wall_s
    return None


@dataclass(frozen=True)
class FinalApproachSpeedPlan:
    """Where the flight steps down to the scored speed, and to what.

    The default is inert on purpose: an evaluator that does not ask for a
    cruise speed produces a plan that adds no child arguments at all, so the
    flight is byte-for-byte the one every earlier run flew.
    """

    del_speedup: float = 0.0
    slow_seq: int = 0

    @property
    def active(self) -> bool:
        return self.del_speedup > 0.0 and self.slow_seq > 0

    def child_args(self) -> list[str]:
        if not self.active:
            return []
        return [
            "--del-speedup",
            repr(float(self.del_speedup)),
            "--slow-seq",
            str(int(self.slow_seq)),
        ]


@dataclass(frozen=True)
class SpeedChange:
    """What was asked of the autopilot, and what its clock actually did."""

    requested: float
    accepted: bool
    measured_rate: float | None

    @property
    def settled(self) -> bool:
        return (
            self.accepted
            and self.measured_rate is not None
            and model.rate_matches(self.measured_rate, self.requested)
        )

    def describe(self) -> str:
        if not self.accepted:
            return (
                f"{model.SPEEDUP_PARAM}={self.requested:g} was not "
                "acknowledged by the autopilot"
            )
        if self.measured_rate is None:
            return (
                f"{model.SPEEDUP_PARAM}={self.requested:g} was acknowledged "
                "but the clock never advanced, so it could not be verified"
            )
        return (
            f"{model.SPEEDUP_PARAM}={self.requested:g} measured "
            f"{self.measured_rate:.3f} sim s per wall s"
        )


def final_approach_speed_plan(
    args: Any,
    speedup: float,
    mission: Any,
    home_abs_alt_m: float,
) -> FinalApproachSpeedPlan:
    """Resolve the gate ordinal to the mission sequence the child waits on.

    The mission import is function-local on purpose: the flight child imports
    this module for `settle_final_approach_speed` alone and has no business loading
    the evaluator's mission machinery to do it.
    """
    from eval_navigation_mission import resolve_poi_expectation

    if launch_speedup(args.cruise_speedup, speedup) <= speedup:
        return FinalApproachSpeedPlan()
    gate = resolve_poi_expectation(
        mission,
        poi_wp=step_down_ordinal(args.scoring_start_wp),
        poi_rel_alt_m=args.poi_alt,
        home_abs_alt_m=home_abs_alt_m,
    )
    return FinalApproachSpeedPlan(del_speedup=speedup, slow_seq=gate.mission_seq)


def settle_final_approach_speed(
    vehicle: Any,
    speedup: float,
    *,
    span_s: float = model.CLOCK_MIN_SPAN_S,
    timeout_s: float = SETTLE_TIMEOUT_S,
) -> SpeedChange:
    """Set the final-approach speed and wait for the clock to follow it.

    The tolerance is unchanged and so is the verdict: a clock that never
    reaches the requested rate is still reported unsettled. Waiting only
    separates a clock still crossing to the new speed from one that is not
    going there at all, which a single window cannot tell apart.
    """
    accepted = bool(vehicle.set_parameter(model.SPEEDUP_PARAM, float(speedup)))
    if not accepted:
        return SpeedChange(float(speedup), False, None)
    requested = float(speedup)
    rate = measure_clock_rate(vehicle, span_s=span_s)
    deadline = time.monotonic() + timeout_s
    while not _followed(rate, requested) and time.monotonic() < deadline:
        rate = measure_clock_rate(vehicle, span_s=span_s)
    return SpeedChange(requested, True, rate)


def _followed(rate: float | None, requested: float) -> bool:
    return rate is not None and model.rate_matches(rate, requested)


__all__ = [
    "CLOCK_MEASURE_TIMEOUT_S",
    "CLOCK_POLL_S",
    "SETTLE_TIMEOUT_S",
    "SpeedChange",
    "FinalApproachSpeedPlan",
    "boot_time_s",
    "launch_speedup",
    "measure_clock_rate",
    "settle_final_approach_speed",
    "step_down_ordinal",
    "final_approach_speed_plan",
]
