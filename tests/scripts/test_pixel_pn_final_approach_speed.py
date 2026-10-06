"""Cruising fast is only safe if the step back down is PROVEN, not requested.

The whole feature trades wall time for the risk that the scored leg is flown at
the cruise clock. A parameter echo does not retire that risk -- it says the
autopilot stored the value, not that it acted on it -- so the load-bearing
property here is that a clock which keeps running fast is rejected even though
the autopilot said yes.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from scripts.pixel_pn_final_approach_speed import (
    SpeedChange,
    FinalApproachSpeedPlan,
    launch_speedup,
    measure_clock_rate,
    settle_final_approach_speed,
    step_down_ordinal,
)

# Short enough to keep the suite quick, long enough to span several polls.
SPAN_S = 0.05
TIMEOUT_S = 2.0


class FakeVehicle:
    """A vehicle whose clock runs at a rate the test chooses."""

    def __init__(self, rate: float, *, accepts: bool = True) -> None:
        self._rate = rate
        self._accepts = accepts
        self._start = time.monotonic()
        self.parameter_sets: list[tuple[str, float]] = []

    def set_parameter(self, name: str, value: float, **_: object) -> bool:
        self.parameter_sets.append((name, value))
        return self._accepts

    @property
    def attitude_sample(self) -> SimpleNamespace:
        elapsed = time.monotonic() - self._start
        return SimpleNamespace(time_boot_s=100.0 + elapsed * self._rate)


class StalledVehicle(FakeVehicle):
    """Telemetry still arrives, but the clock behind it never moves."""

    @property
    def attitude_sample(self) -> SimpleNamespace:
        return SimpleNamespace(time_boot_s=100.0)


class SilentVehicle(FakeVehicle):
    """No ATTITUDE has arrived, so there is no clock to read at all."""

    @property
    def attitude_sample(self) -> None:
        return None


class CrossingVehicle(FakeVehicle):
    """Still at cruise speed when first read, at final-approach speed shortly after.

    Boot time is integrated rather than computed from the elapsed total, so the
    clock stays monotonic across the change instead of jumping backwards.
    """

    def __init__(
        self, *, cruise: float, final_approach: float, crossing_s: float
    ) -> None:
        super().__init__(final_approach)
        self._cruise = cruise
        self._final_approach = final_approach
        self._crossing_s = crossing_s
        self._boot_s = 100.0
        self._read_at = self._start

    @property
    def attitude_sample(self) -> SimpleNamespace:
        now = time.monotonic()
        rate = (
            self._cruise
            if now - self._start < self._crossing_s
            else self._final_approach
        )
        self._boot_s += (now - self._read_at) * rate
        self._read_at = now
        return SimpleNamespace(time_boot_s=self._boot_s)


@pytest.mark.parametrize(
    ("cruise", "scored", "expected"),
    [
        (0.0, 1.0, 1.0),
        (20.0, 1.0, 20.0),
        (1.0, 10.0, 10.0),
        (10.0, 10.0, 10.0),
    ],
)
def test_the_cruise_speed_is_used_only_when_it_is_the_faster_one(
    cruise: float, scored: float, expected: float
) -> None:
    """A cruise SLOWER than the scored speed would make the case worse, not faster."""
    assert launch_speedup(cruise, scored) == expected


def test_the_step_down_happens_one_waypoint_before_scoring_interval() -> None:
    assert step_down_ordinal(2) == 1


def test_there_has_to_be_a_waypoint_left_to_step_down_on() -> None:
    """Ordinal 1 has nothing before it, so there is no gate leg to settle in."""
    with pytest.raises(ValueError, match="before the scoring-start one"):
        step_down_ordinal(1)


def test_a_plan_nobody_asked_for_changes_the_child_command_not_at_all() -> None:
    """The default has to reproduce every earlier run byte for byte."""
    assert FinalApproachSpeedPlan().active is False
    assert FinalApproachSpeedPlan().child_args() == []


def test_an_active_plan_names_both_the_speed_and_where_to_take_it() -> None:
    plan = FinalApproachSpeedPlan(del_speedup=1.0, slow_seq=3)
    assert plan.active is True
    assert plan.child_args() == ["--del-speedup", "1.0", "--slow-seq", "3"]


@pytest.mark.parametrize("rate", [1.0, 5.0])
def test_the_measured_rate_is_the_clock_not_the_request(rate: float) -> None:
    measured = measure_clock_rate(
        FakeVehicle(rate), span_s=SPAN_S, timeout_s=TIMEOUT_S
    )
    assert measured == pytest.approx(rate, rel=0.25)


def test_a_stalled_clock_reports_no_measurement_rather_than_a_rate_of_zero() -> None:
    """Zero would read as a measured result; None says the evidence is missing."""
    assert measure_clock_rate(
        StalledVehicle(0.0), span_s=SPAN_S, timeout_s=0.3
    ) is None


def test_a_vehicle_with_no_attitude_yet_reports_no_measurement() -> None:
    assert measure_clock_rate(
        SilentVehicle(1.0), span_s=SPAN_S, timeout_s=0.3
    ) is None


def test_a_clock_still_running_at_cruise_speed_does_not_count_as_settled() -> None:
    """The load-bearing one.

    The autopilot ACCEPTS the parameter here -- `accepts=True` -- and still
    runs at 20x. If this passed, the scored leg would be flown at the cruise
    clock and would look like an ordinary run until the freshness gate
    rejected it minutes later.
    """
    vehicle = FakeVehicle(20.0)

    # timeout_s only keeps the suite quick: this clock never reaches 1x, so
    # waiting longer changes how long the refusal takes, not that it refuses.
    change = settle_final_approach_speed(vehicle, 1.0, span_s=SPAN_S, timeout_s=0.3)

    assert vehicle.parameter_sets == [("SIM_SPEEDUP", 1.0)]
    assert change.accepted is True
    assert change.settled is False
    assert "measured" in change.describe()


def test_a_clock_that_followed_the_request_counts_as_settled() -> None:
    change = settle_final_approach_speed(FakeVehicle(1.0), 1.0, span_s=SPAN_S)

    assert change.settled is True


def test_a_clock_still_crossing_to_the_new_speed_is_waited_for() -> None:
    """The step is not instant, and one window opened on top of it straddles it.

    Measured on a nine-aircraft fleet stepping 20x -> 1x together: six read
    0.999-1.080 and three read 1.280-1.560 -- between the two speeds, which is
    what a straddling window reads. Rejecting on the first window threw away a
    third of the fleet for a rate it was only passing through.
    """
    vehicle = CrossingVehicle(cruise=20.0, final_approach=1.0, crossing_s=SPAN_S * 3)

    change = settle_final_approach_speed(
        vehicle, 1.0, span_s=SPAN_S, timeout_s=TIMEOUT_S
    )

    assert change.settled is True
    assert change.measured_rate == pytest.approx(1.0, rel=0.25)


def test_waiting_never_turns_a_clock_that_stays_fast_into_a_settled_one() -> None:
    """The wait must not become a way to pass: same tolerance, same verdict."""
    change = settle_final_approach_speed(
        FakeVehicle(20.0), 1.0, span_s=SPAN_S, timeout_s=0.3
    )

    assert change.settled is False


def test_a_refused_parameter_is_never_reported_as_settled() -> None:
    vehicle = FakeVehicle(1.0, accepts=False)

    change = settle_final_approach_speed(vehicle, 1.0, span_s=SPAN_S)

    assert change.settled is False
    assert change.measured_rate is None
    assert "not acknowledged" in change.describe()


def test_an_unverifiable_clock_says_so_instead_of_claiming_a_rate() -> None:
    change = SpeedChange(requested=1.0, accepted=True, measured_rate=None)

    assert change.settled is False
    assert "could not be verified" in change.describe()
