"""The child must refuse to start navigation on a clock it could not slow down.

The step-down is the only thing standing between a fast cruise and a scored
leg flown at cruise speed. If the child scored anyway, the run would produce
a normal-looking miss and only the freshness gate, minutes later, would say
anything was wrong -- and a marginal case might not even trip that.
"""

from __future__ import annotations

import argparse
import time
from types import SimpleNamespace

import pytest

from scripts.direct_pixel_pn_child import _step_to_terminal_speed


class Vehicle:
    """Reaches the step-down sequence at once; its clock runs at `rate`."""

    def __init__(self, rate: float) -> None:
        self._rate = rate
        self._start = time.monotonic()
        self.is_armed = True
        self.mission_items_next = 3
        self.parameter_sets: list[tuple[str, float]] = []

    def set_parameter(self, name: str, value: float, **_: object) -> bool:
        self.parameter_sets.append((name, value))
        return True

    @property
    def attitude_sample(self) -> SimpleNamespace:
        elapsed = time.monotonic() - self._start
        return SimpleNamespace(time_boot_s=100.0 + elapsed * self._rate)


def _options(**overrides: object) -> argparse.Namespace:
    values: dict[str, object] = {
        "del_speedup": 1.0,
        "slow_seq": 3,
        "timeout": 5.0,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


@pytest.mark.parametrize(
    "options",
    [_options(del_speedup=0.0), _options(slow_seq=0)],
    ids=["no terminal speed", "nowhere to step down"],
)
def test_a_run_that_asked_for_nothing_never_touches_the_clock(
    options: argparse.Namespace,
) -> None:
    """Every earlier run took this path, so it must not write a parameter."""
    vehicle = Vehicle(1.0)

    _step_to_terminal_speed(vehicle, options)

    assert vehicle.parameter_sets == []


def test_a_clock_that_slowed_lets_the_flight_continue() -> None:
    vehicle = Vehicle(1.0)

    _step_to_terminal_speed(vehicle, _options())

    assert vehicle.parameter_sets == [("SIM_SPEEDUP", 1.0)]


def test_a_clock_that_stayed_fast_stops_the_flight_before_scoring_interval() -> None:
    """The autopilot accepts the parameter and keeps running at 20x anyway."""
    vehicle = Vehicle(20.0)

    with pytest.raises(RuntimeError, match="terminal speed did not settle"):
        _step_to_terminal_speed(vehicle, _options())
