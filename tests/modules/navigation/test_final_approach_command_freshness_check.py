"""A failed command hold names its bound with the values sampled in the check.

Ready plan step 4 (todos 148, 168): the decision must not change, each clock
is read once per check, and missing, invalid and backward clocks are named.
"""

from __future__ import annotations

import itertools
import math

import pytest

from navpy.core.approach.calc_data import CalcData
from navpy.core.approach.vision_nav.command_freshness import (
    FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
    FinalApproachCommandFreshness,
    FinalApproachCommandTiming,
)
from navpy.core.approach.vision_nav.command_hold import (
    FinalApproachCommandHold,
)

RECEIPT_LIMIT_S = 0.1


def _old_age_is_fresh(timestamp_value, now_provider, maximum_age_s):
    """The decision as it stood before step 4, kept verbatim as the oracle."""
    if timestamp_value is None or now_provider is None:
        return False
    try:
        raw_now_s = now_provider()
    except Exception:  # noqa: BLE001
        return False
    if isinstance(timestamp_value, bool) or isinstance(raw_now_s, bool):
        return False
    try:
        timestamp_s = float(timestamp_value)
        now_s = float(raw_now_s)
        limit_s = float(maximum_age_s)
    except (TypeError, ValueError):
        return False
    age_s = now_s - timestamp_s
    return (
        math.isfinite(timestamp_s)
        and math.isfinite(now_s)
        and math.isfinite(age_s)
        and math.isfinite(limit_s)
        and limit_s > 0.0
        and 0.0 <= age_s <= limit_s
    )


def _raises():
    raise OSError("clock gone")


_STAMPS = [None, 10.0, True, "x", math.nan, math.inf, -1e308]
_NOWS = [
    None,
    lambda: 10.0,  # age 0: fresh
    lambda: 11.0,  # age equals the source limit: fresh
    lambda: 10.05,
    lambda: 9.0,
    lambda: 12.0,
    lambda: True,
    lambda: "x",
    lambda: math.nan,
    lambda: 1e308,
    _raises,
]
_PERIODS = [
    lambda period_s: RECEIPT_LIMIT_S,
    lambda period_s: 0.0,
    lambda period_s: math.nan,
    lambda period_s: _raises(),
]


def _timing(source_stamp, source_now, receipt_stamp, receipt_now):
    return FinalApproachCommandTiming(
        source_stamp, source_now, receipt_stamp, receipt_now,
    )


@pytest.mark.parametrize("period", _PERIODS)
def test_decision_matches_the_previous_rule_for_every_input(period):
    freshness = FinalApproachCommandFreshness(period)
    for source_stamp, source_now, receipt_stamp, receipt_now in itertools.product(
        _STAMPS, _NOWS, _STAMPS, _NOWS,
    ):
        timing = _timing(source_stamp, source_now, receipt_stamp, receipt_now)
        try:
            receipt_limit = float(period(FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S))
        except Exception:  # noqa: BLE001
            receipt_limit = math.nan
        if not receipt_limit > 0.0:
            receipt_limit = math.nan
        expected = _old_age_is_fresh(
            source_stamp, source_now, FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
        ) and _old_age_is_fresh(receipt_stamp, receipt_now, receipt_limit)
        check = freshness.check(timing)
        assert check.fresh is expected, (timing, check)
        assert freshness.is_fresh(timing) is expected


class _CountingClock:
    def __init__(self, value):
        self.value = value
        self.reads = 0

    def __call__(self):
        self.reads += 1
        return self.value


def test_each_clock_is_read_once_and_the_logged_values_are_those_reads():
    source = _CountingClock(10.06)
    receipt = _CountingClock(5.1326)
    check = FinalApproachCommandFreshness(lambda period_s: RECEIPT_LIMIT_S).check(
        _timing(10.0, source, 5.0, receipt)
    )

    assert (source.reads, receipt.reads) == (1, 1)
    assert not check.fresh
    assert check.source.fresh
    assert check.receipt.problem == "too_old"
    assert check.receipt.now == 5.1326
    assert check.receipt.age_s == pytest.approx(0.1326)
    assert check.receipt.limit_s == RECEIPT_LIMIT_S
    text = check.describe()
    assert "receipt: too_old age_s=0.1326 limit_s=0.1000" in text
    assert "now=5.1326" in text


def test_receipt_clock_is_not_read_after_the_source_bound_failed():
    receipt = _CountingClock(5.0)
    check = FinalApproachCommandFreshness(lambda period_s: RECEIPT_LIMIT_S).check(
        _timing(10.0, lambda: 12.0, 5.0, receipt)
    )

    assert receipt.reads == 0
    assert check.source.problem == "too_old"
    assert check.receipt is None
    assert "receipt: not checked" in check.describe()


@pytest.mark.parametrize(
    ("stamp", "now", "problem"),
    [
        (None, lambda: 10.0, "missing_timestamp"),
        (10.0, None, "missing_clock"),
        (10.0, _raises, "clock_error:OSError"),
        (True, lambda: 10.0, "invalid_value"),
        (10.0, lambda: "x", "invalid_value"),
        (10.0, lambda: math.nan, "non_finite_clock"),
        (math.inf, lambda: 10.0, "non_finite_clock"),
        (-1e308, lambda: 1e308, "non_finite_age"),
        (10.0, lambda: 9.5, "backward_clock"),
        (10.0, lambda: 11.5, "too_old"),
    ],
)
def test_each_bad_clock_and_failed_bound_is_named(stamp, now, problem):
    check = FinalApproachCommandFreshness(lambda period_s: RECEIPT_LIMIT_S).check(
        _timing(stamp, now, 0.0, lambda: 0.0)
    )

    assert check.source.name == "source"
    assert check.source.problem == problem
    assert not check.fresh


@pytest.mark.parametrize("period", [lambda p: 0.0, lambda p: _raises()])
def test_an_invalid_receipt_limit_is_named(period):
    check = FinalApproachCommandFreshness(period).check(
        _timing(10.0, lambda: 10.0, 5.0, lambda: 5.0)
    )

    assert check.receipt.problem == "invalid_limit"
    assert math.isnan(check.receipt.limit_s)


class _Actuator:
    def __init__(self):
        self.calls = 0

    def issue(self, roll, pitch, throttle):
        self.calls += 1


class _Failures:
    def __init__(self):
        self.calls = 0

    def mark_failed(self):
        self.calls += 1


def _hold(report):
    actuator, failures = _Actuator(), _Failures()
    hold = FinalApproachCommandHold(
        actuator,
        source_time=None,
        failures=failures,
        freshness=FinalApproachCommandFreshness(lambda period_s: RECEIPT_LIMIT_S),
        report_stale=report,
    )
    return hold, actuator, failures


def _remember(hold, receipt_now):
    hold.remember(
        CalcData(0.0, 0.0, 1.0, 2.0, 0.5),
        timing=_timing(10.0, lambda: 10.06, 5.0, receipt_now),
    )


def test_a_failed_hold_reports_the_check_once_and_still_fails():
    reports = []
    hold, actuator, failures = _hold(reports.append)
    receipt = _CountingClock(5.05)
    _remember(hold, receipt)

    receipt.value = 5.1326
    assert hold.issue() is None
    assert hold.issue() is None  # the command is gone: no second report

    assert len(reports) == 1
    assert reports[0].receipt.problem == "too_old"
    assert reports[0].receipt.now == 5.1326
    assert receipt.reads == 1
    assert (actuator.calls, failures.calls) == (0, 1)
    assert not hold.available


def test_a_failing_report_does_not_change_the_decision():
    def report(_check):
        raise RuntimeError("logger broken")

    hold, actuator, failures = _hold(report)
    _remember(hold, lambda: 5.2)

    assert hold.issue() is None
    assert (actuator.calls, failures.calls) == (0, 1)
    assert not hold.available


def test_a_fresh_hold_reports_nothing():
    reports = []
    hold, actuator, failures = _hold(reports.append)
    source = _CountingClock(10.06)
    hold.remember(
        CalcData(0.0, 0.0, 1.0, 2.0, 0.5),
        timing=_timing(10.0, source, 5.0, lambda: 5.05),
    )
    source.value = 10.6  # a refresh period of aircraft time later

    assert hold.issue() is not None
    assert reports == []
    assert (actuator.calls, failures.calls) == (1, 0)
