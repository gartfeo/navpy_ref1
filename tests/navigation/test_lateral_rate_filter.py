"""The lateral rate must low-pass like the vertical one, or roll dithers.

The roll command is proportional to this rate (`atan(N*V*rate/g)`), so an
unfiltered two-point bearing derivative reached the bank whole, times ~10.
These tests pin the fix: a tau attenuates alternating-sign bearing noise, and
tau=None keeps the old raw passthrough so nothing silently changed for callers
that do not opt in.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.lateral_rate import LateralRateFilter


def _frame(timestamp_s: float, bearing_rad: float) -> TerminalVisionFrame:
    # Unit control ray in the horizontal plane at the requested bearing; body
    # ray is any fixed unit vector (the filter reads only the control bearing).
    return TerminalVisionFrame(
        source_name="test",
        source_generation=0,
        task_id=1,
        obj_id=1,
        source_timestamp_s=timestamp_s,
        body_x=1.0,
        body_y=0.0,
        body_z=0.0,
        control_x=math.cos(bearing_rad),
        control_y=math.sin(bearing_rad),
        control_z=0.0,
    )


def _run(tau_s, bearings, dt_s=0.125):
    the_filter = LateralRateFilter()
    the_filter.seed(_frame(0.0, bearings[0]))
    rates = []
    for index, bearing in enumerate(bearings[1:], start=1):
        plan = the_filter.plan(_frame(index * dt_s, bearing), tau_s)
        the_filter.commit(plan)
        rates.append((plan.raw_inertial_rate_rad_s, plan.rate_rad_s))
    return rates


# Alternating +/- lateral wobble around dead-ahead: the classic dither, zero
# mean, that PN was amplifying into the bank.
_NOISE = [0.0, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02]


def test_none_is_the_old_raw_passthrough() -> None:
    for raw, delivered in _run(None, _NOISE):
        assert delivered == raw


def _swing(rates):
    return max(f for _, f in rates) - min(f for _, f in rates)


def test_tau_attenuates_the_alternating_noise() -> None:
    filtered = _run(0.10, _NOISE)
    raw_swing = max(r for r, _ in filtered) - min(r for r, _ in filtered)
    # The low-pass must shrink the peak-to-peak of the dither it is there to
    # remove; how much is a tuning choice measured in SITL, not pinned here.
    assert _swing(filtered) < raw_swing


def test_more_tau_attenuates_more() -> None:
    # Monotonicity is the real contract: a longer time constant is a heavier
    # low-pass, so the delivered dither must shrink as tau grows.
    assert _swing(_run(0.30, _NOISE)) < _swing(_run(0.10, _NOISE))


def test_a_steady_rate_survives_the_filter() -> None:
    # A real constant LOS rate (steadily increasing bearing) must pass through:
    # the filter removes jitter, not signal. After settling, filtered tracks
    # raw within a few percent.
    steady = [i * 0.03 for i in range(12)]
    filtered = _run(0.10, steady)
    raw, delivered = filtered[-1]
    assert abs(delivered - raw) < 0.05 * abs(raw)


# The filter is NOT only a noise filter, and the tests above cannot see that.
# A CONSTANT rate is the one profile a low-pass tracks perfectly, so
# `test_a_steady_rate_survives_the_filter` is blind to the filter's other
# effect: it lags a rate that is still CHANGING. Crosswind makes the rate
# change (the aircraft crabs), which is why the point-mass bench shows
# crosswind roll commands differing by up to 2.03 deg between tau=None and
# tau=0.30 with ZERO injected noise. These pin the mechanism so the filter is
# not read as noise-only again, and so a tau change has to face its cost.
_DT_S = 0.125
_CONSTANT_RATE = [0.03 * index for index in range(14)]
# Bearing quadratic in time, so the LOS rate ramps. Noise-free: pure signal.
_CHANGING_RATE = [0.015 * (index * _DT_S) ** 2 / _DT_S for index in range(14)]


def _lag_fraction(tau_s, bearings) -> float:
    raw, delivered = _run(tau_s, bearings)[-1]
    return (raw - delivered) / raw


def test_tau_none_never_lags_even_a_changing_rate() -> None:
    assert _lag_fraction(None, _CHANGING_RATE) == 0.0


def test_the_filter_lags_a_changing_rate_far_more_than_a_constant_one() -> None:
    constant = _lag_fraction(0.30, _CONSTANT_RATE)
    changing = _lag_fraction(0.30, _CHANGING_RATE)
    # Measured 0.44% and 15.4% at this dt; the contract is the ORDER of
    # magnitude between them, not the digits.
    assert constant < 0.02
    assert changing > 0.10
    assert changing > 10.0 * constant


def test_a_longer_tau_lags_a_changing_rate_more() -> None:
    # The cost side of the tuning knob: whatever tau buys in dither rejection
    # it pays for here. Monotone, so neither direction is a free choice.
    assert _lag_fraction(0.30, _CHANGING_RATE) > _lag_fraction(0.10, _CHANGING_RATE)
    assert _lag_fraction(0.10, _CHANGING_RATE) > _lag_fraction(None, _CHANGING_RATE)


# --- AAS_LAT_GYRO_TERM arm gate (exp/lat-gyro-endpoint) ---


def _yaw_frame(timestamp_s, bearing_rad, yaw_rate_rad_s):
    return dataclasses.replace(
        _frame(timestamp_s, bearing_rad),
        aircraft_yaw_rate_rad_s=yaw_rate_rad_s,
    )


def test_default_gyro_term_is_the_midpoint_blend(monkeypatch) -> None:
    # Default flipped to blend after the 2026-08-27 A/B chain; d/T =
    # 0.01125/0.125 = 0.09 -> weights 0.41 / 0.59.
    monkeypatch.delenv("AAS_LAT_GYRO_TERM", raising=False)
    monkeypatch.delenv("AAS_LAT_GYRO_DELAY_S", raising=False)
    the_filter = LateralRateFilter()
    the_filter.seed(_yaw_frame(0.0, 0.0, 0.10))
    plan = the_filter.plan(_yaw_frame(0.125, 0.0, 0.30), None)
    assert plan.raw_inertial_rate_rad_s == pytest.approx(
        0.41 * 0.10 + 0.59 * 0.30
    )


def test_avg_gyro_term_is_the_trapezoid_average(monkeypatch) -> None:
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "avg")
    the_filter = LateralRateFilter()
    the_filter.seed(_yaw_frame(0.0, 0.0, 0.10))
    plan = the_filter.plan(_yaw_frame(0.125, 0.0, 0.30), None)
    assert plan.raw_inertial_rate_rad_s == pytest.approx(0.20)


def test_endpoint_gyro_term_uses_only_the_newest_sample(monkeypatch) -> None:
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "endpoint")
    the_filter = LateralRateFilter()
    the_filter.seed(_yaw_frame(0.0, 0.0, 0.10))
    plan = the_filter.plan(_yaw_frame(0.125, 0.0, 0.30), None)
    assert plan.raw_inertial_rate_rad_s == pytest.approx(0.30)


def test_unknown_gyro_term_fails_loud(monkeypatch) -> None:
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "midpoint")
    with pytest.raises(ValueError):
        LateralRateFilter()


def test_blend_aims_the_two_samples_at_the_frame_midpoint(monkeypatch) -> None:
    # d/T = 0.005/0.125 = 0.04 -> weights 0.46 / 0.54.
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "blend")
    monkeypatch.setenv("AAS_LAT_GYRO_DELAY_S", "0.005")
    the_filter = LateralRateFilter()
    the_filter.seed(_yaw_frame(0.0, 0.0, 0.10))
    plan = the_filter.plan(_yaw_frame(0.125, 0.0, 0.30), None)
    assert plan.raw_inertial_rate_rad_s == pytest.approx(
        0.46 * 0.10 + 0.54 * 0.30
    )


def test_blend_equals_linear_midpoint_correction_at_bench_cadence(
    monkeypatch,
) -> None:
    # Default delay 11.25 ms at a 27 ms frame: ratio 0.4167 -> weights
    # 0.0833 / 0.9167 - nearly the endpoint arm, by construction.
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "blend")
    monkeypatch.delenv("AAS_LAT_GYRO_DELAY_S", raising=False)
    the_filter = LateralRateFilter()
    the_filter.seed(_yaw_frame(0.0, 0.0, 0.10))
    plan = the_filter.plan(_yaw_frame(0.027, 0.0, 0.30), None)
    ratio = 0.01125 / 0.027
    assert plan.raw_inertial_rate_rad_s == pytest.approx(
        (0.5 - ratio) * 0.10 + (0.5 + ratio) * 0.30
    )


def test_blend_extrapolation_is_capped_at_one_sample_step(monkeypatch) -> None:
    # Frame interval shorter than the delay: ratio clamps at 1.0 -> weights
    # -0.5 / 1.5, never further.
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "blend")
    monkeypatch.setenv("AAS_LAT_GYRO_DELAY_S", "0.05")
    the_filter = LateralRateFilter()
    the_filter.seed(_yaw_frame(0.0, 0.0, 0.10))
    plan = the_filter.plan(_yaw_frame(0.008, 0.0, 0.30), None)
    assert plan.raw_inertial_rate_rad_s == pytest.approx(
        -0.5 * 0.10 + 1.5 * 0.30
    )


def test_bad_gyro_delay_fails_loud(monkeypatch) -> None:
    monkeypatch.setenv("AAS_LAT_GYRO_TERM", "blend")
    monkeypatch.setenv("AAS_LAT_GYRO_DELAY_S", "0.5")
    with pytest.raises(ValueError):
        LateralRateFilter()
    monkeypatch.setenv("AAS_LAT_GYRO_DELAY_S", "banana")
    with pytest.raises(ValueError):
        LateralRateFilter()
