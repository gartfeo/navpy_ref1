"""Mechanism tests for the ArduPlane 4.6 controller port (tools/tuning/ap_controller.py, ap_filter.py).

The port is checked against SITL logs by tools/tuning/port_check.py; these tests pin the
individual C++ behaviours that a replay with quiet data does not exercise.
"""
import numpy as np
import pytest

from tools.tuning.ap_controller import AcPid, Gains, RateController, speed_scaler
from tools.tuning.ap_filter import SlewLimiter, lowpass_alpha

DT = 0.005


def gains(n=1, **kw):
    base = dict(P=0.1, I=0.1, D=0.0, FF=0.3, IMAX=0.666, FLTT=0.0, FLTE=0.0, FLTD=0.0, SMAX=0.0,
                TCONST=0.5, RMAX_POS=75.0, RMAX_NEG=75.0)
    base.update(kw)
    return Gains(**{k: np.full(n, float(v)) for k, v in base.items()})


def test_lowpass_alpha_matches_calc_lowpass_alpha_dt():
    assert lowpass_alpha(DT, 0.0) == 1.0                       # cutoff 0 disables the filter
    rc = 1 / (2 * np.pi * 10.0)
    assert lowpass_alpha(DT, 10.0) == pytest.approx(DT / (DT + rc))


def test_integrator_clamps_to_imax_and_only_unwinds_when_limited():
    pid = AcPid(gains(I=100.0, P=0.0), 1)
    for k in range(200):
        pid.update_all(1.0, 0.0, DT, np.array([False]), k * 5)
    assert pid.integrator[0] == pytest.approx(0.666)
    # with limit set, a positive error may not grow it; a negative error may shrink it
    pid.update_all(1.0, 0.0, DT, np.array([True]), 1000)
    assert pid.integrator[0] == pytest.approx(0.666)
    pid.update_all(-1.0, 0.0, DT, np.array([True]), 1005)
    assert pid.integrator[0] < 0.666


def test_slew_limiter_reduces_gain_only_for_sustained_fast_alternation():
    quiet = SlewLimiter(1, smax=150.0)
    for k in range(400):
        mod = quiet.modifier(np.array([0.01 * np.sin(k * DT)]), DT, k * 5)
    assert mod[0] == pytest.approx(1.0)
    shaking = SlewLimiter(1, smax=150.0)
    for k in range(400):
        mod = shaking.modifier(np.array([20.0 * np.sin(2 * np.pi * 8 * k * DT)]), DT, k * 5)
    assert mod[0] < 0.9
    disabled = SlewLimiter(1, smax=0.0)
    assert disabled.modifier(np.array([100.0]), DT, 0)[0] == 1.0


def test_roll_output_feedforward_scales_with_scaler_over_eas2tas_and_clips():
    ctl = RateController("roll", gains(P=0.0, I=0.0), 1, airspeed_min=10.0, airspeed_max=30.0)
    out = ctl.rate_out(np.array([30.0]), np.array([0.0]), 1.5, 1.2, np.array([20.0]), DT, 0)
    assert ctl.info.FF[0] == pytest.approx(30.0 * 0.3 * 1.5 / 1.2, rel=1e-6)
    assert out[0] == pytest.approx(ctl.info.out[0])
    big = RateController("roll", gains(FF=5.0, P=0.0, I=0.0), 1, 10.0, 30.0)
    assert big.rate_out(np.array([100.0]), np.array([0.0]), 1.0, 1.0, np.array([20.0]), DT, 0)[0] == 45.0


def test_roll_underspeed_locks_integrator_but_output_uses_updated_value():
    ctl = RateController("roll", gains(P=0.0, FF=0.0, I=5.0), 1, airspeed_min=15.0, airspeed_max=30.0)
    ctl.pid.integrator[:] = 0.2
    ctl.rate_out(np.array([-50.0]), np.array([0.0]), 1.0, 1.0, np.array([12.0]), DT, 0)
    # limit_I is set when underspeed, so a shrinking error may update _pid_info.I ...
    assert ctl.info.I[0] < 0.2 * 180 / np.pi
    # ... but the integrator itself is put back to its old value (set_integrator(old_I))
    assert ctl.pid.integrator[0] == pytest.approx(0.2)


def test_pitch_underspeed_threshold_is_half_airspeed_min():
    ctl = RateController("pitch", gains(P=0.0, FF=0.0, I=5.0), 1, airspeed_min=15.0, airspeed_max=30.0)
    ctl.pid.integrator[:] = 0.2
    ctl.rate_out(np.array([-50.0]), np.array([0.0]), 1.0, 1.0, np.array([12.0]), DT, 0)
    assert ctl.pid.integrator[0] < 0.2                       # 12 m/s is not underspeed for pitch


def test_roll_angle_loop_rate_limit_and_tau_floor():
    ctl = RateController("roll", gains(TCONST=0.01, RMAX_POS=60.0), 1, 10.0, 30.0)
    assert ctl.desired_rate(np.array([2.0]))[0] == pytest.approx(2.0 / 0.05)   # tau floored at 0.05
    assert ctl.desired_rate(np.array([30.0]))[0] == 60.0


def test_pitch_coordination_adds_level_turn_rate():
    ctl = RateController("pitch", gains(RMAX_POS=0.0, RMAX_NEG=0.0), 1, 10.0, 30.0)
    bank = np.radians(30.0)
    rate = ctl.desired_rate(np.array([0.0]), roll_rad=bank, pitch_rad=0.0, aspeed=20.0, eas2tas=1.0)
    expected = np.degrees(9.80665 / 20.0 * np.tan(bank) * np.sin(bank))
    assert rate[0] == pytest.approx(expected, rel=1e-6)


def test_speed_scaler_limits():
    assert speed_scaler(24.0, 24.0, 20.0, 33.0) == pytest.approx(1.0)
    assert speed_scaler(0.0, 24.0, 20.0, 33.0) == pytest.approx(max(2.0, 24.0 / 14.0))
    assert speed_scaler(200.0, 24.0, 20.0, 33.0) == pytest.approx(min(0.5, 24.0 / 66.0))


def test_batch_rows_are_independent():
    g = gains(n=3)
    g.P = np.array([0.05, 0.1, 0.2])
    ctl = RateController("roll", g, 3, 10.0, 30.0)
    out = ctl.rate_out(np.array([10.0]), np.array([0.0]), 1.0, 1.0, np.array([20.0]), DT, 0)
    assert out[0] < out[1] < out[2]
