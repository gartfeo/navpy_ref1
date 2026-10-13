"""Tests for the elevon mapping, closed-loop identification and gain design (tools/tuning)."""
import numpy as np
import pytest

from tools.tuning import closed_loop, design
from tools.tuning.ap_controller import Gains
from tools.tuning.model_id import AxisData, ElevonMap, fit_axis, validate

PARAMS = {
    "AIRSPEED_MIN": 15.0, "AIRSPEED_MAX": 30.0, "SCALING_SPEED": 22.0, "SCHED_LOOP_RATE": 100.0,
    "RLL_RATE_P": 0.1, "RLL_RATE_I": 0.1, "RLL_RATE_D": 0.002, "RLL_RATE_FF": 0.3, "RLL_RATE_IMAX": 0.666,
    "RLL_RATE_FLTT": 3.18, "RLL_RATE_FLTE": 0.0, "RLL_RATE_FLTD": 10.0, "RLL_RATE_SMAX": 150.0,
    "RLL2SRV_TCONST": 0.5, "RLL2SRV_RMAX": 75.0, "MIXING_GAIN": 0.5,
    "SERVO1_FUNCTION": 77, "SERVO1_MIN": 1000, "SERVO1_TRIM": 1500, "SERVO1_MAX": 2000, "SERVO1_REVERSED": 0,
    "SERVO2_FUNCTION": 78, "SERVO2_MIN": 1000, "SERVO2_TRIM": 1500, "SERVO2_MAX": 2000, "SERVO2_REVERSED": 1,
}


def test_off_centre_trim_gives_different_travel_per_direction():
    # SERVO1 auto-trimmed to 1400 (the limit of auto-trim): up travel uses MAX-TRIM, down uses TRIM-MIN
    emap = ElevonMap(gain=0.5, left=(1000.0, 1400.0, 2000.0, False), right=(1000.0, 1500.0, 2000.0, True))
    up_l, _ = emap.travel(0.0, 4500.0)            # full pitch up: left elevon out = +2250
    down_l, _ = emap.travel(0.0, -4500.0)
    assert up_l == pytest.approx(2250 * 600 / 4500)
    assert down_l == pytest.approx(-2250 * 400 / 4500)


def test_mixer_signs_roll_right_lowers_left_and_raises_right():
    emap = ElevonMap.from_params(PARAMS)
    left, right = emap.travel(1000.0, 0.0)
    assert left < 0 < right


def _multistep(n, dt, amp=15.0, unit=0.2):
    """3-2-1-1 pulses of both polarities with level gaps, repeated to length n."""
    seq = []
    for sign in (1.0, -1.0):
        for mult, s in ((3, 1.0), (2, -1.0), (1, 1.0), (1, -1.0)):
            seq += [sign * s * amp] * int(round(mult * unit / dt))
        seq += [0.0] * int(round(1.5 / dt))
    return np.resize(np.array(seq), n)


def _synthetic_roll_data(theta, seconds=16.0, seed=1):
    """Closed-loop data from a known roll model, with gyro noise and a gusty disturbance."""
    dt = 1.0 / PARAMS["SCHED_LOOP_RATE"]
    n = int(seconds / dt)
    t = np.arange(n) * dt
    demand = _multistep(n, dt)
    rng = np.random.default_rng(seed)
    dist = np.convolve(rng.standard_normal(n), np.ones(20) / 20, "same") * 15.0
    asp = np.full(n, 22.0)
    res = closed_loop.simulate("roll", PARAMS, Gains.from_params(PARAMS, "roll"), theta, 22.0,
                               ElevonMap.from_params(PARAMS), demand, asp, np.ones(n), np.zeros(n), np.zeros(n),
                               dt, start_ms=0, angle0=0.0, rate0=0.0, disturbance=dist)
    rate = res["rate"][0] + rng.normal(0, 0.5, n)
    angle = res["angle"][0]
    d = AxisData(axis="roll", dt=dt, t=t, demand=demand, angle=angle, rate=rate, out_this=res["u"][0],
                 out_other=np.zeros(n), i_this=np.zeros(n), aspeed=asp, e2t=np.ones(n), roll=np.zeros(n),
                 pitch=np.zeros(n), params=PARAMS)
    half = n // 2
    d.segments = [(0, half), (half, n)]
    return d


def _roll_response(theta, w, dt):
    s = 1j * w
    return theta[1] / (s + theta[0]) * np.exp(-s * (theta[3] / 1000 + dt)) / (1 + theta[4] / 1000 * s)


def test_closed_loop_fit_recovers_frequency_response_of_known_plant():
    # The roll pole and the actuator lag can swap roles without changing the response at
    # one airspeed, so the parameters themselves are not unique. What the gain design
    # uses is the frequency response near crossover, which sharp pulses must pin down.
    truth = np.array([6.0, 0.6, 0.0, 20.0, 40.0])        # a0, k, m0, delay_ms, lag_ms
    d = _synthetic_roll_data(truth)
    fit = fit_axis(d, [d.segments[0]], v0=22.0, max_gen=40, popsize=16, seed=3)
    dt = d.dt
    w = np.array([2.0, 5.0, 10.0])
    ratio = _roll_response(fit.theta, w, dt) / _roll_response(truth, w, dt)
    assert np.all(np.abs(np.abs(ratio) - 1.0) < 0.15)
    assert np.all(np.abs(np.degrees(np.angle(ratio))) < 8.0)
    # and it predicts unseen data about as well as the true model does (the gusts are unmodelled)
    from tools.tuning.model_id import FitResult
    true_fit = FitResult(axis="roll", names=fit.names, theta=truth, cost=0.0, v0=22.0)
    held = validate(d, fit, [d.segments[1]])["nrmse_rate_median"]
    held_true = validate(d, true_fit, [d.segments[1]])["nrmse_rate_median"]
    assert held <= 1.25 * held_true + 0.03


def test_phase_and_gain_margin_of_delayed_integrator():
    # L = K e^{-s tau} / s: crossover at K, PM = 90 - K tau (deg), GM at w = pi / (2 tau)
    k, tau = 10.0, 0.02
    w = np.logspace(-1, 3, 4000)
    L = (k * np.exp(-1j * w * tau) / (1j * w))[None, :]
    pm, gm = design._pm_gm(L, w)
    assert pm[0] == pytest.approx(90.0 - np.degrees(k * tau), abs=0.5)
    assert gm[0] == pytest.approx(20 * np.log10((np.pi / (2 * tau)) / k), abs=0.2)


def test_step_metrics_of_first_order_response():
    dt = 0.01
    spec = design.Spec(axis="roll", speeds=(22.0,), steps_deg=(20.0,), hold_s=3.0)
    demand, windows = design.card_demand(spec, dt)
    tau = 0.2
    angle = np.zeros(len(demand))
    for k in range(1, len(demand)):
        angle[k] = angle[k - 1] + dt / tau * (demand[k - 1] - angle[k - 1])
    m = design.step_metrics(angle[None, :], windows, dt)
    assert m["overshoot"].max() == pytest.approx(0.0, abs=1e-9)
    assert m["t90"][0, 0] == pytest.approx(tau * np.log(10), abs=0.02)


def test_score_penalises_gains_that_destroy_margins():
    theta = np.array([6.0, 0.6, 0.0, 20.0, 40.0])
    base = Gains.from_params(PARAMS, "roll")
    emap = ElevonMap.from_params(PARAMS)
    spec = design.Spec(axis="roll", speeds=(22.0,), steps_deg=(20.0,), hold_s=2.0)
    good = design.gains_vector(base)
    wild = good * np.array([1.0, 30.0, 1.0, 20.0, 1.0])  # P and D far too high
    sc = design.score(spec, PARAMS, base, np.vstack([good, wild]), [theta], 22.0, emap)
    assert sc.metrics["pm_min"][1] < sc.metrics["pm_min"][0]
    assert sc.cost[1] > sc.cost[0]
