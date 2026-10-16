"""Search ArduPlane roll/pitch gains against identified aircraft models.

Each candidate gain set is scored with the controller port in closed loop
(closed_loop.simulate) on the test-card demands at several airspeeds and on every
model in the identified set, plus linear gain/phase margins of the rate loop and
the complete attitude loop. The baseline gains are scored the same way, so the
result says by how much the candidate is predicted to be better, and where.

Searched: FF, P, I, D, TCONST. FLTT follows TCONST with AUTOTUNE's rule
10/(2 pi TCONST); RMAX, IMAX, FLTD, FLTE and SMAX keep the aircraft's values.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Sequence

import numpy as np

from tools.tuning import closed_loop
from tools.tuning.ap_controller import Gains, speed_scaler
from tools.tuning.optim import cmaes

if TYPE_CHECKING:
    from tools.tuning.model_id import ElevonMap

SEARCHED = ("FF", "P", "I", "D", "TCONST")


@dataclass
class Spec:
    """Goals and limits; all fixed before the search."""
    axis: str
    speeds: tuple                    # EAS values to evaluate, m/s
    steps_deg: tuple = (20.0, 30.0)  # step sizes, flown both directions
    hold_s: float = 3.0
    pm_min_deg: float = 45.0
    gm_min_db: float = 6.0
    overshoot_max: float = 0.10
    w_overshoot: float = 4.0
    w_dist: float = 0.5
    w_servo: float = 0.02
    other_out_deg: float = 0.0       # the other axis' steady output (elevon operating point)
    seed: int = 0


@dataclass
class Score:
    cost: np.ndarray
    metrics: dict = field(default_factory=dict)


def card_demand(spec: Spec, dt: float) -> tuple[np.ndarray, list[tuple[int, int, float, float]]]:
    """Level, then each step size up and down with returns to level. Returns (demand, windows)."""
    seq = [(0.0, 1.0)]
    for a in spec.steps_deg:
        seq += [(a, spec.hold_s), (0.0, spec.hold_s), (-a, spec.hold_s), (0.0, spec.hold_s)]
    demand, windows, k = [], [], 0
    prev = 0.0
    for value, dur in seq:
        n = int(round(dur / dt))
        if value != prev:
            windows.append((k, k + n, prev, value))
        demand.append(np.full(n, value))
        k += n
        prev = value
    return np.concatenate(demand), windows


def step_metrics(angle: np.ndarray, windows: list[tuple[int, int, float, float]], dt: float) -> dict:
    """Per-step normalised mean absolute error, overshoot and time to 90 % (arrays (N, steps))."""
    mae, over, t90 = [], [], []
    for s, e, a0, a1 in windows:
        size = a1 - a0
        y = (angle[:, s:e] - a0) / size          # 0 -> 1 progress
        mae.append(np.mean(np.abs(1.0 - y), axis=1))
        over.append(np.maximum(np.max(y, axis=1) - 1.0, 0.0))
        reached = y >= 0.9
        first = np.where(reached.any(axis=1), reached.argmax(axis=1), y.shape[1])
        t90.append(first * dt)
    return {"mae": np.array(mae).T, "overshoot": np.array(over).T, "t90": np.array(t90).T}


def candidate_gains(base: Gains, x: np.ndarray) -> Gains:
    """Gains for a batch of searched vectors x (N, 5) in the order of SEARCHED."""
    n = x.shape[0]

    def rep(v: np.ndarray) -> np.ndarray:
        return np.broadcast_to(np.asarray(v, dtype=float).ravel()[:1], (n,)).copy()
    tconst = x[:, 4]
    return replace(base, FF=x[:, 0], P=x[:, 1], I=x[:, 2], D=x[:, 3], TCONST=tconst,
                   FLTT=10.0 / (2 * np.pi * tconst), IMAX=rep(base.IMAX), FLTE=rep(base.FLTE),
                   FLTD=rep(base.FLTD), SMAX=rep(base.SMAX), RMAX_POS=rep(base.RMAX_POS),
                   RMAX_NEG=rep(base.RMAX_NEG), D_FF=rep(base.D_FF), PDMX=rep(base.PDMX))


def gains_vector(g: Gains) -> np.ndarray:
    return np.array([float(np.ravel(getattr(g, k))[0]) for k in SEARCHED])


# ---- linear margins -------------------------------------------------------------
def _lpf(s: np.ndarray, hz: np.ndarray) -> np.ndarray:
    hz = np.asarray(hz, dtype=float)[:, None]
    return np.where(hz > 0, 1.0 / (1.0 + s / (2 * np.pi * np.maximum(hz, 1e-9))), 1.0)


def travel_per_deg(axis: str, emap: ElevonMap, other_out_deg: float) -> tuple[float, float]:
    """Linearised elevon travel (us) per degree of controller output, for + and - commands."""
    gains = []
    for sign in (1.0, -1.0):
        u = sign * 2.0
        if axis == "roll":
            l0, r0 = emap.travel(0.0, other_out_deg * 100)
            l1, r1 = emap.travel(u * 100, other_out_deg * 100)
            gains.append(float(((r1 - l1) - (r0 - l0)) / u))
        else:
            l0, r0 = emap.travel(other_out_deg * 100, 0.0)
            l1, r1 = emap.travel(other_out_deg * 100, u * 100)
            gains.append(float(((l1 + r1) - (l0 + r0)) / u))
    return gains[0], gains[1]


def margins(axis: str, g: Gains, theta: np.ndarray, v: float, v0: float, params: dict, kt: float,
            e2t: float = 1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(pm_rate, gm_rate, pm_att, gm_att) in degrees and dB for each candidate in g."""
    w = np.logspace(-1, np.log10(600.0), 800)
    s = 1j * w[None, :]
    th = np.asarray(theta, dtype=float)
    dt = 1.0 / params.get("SCHED_LOOP_RATE", 50.0)
    r = v / v0
    sc = float(speed_scaler(v, params.get("SCALING_SPEED", 15.0), params["AIRSPEED_MIN"], params["AIRSPEED_MAX"]))
    delay = th[-2] / 1000.0 + dt
    lag = th[-1] / 1000.0
    if axis == "roll":
        plant = kt * th[1] * r * r / (s + th[0] * r)
    else:
        wn, zeta, z = th[0] * r, th[1], th[2] * r
        plant = kt * th[3] * r * r * (s + z) / (s * s + 2 * zeta * wn * s + wn * wn)
    plant = plant * np.exp(-s * delay) / (1.0 + lag * s)
    P, I, D, FF = (np.asarray(x, dtype=float)[:, None] for x in (g.P, g.I, g.D, g.FF))
    tau = np.maximum(np.asarray(g.TCONST, dtype=float)[:, None], 0.05)
    fe, ft, fd = _lpf(s, g.FLTE), _lpf(s, g.FLTT), _lpf(s, g.FLTD)
    c_pid = sc * sc * ((P + I / s) * fe + D * s * fe * fd)
    k_ff = FF * sc / e2t
    l_rate = plant * c_pid
    l_att = plant * (c_pid * (1.0 + ft / (tau * s)) + k_ff * ft / (tau * s))
    return (*_pm_gm(l_rate, w), *_pm_gm(l_att, w))


def _pm_gm(L: np.ndarray, w: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L), axis=1))
    n = L.shape[0]
    pm = np.full(n, 180.0)
    gm = np.full(n, 60.0)
    for i in range(n):
        down = np.flatnonzero((mag[i, :-1] >= 1.0) & (mag[i, 1:] < 1.0))
        if len(down):
            # phase margin wrapped to [-180, 180): negative means unstable at that crossover
            pm[i] = min(((ph[i, k] + 360.0) % 360.0) - 180.0 for k in down)
        cross = np.flatnonzero(np.floor((ph[i, :-1] + 180.0) / 360.0) != np.floor((ph[i, 1:] + 180.0) / 360.0))
        if len(cross):
            gm[i] = min(-20 * np.log10(max(mag[i, k], 1e-12)) for k in cross)
    return pm, gm


# ---- scoring --------------------------------------------------------------------
def disturbance(n_steps: int, dt: float, rms: float, seed: int) -> np.ndarray:
    """Band-limited (about 2 Hz) random moment, same for every candidate."""
    rng = np.random.default_rng(seed)
    x = rng.standard_normal(n_steps)
    a = dt / (dt + 1.0 / (2 * np.pi * 2.0))
    y = np.zeros(n_steps)
    for k in range(1, n_steps):
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y * rms / max(np.std(y), 1e-9)


def score(spec: Spec, params: dict, base: Gains, x: np.ndarray, models: Sequence[np.ndarray], v0: float,
          emap: ElevonMap) -> Score:
    """Cost (lower is better) for each candidate row of x (N, 5) over all models and speeds."""
    dt = 1.0 / params.get("SCHED_LOOP_RATE", 50.0)
    g = candidate_gains(base, x)
    n = x.shape[0]
    demand, windows = card_demand(spec, dt)
    t_steps = len(demand)
    kt_pos, kt_neg = travel_per_deg(spec.axis, emap, spec.other_out_deg)
    track = np.zeros(n)
    over_pen = np.zeros(n)
    dist_cost = np.zeros(n)
    servo = np.zeros(n)
    sat = np.zeros(n)
    dmod_bad = np.zeros(n)
    pm_min = np.full(n, 180.0)
    gm_min = np.full(n, 60.0)
    worst = {"mae": np.zeros(n), "overshoot": np.zeros(n), "t90": np.zeros(n)}
    count = 0
    for theta in models:
        for v in spec.speeds:
            asp = np.full(t_steps, v)
            e2t = np.ones(t_steps)
            zeros = np.zeros(t_steps)
            other = np.full(t_steps, spec.other_out_deg)
            res = closed_loop.simulate(spec.axis, params, g, theta, v0, emap, demand, asp, e2t, zeros, other, dt,
                                       start_ms=0, angle0=0.0, rate0=0.0)
            m = step_metrics(res["angle"], windows, dt)
            track += m["mae"].mean(axis=1)
            over_pen += np.maximum(m["overshoot"] - spec.overshoot_max, 0.0).sum(axis=1)
            for key in worst:
                worst[key] = np.maximum(worst[key], m[key].max(axis=1))
            servo += np.sqrt(np.mean(np.diff(res["u"], axis=1) ** 2, axis=1)) / dt
            sat += np.mean(np.abs(res["u"]) >= 44.9, axis=1)
            dmod_bad += np.mean(res["dmod"] < 0.999, axis=1)
            # turbulence: zero demand, the same random moment for every candidate
            dist = disturbance(int(10.0 / dt), dt, rms=20.0, seed=spec.seed)
            dres = closed_loop.simulate(spec.axis, params, g, theta, v0, emap, np.zeros(len(dist)),
                                        np.full(len(dist), v), np.ones(len(dist)), np.zeros(len(dist)),
                                        np.full(len(dist), spec.other_out_deg), dt, start_ms=0,
                                        angle0=0.0, rate0=0.0, disturbance=dist)
            dist_cost += np.sqrt(np.mean(dres["angle"] ** 2, axis=1))
            for kt in (kt_pos, kt_neg):
                pr, gr, pa, ga = margins(spec.axis, g, theta, v, v0, params, kt)
                pm_min = np.minimum(pm_min, np.minimum(pr, pa))
                gm_min = np.minimum(gm_min, np.minimum(gr, ga))
            count += 1
    track /= count
    dist_cost /= count
    servo /= count
    margin_pen = np.maximum(spec.pm_min_deg - pm_min, 0.0) / 5.0 + np.maximum(spec.gm_min_db - gm_min, 0.0)
    cost = (track + spec.w_overshoot * over_pen / count + spec.w_dist * dist_cost / 10.0
            + spec.w_servo * servo / 100.0 + 10.0 * sat / count + 10.0 * dmod_bad / count + 10.0 * margin_pen)
    cost = np.where(np.isfinite(cost), cost, 1e9)
    metrics = {"tracking_mae": track, "overshoot_worst": worst["overshoot"], "t90_worst": worst["t90"],
               "mae_worst": worst["mae"], "dist_rms_deg": dist_cost, "servo_rate": servo,
               "saturation": sat / count, "dmod_active": dmod_bad / count, "pm_min": pm_min, "gm_min": gm_min}
    return Score(cost=cost, metrics=metrics)


def search(spec: Spec, params: dict, base: Gains, models: Sequence[np.ndarray], v0: float, emap: ElevonMap,
           max_gen: int = 40, popsize: int = 16) -> tuple[np.ndarray, Score, Score, list[float]]:
    """CMA-ES in log-gain space around the baseline; returns (best_x, best_score, baseline_score)."""
    x_base = gains_vector(base)
    x_base = np.maximum(x_base, [0.02, 0.005, 0.005, 0.0002, 0.15])
    lo = np.log(np.array([0.02, 0.003, 0.003, 0.00005, 0.12]))
    hi = np.log(np.array([3.0, 2.0, 3.0, 0.05, 1.0]))

    def f(z: np.ndarray) -> np.ndarray:
        return score(spec, params, base, np.exp(z), models, v0, emap).cost

    zb, fb, hist = cmaes(f, np.log(x_base), 0.4, max_gen=max_gen, popsize=popsize, seed=spec.seed,
                         lower=lo, upper=hi)
    best = np.exp(zb)
    return best, score(spec, params, base, best[None, :], models, v0, emap), \
        score(spec, params, base, gains_vector(base)[None, :], models, v0, emap), hist
