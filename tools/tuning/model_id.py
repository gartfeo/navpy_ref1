"""Closed-loop identification of roll and pitch dynamics from loop-rate logs.

The whole loop is simulated from the logged attitude demand: the ArduPlane 4.6
controller port (ap_controller.py), ArduPilot's elevon mixer and two-sided servo
PWM mapping, then a candidate aircraft model. Model parameters are fitted so the
simulated body rate and attitude match the log (closed-loop output error). The
simulated surface command depends only on the demand and the model, not on
measured noise fed back through the controller, which is what biases a direct fit
of logged controller output against logged rate.

Surface commands are expressed as elevon "up" travel in microseconds of PWM from
SERVOn_TRIM, because pwm_from_angle (libraries/SRV_Channel/SRV_Channel.cpp) maps
positive and negative commands onto MAX-TRIM and TRIM-MIN separately: with a trim
away from mid-range the same command moves the surface further one way.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterable

import numpy as np

from tools.tuning import closed_loop
from tools.tuning.ap_controller import Gains
from tools.tuning.closed_loop import PITCH_PARAMS, ROLL_PARAMS
from tools.tuning.optim import cmaes

if TYPE_CHECKING:
    from tools.tuning.logdata import FlightLog

DEG = 180.0 / np.pi


# ---- elevon mixer and servo mapping -------------------------------------------
@dataclass
class ElevonMap:
    """Plane::channel_function_mixer plus SRV_Channel::pwm_from_angle for both elevons."""
    gain: float = 0.5
    offset: float = 0.0
    left: tuple = (1000.0, 1500.0, 2000.0, False)    # MIN, TRIM, MAX, REVERSED
    right: tuple = (1000.0, 1500.0, 2000.0, False)

    @classmethod
    def from_params(cls, p: dict) -> "ElevonMap":
        left = right = None
        for n in range(1, 17):
            func = p.get(f"SERVO{n}_FUNCTION")
            spec = (p.get(f"SERVO{n}_MIN", 1100.0), p.get(f"SERVO{n}_TRIM", 1500.0),
                    p.get(f"SERVO{n}_MAX", 1900.0), bool(p.get(f"SERVO{n}_REVERSED", 0)))
            if func == 77:
                left = spec
            elif func == 78:
                right = spec
        if left is None or right is None:
            raise ValueError("no elevon outputs (SERVOn_FUNCTION 77/78)")
        return cls(gain=p.get("MIXING_GAIN", 0.5), offset=p.get("MIXING_OFFSET", 0.0), left=left, right=right)

    @staticmethod
    def _up_travel(out_cd: np.ndarray, spec: tuple[float, float, float, bool]) -> np.ndarray:
        """Up travel in us for an 'up' command out_cd (+-4500 = full)."""
        lo, trim, hi, rev = spec
        out = np.clip(out_cd, -4500.0, 4500.0)
        pos_span, neg_span = (trim - lo, hi - trim) if rev else (hi - trim, trim - lo)
        return np.where(out > 0, out * pos_span / 4500.0, out * neg_span / 4500.0)

    def travel(self, ail_cd: np.ndarray | float, elev_cd: np.ndarray | float) -> tuple[np.ndarray, np.ndarray]:
        """(left_up_us, right_up_us) for aileron and elevator commands in centidegrees."""
        in1, in2 = np.asarray(ail_cd, dtype=float), np.asarray(elev_cd, dtype=float)
        if self.offset < 0:
            in2 = in2 * (100 - self.offset) * 0.01
        elif self.offset > 0:
            in1 = in1 * (100 + self.offset) * 0.01
        out1 = np.clip((in2 - in1) * self.gain, -4500, 4500)     # left: up for pitch up, down for roll right
        out2 = np.clip((in2 + in1) * self.gain, -4500, 4500)
        return self._up_travel(out1, self.left), self._up_travel(out2, self.right)


# ---- data ----------------------------------------------------------------------
@dataclass
class AxisData:
    """Loop-rate samples for one axis, cut into fit segments."""
    axis: str
    dt: float
    t: np.ndarray
    demand: np.ndarray          # angle demand at the controller input, deg
    angle: np.ndarray           # measured angle, deg
    rate: np.ndarray            # measured body rate (PID Act), deg/s
    out_this: np.ndarray        # logged controller output of this axis, deg (+-45)
    out_other: np.ndarray       # logged controller output of the other axis, deg
    i_this: np.ndarray          # logged integrator term of this axis, deg
    aspeed: np.ndarray          # EAS, m/s
    e2t: np.ndarray
    roll: np.ndarray            # measured roll and pitch, rad (pitch coordination term)
    pitch: np.ndarray
    params: dict
    segments: list = field(default_factory=list)


def axis_data(log: FlightLog, axis: str, t0: float | None = None, t1: float | None = None,
              mode: int | None = 5, seg_len_s: float = 12.0) -> AxisData:
    """Build AxisData from a FlightLog (logdata.standard_log) for FBWA flight."""
    this, other = ("PIDR", "PIDP") if axis == "roll" else ("PIDP", "PIDR")
    t = log.get(this, "TimeUS")
    sel = np.ones(len(t), dtype=bool)
    if t0 is not None:
        sel &= t >= t0 * 1e6
    if t1 is not None:
        sel &= t <= t1 * 1e6
    if mode is not None and log.has("MODE"):
        mt, mn = log.get("MODE", "TimeUS"), log.get("MODE", "ModeNum")
        idx = np.searchsorted(mt, t, side="right") - 1
        sel &= (idx >= 0) & (mn[np.maximum(idx, 0)] == mode)
    if log.has("STAT"):
        sel &= np.interp(t, log.get("STAT", "TimeUS"), log.get("STAT", "isFlying")) > 0.5
    t = t[sel]

    def pid_sum(mtype: str) -> np.ndarray:
        tt = log.get(mtype, "TimeUS")
        s = sum(log.get(mtype, k) for k in ("FF", "P", "I", "D", "DFF"))
        return np.clip(np.interp(t, tt, s), -45, 45)

    att_t = log.get("ATT", "TimeUS")
    des_key, ang_key = ("DesRoll", "Roll") if axis == "roll" else ("DesPitch", "Pitch")
    params = dict(log.params)
    demand = np.interp(t, att_t, log.get("ATT", des_key))
    if axis == "pitch":
        # stabilize_pitch adds PTCH_TRIM_DEG and KFF_THR2PTCH * throttle to the logged demand
        thr = np.interp(t, log.get("CTUN", "TimeUS"), log.get("CTUN", "ThO"))
        demand = demand + params.get("PTCH_TRIM_DEG", 0.0) + params.get("KFF_THR2PTCH", 0.0) * thr
    data = AxisData(
        axis=axis, dt=1.0 / params.get("SCHED_LOOP_RATE", 50.0), t=t / 1e6,
        demand=demand, angle=np.interp(t, att_t, log.get("ATT", ang_key)),
        rate=np.interp(t, log.get(this, "TimeUS"), log.get(this, "Act")),
        out_this=pid_sum(this), out_other=pid_sum(other),
        i_this=np.interp(t, log.get(this, "TimeUS"), log.get(this, "I")),
        aspeed=np.interp(t, log.get("CTUN", "TimeUS"), log.get("CTUN", "As")),
        e2t=np.interp(t, log.get("CTUN", "TimeUS"), log.get("CTUN", "E2T")),
        roll=np.radians(np.interp(t, att_t, log.get("ATT", "Roll"))),
        pitch=np.radians(np.interp(t, att_t, log.get("ATT", "Pitch"))),
        params=params)
    data.segments = split_segments(data, seg_len_s)
    return data


def split_segments(d: AxisData, seg_len_s: float) -> list[tuple[int, int]]:
    """Contiguous windows of seg_len_s with some demand activity and no output saturation."""
    gaps = np.flatnonzero(np.diff(d.t) > 3 * d.dt)
    bounds = np.concatenate(([0], gaps + 1, [len(d.t)]))
    n_seg = int(round(seg_len_s / d.dt))
    segs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        for s in range(a, b - n_seg + 1, n_seg):
            e = s + n_seg
            if np.ptp(d.demand[s:e]) < 3.0:
                continue                        # nothing to identify
            if np.any(np.abs(d.out_this[s:e]) >= 44.9):
                continue                        # controller output saturated
            segs.append((s, e))
    return segs


# ---- models --------------------------------------------------------------------
@dataclass
class FitResult:
    axis: str
    names: tuple
    theta: np.ndarray           # best parameters
    cost: float
    v0: float
    bootstrap: np.ndarray | None = None
    stats: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return dict(zip(self.names, map(float, self.theta)))


def simulate_segment(d: AxisData, s: int, e: int, theta: np.ndarray, v0: float) -> tuple[np.ndarray, np.ndarray]:
    """Closed-loop simulation of one logged segment with the logged gains; returns (rate, angle) (N, T)."""
    theta = np.atleast_2d(theta)
    p = d.params
    res = closed_loop.simulate(
        d.axis, p, Gains.from_params(p, d.axis, theta.shape[0]), theta, v0, ElevonMap.from_params(p),
        d.demand[s:e], d.aspeed[s:e], d.e2t[s:e], d.roll[s:e], d.out_other[s:e], d.dt,
        start_ms=int(d.t[s] * 1000), angle0=d.angle[s], rate0=d.rate[s], integrator0=d.i_this[s] / DEG)
    return res["rate"], res["angle"]


def _cost(d: AxisData, segs: list[tuple[int, int]], theta: np.ndarray, v0: float,
          w_angle: float = 1.0) -> np.ndarray:
    total = np.zeros(theta.shape[0])
    for s, e in segs:
        r, a = simulate_segment(d, s, e, theta, v0)
        er = (r - d.rate[s:e]) / max(np.std(d.rate[s:e]), 1.0)
        ea = (a - d.angle[s:e]) / max(np.std(d.angle[s:e]), 1.0)
        c = np.mean(er ** 2, axis=1) + w_angle * np.mean(ea ** 2, axis=1)
        total += np.where(np.isfinite(c), c, 1e6)
    return total / max(len(segs), 1)


def default_bounds(axis: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if axis == "roll":
        # a0 [1/s], k [deg/s^2 per us], m0 [deg/s^2], delay [ms], lag [ms]
        x0 = np.array([5.0, 0.5, 0.0, 15.0, 20.0])
        lo = np.array([0.3, 0.005, -300.0, 0.0, 0.0])
        hi = np.array([100.0, 50.0, 300.0, 100.0, 400.0])
    else:
        # wn0 [rad/s], zeta, z0 [1/s], k, c0, c1, delay [ms], lag [ms]
        x0 = np.array([6.0, 0.6, 2.0, 0.5, 0.0, 0.0, 15.0, 20.0])
        lo = np.array([1.0, 0.1, 0.1, 0.005, -500.0, -500.0, 0.0, 0.0])
        hi = np.array([40.0, 2.0, 20.0, 50.0, 500.0, 500.0, 100.0, 400.0])
    return x0, lo, hi


def fit_axis(d: AxisData, segs: Iterable[tuple[int, int]] | None = None, v0: float | None = None,
             max_gen: int = 40, seed: int = 0, popsize: int = 16) -> FitResult:
    """CMA-ES fit in normalised parameter space; returns the best model."""
    segs = list(segs if segs is not None else d.segments)
    if not segs:
        raise ValueError(f"no usable {d.axis} segments")
    v0 = v0 or float(np.median(d.aspeed[np.concatenate([np.arange(s, e) for s, e in segs])]))
    x0, lo, hi = default_bounds(d.axis)
    span = hi - lo

    def f(z: np.ndarray) -> np.ndarray:
        theta = lo + span * np.clip(z, 0, 1)
        return _cost(d, segs, theta, v0)

    z0 = (x0 - lo) / span
    zb, fb, hist = cmaes(f, z0, 0.15, max_gen=max_gen, popsize=popsize, seed=seed,
                         lower=np.zeros_like(z0), upper=np.ones_like(z0))
    names = ROLL_PARAMS if d.axis == "roll" else PITCH_PARAMS
    return FitResult(axis=d.axis, names=names, theta=lo + span * zb, cost=float(fb), v0=v0,
                     stats={"history": hist, "segments": segs})


def simulate(d: AxisData, fit: FitResult, s: int, e: int) -> tuple[np.ndarray, np.ndarray]:
    """Simulated (rate, angle) for one segment with the fitted model."""
    r, a = simulate_segment(d, s, e, fit.theta[None, :], fit.v0)
    return r[0], a[0]


def validate(d: AxisData, fit: FitResult, segs: Iterable[tuple[int, int]]) -> dict:
    """Prediction quality on segments: NRMSE of rate and angle, and residual whiteness."""
    out = []
    for s, e in segs:
        r, a = simulate(d, fit, s, e)
        res = d.rate[s:e] - r
        nrmse_r = float(np.sqrt(np.mean(res ** 2)) / max(np.std(d.rate[s:e]), 1e-6))
        nrmse_a = float(np.sqrt(np.mean((d.angle[s:e] - a) ** 2)) / max(np.std(d.angle[s:e]), 1e-6))
        # lag-1 autocorrelation of the rate residual and its correlation with the demand
        rc = float(np.corrcoef(res[:-1], res[1:])[0, 1]) if len(res) > 2 else np.nan
        dc = float(np.corrcoef(res, d.demand[s:e])[0, 1]) if np.std(d.demand[s:e]) > 0 else np.nan
        out.append({"t0": float(d.t[s]), "nrmse_rate": nrmse_r, "nrmse_angle": nrmse_a,
                    "resid_lag1": rc, "resid_demand_corr": dc})
    return {"segments": out,
            "nrmse_rate_median": float(np.median([o["nrmse_rate"] for o in out])) if out else np.nan,
            "nrmse_angle_median": float(np.median([o["nrmse_angle"] for o in out])) if out else np.nan}
