"""Batched closed-loop simulation: ArduPlane 4.6 controller port + elevon mapping + aircraft model.

Used both to identify the aircraft model (model_id.py: many models, logged gains) and
to design gains (design.py: one model, many gain sets). Every array runs over a batch
of N candidates.

Aircraft models (input: elevon travel in us from SERVOn_TRIM, see model_id.ElevonMap):
- roll:  p' = -a0*r*p + k*r^2*act + m0*r^2, phi' = p
- pitch: x2' = -wn^2 x1 - 2 zeta wn x2 + k*r^2*act + c0*r^2 + c1, x1' = x2,
         q = z x1 + x2, theta' = q cos(phi), with wn = wn0*r and z = z0*r
where r = EAS / v0, and act is the travel command after a pure delay and a first-order lag.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from tools.tuning.ap_controller import Gains, RateController, speed_scaler

if TYPE_CHECKING:
    from tools.tuning.model_id import ElevonMap

DEG = 180.0 / np.pi
ROLL_PARAMS = ("a0", "k", "m0", "delay_ms", "lag_ms")
PITCH_PARAMS = ("wn0", "zeta", "z0", "k", "c0", "c1", "delay_ms", "lag_ms")
HIST_LEN = 64


def simulate(axis: str, params: dict, gains: Gains, theta: np.ndarray, v0: float, emap: ElevonMap,
             demand: np.ndarray, aspeed: np.ndarray, e2t: np.ndarray, roll_rad: np.ndarray,
             other_out: np.ndarray, dt: float, start_ms: int, angle0: float, rate0: float, integrator0: float = 0.0,
             disturbance: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """Run the loop over len(demand) samples. Returns dict of (N, T) arrays.

    theta: (N, p) model parameters or (p,) for one model shared by the batch.
    gains: Gains with arrays of shape (N,) (or broadcastable).
    roll_rad: measured or fixed roll angle per sample (pitch coordination term).
    other_out: the other axis' controller output in degrees per sample (elevon mixing).
    disturbance: optional (T,) or (N, T) moment added to the rate equation, deg/s^2.
    """
    theta = np.asarray(theta, dtype=float)
    n = max(theta.shape[0] if theta.ndim == 2 else 1, np.size(gains.P))
    th = np.broadcast_to(theta if theta.ndim == 2 else theta[None, :], (n, theta.shape[-1]))
    steps = len(demand)
    ctl = RateController(axis, gains, n, params["AIRSPEED_MIN"], params["AIRSPEED_MAX"],
                         roll_limit_deg=params.get("ROLL_LIMIT_DEG", 45.0), roll_ff=params.get("PTCH2SRV_RLL", 1.0))
    ctl.pid.integrator[:] = integrator0
    scaling_speed = params.get("SCALING_SPEED", 15.0)
    delay_samples = np.clip(th[:, -2] / 1000.0 / dt, 0, HIST_LEN - 2)
    di = np.floor(delay_samples).astype(int)
    df = delay_samples - di
    lag_a = np.where(th[:, -1] > 0.1, dt / (dt + th[:, -1] / 1000.0), 1.0)
    rows = np.arange(n)
    hist = np.zeros((n, HIST_LEN))
    head = 0
    angle = np.full(n, float(angle0))
    rate = np.full(n, float(rate0))
    act = np.zeros(n)
    x1 = np.zeros(n)
    x2 = np.zeros(n)
    if axis == "pitch":
        # start the second-order states at the given rate with zero acceleration
        z = th[:, 2] * (aspeed[0] / v0)
        x1 = np.where(z > 1e-6, rate / np.maximum(z, 1e-6), 0.0)
    res = {k: np.zeros((n, steps)) for k in ("rate", "angle", "u", "dmod", "travel")}
    for j in range(steps):
        v = aspeed[j]
        r = v / v0
        scaler = speed_scaler(v, scaling_speed, params["AIRSPEED_MIN"], params["AIRSPEED_MAX"])
        err = demand[j] - angle
        if axis == "roll":
            des = ctl.desired_rate(err)
        else:
            des = ctl.desired_rate(err, roll_rad=roll_rad[j], pitch_rad=np.radians(angle), aspeed=v, eas2tas=e2t[j])
        u = ctl.rate_out(des, np.radians(rate), scaler, e2t[j], v, dt, start_ms + int(j * dt * 1000))
        if axis == "roll":
            left, right = emap.travel(u * 100.0, other_out[j] * 100.0)
            cmd = right - left
        else:
            left, right = emap.travel(other_out[j] * 100.0, u * 100.0)
            cmd = left + right
        head = (head - 1) % HIST_LEN
        hist[:, head] = cmd
        delayed = (1 - df) * hist[rows, (head + di) % HIST_LEN] + df * hist[rows, (head + di + 1) % HIST_LEN]
        act = act + lag_a * (delayed - act)
        dist = 0.0 if disturbance is None else (disturbance[..., j] if np.ndim(disturbance) == 2 else disturbance[j])
        if axis == "roll":
            rate = rate + dt * (-th[:, 0] * r * rate + th[:, 1] * r * r * act + th[:, 2] * r * r + dist)
            angle = angle + dt * rate
        else:
            wn, zeta, zz = th[:, 0] * r, th[:, 1], th[:, 2] * r
            drive = th[:, 3] * r * r * act + th[:, 4] * r * r + th[:, 5] + dist
            x2 = x2 + dt * (-wn * wn * x1 - 2 * zeta * wn * x2 + drive)
            x1 = x1 + dt * x2
            rate = zz * x1 + x2
            angle = angle + dt * rate * np.cos(roll_rad[j])
        res["rate"][:, j] = rate
        res["angle"][:, j] = angle
        res["u"][:, j] = u
        res["dmod"][:, j] = ctl.info.Dmod
        res["travel"][:, j] = cmd
    return res
