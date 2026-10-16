"""Python port of the ArduPlane 4.6 roll and pitch attitude controllers.

Ported code path by code path from the ArduPilot-4.6 branch so that gains can be
scored offline exactly as the autopilot would use them:

- libraries/APM_Control/AP_RollController.cpp  (_get_rate_out, get_servo_out)
- libraries/APM_Control/AP_PitchController.cpp (_get_rate_out, get_servo_out,
  _get_coordination_rate_offset)
- libraries/AC_PID/AC_PID.cpp                  (update_all, update_i)
- ArduPlane/Attitude.cpp                       (calc_speed_scaler, airspeed branch)

The slew limiter and low-pass filter they use are ported in ap_filter.py.

Every state is a numpy array over a batch of N gain sets, so one Python time loop
evaluates N candidates at once. Gains are passed as arrays of shape (N,) or scalars.
Units follow the C++ code: rates in deg/s at the interface, AC_PID internals in
rad/s multiplied by scaler**2, outputs in degrees of notional deflection (+-45).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from tools.tuning.ap_filter import SlewLimiter, lowpass_alpha

GRAVITY_MSS = 9.80665
DEG = 180.0 / np.pi
SLEW_LIMIT_SCALE = 45.0         # rate_pid.set_slew_limit_scale(45) in both controllers
MIN_AIRSPEED_MIN = 5            # ArduPlane/defines.h


@dataclass
class Gains:
    """One axis' rate and angle controller parameters (arrays over the batch)."""
    P: np.ndarray
    I: np.ndarray
    D: np.ndarray
    FF: np.ndarray
    IMAX: np.ndarray
    FLTT: np.ndarray
    FLTE: np.ndarray
    FLTD: np.ndarray
    SMAX: np.ndarray
    TCONST: np.ndarray
    RMAX_POS: np.ndarray
    RMAX_NEG: np.ndarray
    D_FF: np.ndarray = field(default_factory=lambda: np.zeros(1))
    PDMX: np.ndarray = field(default_factory=lambda: np.zeros(1))

    @classmethod
    def from_params(cls, params: dict, axis: str, n: int = 1) -> "Gains":
        """Read RLL_RATE_* / RLL2SRV_* or PTCH_RATE_* / PTCH2SRV_* from a parameter dict."""
        if axis == "roll":
            rate, srv = "RLL_RATE_", "RLL2SRV_"
            rmax_pos = rmax_neg = params.get(srv + "RMAX", 0.0)
        elif axis == "pitch":
            rate, srv = "PTCH_RATE_", "PTCH2SRV_"
            rmax_pos = params.get(srv + "RMAX_UP", 0.0)
            rmax_neg = params.get(srv + "RMAX_DN", 0.0)
        else:
            raise ValueError(axis)

        def a(v: float) -> np.ndarray:
            return np.full(n, float(v))
        return cls(P=a(params[rate + "P"]), I=a(params[rate + "I"]), D=a(params[rate + "D"]),
                   FF=a(params[rate + "FF"]), IMAX=a(params[rate + "IMAX"]),
                   FLTT=a(params.get(rate + "FLTT", 0.0)), FLTE=a(params.get(rate + "FLTE", 0.0)),
                   FLTD=a(params.get(rate + "FLTD", 0.0)), SMAX=a(params.get(rate + "SMAX", 0.0)),
                   TCONST=a(params[srv + "TCONST"]), RMAX_POS=a(rmax_pos), RMAX_NEG=a(rmax_neg),
                   D_FF=a(params.get(rate + "D_FF", 0.0)), PDMX=a(params.get(rate + "PDMX", 0.0)))


class AcPid:
    """AC_PID::update_all and update_i (notch filters not used on these aircraft)."""

    def __init__(self, g: Gains, n: int) -> None:
        self.g = g
        self.reset_filter = True
        self.target = np.zeros(n)
        self.error = np.zeros(n)
        self.derivative = np.zeros(n)
        self.target_derivative = np.zeros(n)
        self.integrator = np.zeros(n)
        self.I_info = np.zeros(n)
        self.P = np.zeros(n)          # AC_PID _pid_info.P / D from the previous call
        self.D = np.zeros(n)
        self.FF = np.zeros(n)
        self.DFF = np.zeros(n)
        self.Dmod = np.ones(n)
        self.slew = SlewLimiter(n, g.SMAX)

    def update_all(self, target: np.ndarray | float, measurement: np.ndarray | float, dt: float,
                   limit: np.ndarray, now_ms: int) -> np.ndarray:
        g = self.g
        if self.reset_filter:
            self.reset_filter = False
            self.target = np.array(target, dtype=float) * np.ones_like(self.target)
            self.error = self.target - measurement
            self.derivative = np.zeros_like(self.target)
            self.target_derivative = np.zeros_like(self.target)
        else:
            target_last = self.target
            self.target = self.target + lowpass_alpha(dt, g.FLTT) * (target - self.target)
            error_last = self.error
            error = self.target - measurement
            self.error = self.error + lowpass_alpha(dt, g.FLTE) * (error - self.error)
            if dt > 0:
                derivative = (self.error - error_last) / dt
                self.derivative = self.derivative + lowpass_alpha(dt, g.FLTD) * (derivative - self.derivative)
                self.target_derivative = (self.target - target_last) / dt
        self._update_i(dt, limit)
        p_out = self.error * g.P
        d_out = self.derivative * g.D
        self.Dmod = self.slew.modifier((self.P + self.D) * SLEW_LIMIT_SCALE, dt, now_ms)
        p_out = p_out * self.Dmod
        d_out = d_out * self.Dmod
        pd = np.abs(p_out + d_out)
        scale = np.where((g.PDMX > 0) & (pd > g.PDMX), g.PDMX / np.maximum(pd, 1e-12), 1.0)
        p_out = p_out * scale
        d_out = d_out * scale
        self.P, self.D = p_out, d_out
        self.FF = self.target * g.FF
        self.DFF = self.target_derivative * g.D_FF
        return p_out + d_out + self.integrator

    def _update_i(self, dt: float, limit: np.ndarray) -> None:
        g = self.g
        allowed = (~limit) | ((self.integrator > 0) & (self.error < 0)) | ((self.integrator < 0) & (self.error > 0))
        grow = (g.I != 0) & (dt > 0) & allowed
        new = np.clip(self.integrator + self.error * g.I * dt, -g.IMAX, g.IMAX)
        self.integrator = np.where(g.I != 0, np.where(grow, new, self.integrator), 0.0)
        # _pid_info.I keeps this value even if the caller then restores the integrator
        self.I_info = self.integrator.copy()

    def set_integrator(self, value: np.ndarray) -> None:
        self.integrator = np.clip(value, -self.g.IMAX, self.g.IMAX)


@dataclass
class PidInfo:
    """Controller terms in the units logged in PIDR/PIDP."""
    target: np.ndarray
    actual: np.ndarray
    FF: np.ndarray
    P: np.ndarray
    I: np.ndarray
    D: np.ndarray
    DFF: np.ndarray
    Dmod: np.ndarray
    out: np.ndarray


class RateController:
    """AP_RollController / AP_PitchController rate loop and angle loop."""

    def __init__(self, axis: str, g: Gains, n: int, airspeed_min: float, airspeed_max: float,
                 roll_limit_deg: float = 45.0, roll_ff: float = 1.0) -> None:
        if axis not in ("roll", "pitch"):
            raise ValueError(axis)
        self.axis = axis
        self.g = g
        self.pid = AcPid(g, n)
        self.last_out = np.zeros(n)
        self.airspeed_min = airspeed_min
        self.airspeed_max = airspeed_max
        self.roll_limit_deg = roll_limit_deg
        self.roll_ff = roll_ff        # PTCH2SRV_RLL
        self.info: PidInfo | None = None

    def rate_out(self, desired_rate_dps: np.ndarray | float, gyro_rads: np.ndarray | float,
                 scaler: np.ndarray | float, eas2tas: float, aspeed: np.ndarray | float, dt: float, now_ms: int,
                 disable_integrator: bool = False) -> np.ndarray:
        """_get_rate_out: returns degrees of notional deflection, clipped to +-45."""
        limit_i = np.abs(self.last_out) >= 45
        if self.axis == "roll":
            underspeed = aspeed <= self.airspeed_min
        else:
            underspeed = aspeed <= 0.5 * self.airspeed_min
        limit_i = limit_i | underspeed
        old_i = self.pid.integrator.copy()
        s2 = scaler * scaler
        self.pid.update_all(np.radians(desired_rate_dps) * s2, gyro_rads * s2, dt, limit_i, now_ms)
        self.pid.set_integrator(np.where(underspeed, old_i, self.pid.integrator))
        ff = DEG * (self.pid.FF + self.pid.DFF) / (scaler * eas2tas)
        if disable_integrator:
            self.pid.integrator = np.zeros_like(self.pid.integrator)
        # the output uses _pid_info.I from update_i, before any underspeed restore or reset
        p = self.pid.P * DEG
        i = self.pid.I_info * DEG
        d = self.pid.D * DEG
        dff = self.pid.DFF * DEG
        out = ff + p + i + d + dff
        self.last_out = out
        self.info = PidInfo(target=np.asarray(desired_rate_dps, dtype=float), actual=gyro_rads * DEG, FF=ff,
                            P=p, I=i, D=d, DFF=dff, Dmod=self.pid.Dmod, out=out)
        return np.clip(out, -45.0, 45.0)

    def desired_rate(self, angle_err_deg: np.ndarray, roll_rad: np.ndarray | float | None = None,
                     pitch_rad: np.ndarray | float | None = None, aspeed: np.ndarray | float | None = None,
                     eas2tas: float = 1.0) -> np.ndarray:
        """get_servo_out up to the rate demand (upright flight only)."""
        g = self.g
        tau = np.maximum(g.TCONST, 0.05)
        rate = angle_err_deg / tau
        if self.axis == "roll":
            pos = g.RMAX_POS > 0
            return np.where(pos, np.clip(rate, -g.RMAX_POS, g.RMAX_POS), rate)
        bank = np.clip(roll_rad, -np.radians(80), np.radians(80))
        v = np.maximum(aspeed * eas2tas, max(self.airspeed_min, 1))
        offset = np.cos(pitch_rad) * np.abs(DEG * (GRAVITY_MSS / v) * np.tan(bank) * np.sin(bank)) * self.roll_ff
        offset = np.where(np.abs(np.degrees(pitch_rad)) > 70, 0.0, offset)
        rate = rate + offset
        rate = np.where((g.RMAX_NEG > 0) & (rate < -g.RMAX_NEG), -g.RMAX_NEG, rate)
        rate = np.where((g.RMAX_POS > 0) & (rate > g.RMAX_POS), g.RMAX_POS, rate)
        roll_deg = np.abs(np.degrees(roll_rad))
        margin = min(self.roll_limit_deg + 5.0, 85.0)
        prop = np.clip((roll_deg - margin) / (90.0 - margin), 0.0, 1.0)
        return rate * np.where((roll_deg > margin) & (np.abs(np.degrees(pitch_rad)) < 70), 1 - prop, 1.0)


def speed_scaler(aspeed: np.ndarray | float, scaling_speed: float, airspeed_min: float,
                 airspeed_max: float) -> np.ndarray:
    """Plane::calc_speed_scaler when an airspeed estimate is available."""
    a_min = max(airspeed_min, MIN_AIRSPEED_MIN)
    scale_min = min(0.5, scaling_speed / (2.0 * airspeed_max))
    scale_max = max(2.0, scaling_speed / (0.7 * a_min))
    aspeed = np.asarray(aspeed, dtype=float)
    s = np.where(aspeed > 0.0001, scaling_speed / np.maximum(aspeed, 0.0001), scale_max)
    return np.clip(s, scale_min, scale_max)
