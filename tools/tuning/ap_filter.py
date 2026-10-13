"""Python port of the ArduPilot filters used by the roll and pitch rate controllers.

Ported from the ArduPilot-4.6 branch (see ap_controller.py for the controllers):

- libraries/Filter/SlewLimiter.cpp             (modifier)
- libraries/AP_Math/AP_Math.cpp                (calc_lowpass_alpha_dt)

Every state is a numpy array over a batch of N gain sets, like ap_controller.py.
"""
from __future__ import annotations

import numpy as np

SLEW_WINDOW_MS = 300            # SlewLimiter.cpp WINDOW_MS
SLEW_MODIFIER_GAIN = 1.5        # SlewLimiter.cpp MODIFIER_GAIN
SLEW_DERIVATIVE_CUTOFF_HZ = 25.0
SLEW_N_EVENTS = 2               # SlewLimiter.h SLEWLIMITER_N_EVENTS
SLEW_RATE_TAU = 1.0             # AC_PID default srtau in AP_Roll/PitchController.h


def lowpass_alpha(dt: float, cutoff_hz: np.ndarray | float) -> np.ndarray:
    """calc_lowpass_alpha_dt: 1 when the cutoff is zero (filter disabled)."""
    cutoff_hz = np.asarray(cutoff_hz, dtype=float)
    rc = np.divide(1.0, 2 * np.pi * cutoff_hz, out=np.zeros_like(cutoff_hz), where=cutoff_hz > 0)
    return np.where(cutoff_hz > 0, dt / (dt + rc), 1.0)


class LowPass:
    """LowPassFilterFloat with a variable dt."""

    def __init__(self, n: int, cutoff_hz: float) -> None:
        self.cutoff_hz = cutoff_hz
        self.out = np.zeros(n)

    def apply(self, sample: np.ndarray, dt: float) -> np.ndarray:
        self.out = self.out + (sample - self.out) * lowpass_alpha(dt, self.cutoff_hz)
        return self.out


class SlewLimiter:
    """SlewLimiter::modifier: gain reduction for P+D when the actuator slews too fast."""

    def __init__(self, n: int, smax: np.ndarray | float, tau: float = SLEW_RATE_TAU) -> None:
        self.smax = np.broadcast_to(np.asarray(smax, dtype=float), (n,)).copy()
        self.tau = tau
        self.filter = LowPass(n, SLEW_DERIVATIVE_CUTOFF_HZ)
        self.last_sample = np.zeros(n)
        self.max_pos = np.zeros(n)
        self.max_neg = np.zeros(n)
        self.max_pos_ms = np.zeros(n, dtype=np.int64)
        self.max_neg_ms = np.zeros(n, dtype=np.int64)
        self.output_slew_rate = np.zeros(n)
        self.modifier_slew_rate = np.zeros(n)
        self.pos_event_ms = np.zeros((n, SLEW_N_EVENTS), dtype=np.int64)
        self.neg_event_ms = np.zeros((n, SLEW_N_EVENTS), dtype=np.int64)
        self.pos_index = np.zeros(n, dtype=np.int64)
        self.neg_index = np.zeros(n, dtype=np.int64)
        self.pos_stored = np.zeros(n, dtype=bool)
        self.neg_stored = np.zeros(n, dtype=bool)
        self._rows = np.arange(n)

    def modifier(self, sample: np.ndarray, dt: float, now_ms: int) -> np.ndarray:
        if dt <= 0:
            return np.ones_like(self.smax)
        slew_rate = self.filter.apply((sample - self.last_sample) / dt, dt)
        self.last_sample = np.array(sample, dtype=float)
        decay_alpha = min(dt, self.tau) / self.tau
        attack_alpha = min(2.0 * decay_alpha, 1.0)

        new_pos = slew_rate > self.max_pos
        stale_pos = (now_ms - self.max_pos_ms) > SLEW_WINDOW_MS
        self.max_pos = np.where(new_pos, slew_rate,
                                np.where(stale_pos, self.max_pos * (1.0 - decay_alpha), self.max_pos))
        self.max_pos_ms = np.where(new_pos, now_ms, self.max_pos_ms)
        new_neg = -slew_rate > self.max_neg
        stale_neg = (now_ms - self.max_neg_ms) > SLEW_WINDOW_MS
        self.max_neg = np.where(new_neg, -slew_rate,
                                np.where(stale_neg, self.max_neg * (1.0 - decay_alpha), self.max_neg))
        self.max_neg_ms = np.where(new_neg, now_ms, self.max_neg_ms)

        raw = 0.5 * (self.max_pos + self.max_neg)
        self.output_slew_rate = (1.0 - attack_alpha) * self.output_slew_rate + attack_alpha * raw
        self.output_slew_rate = np.minimum(self.output_slew_rate, raw)

        active = self.smax > 0
        smax = np.where(active, self.smax, 1.0)
        limited_raw = 0.5 * (np.minimum(self.max_pos, 10.0 * smax) + np.minimum(self.max_neg, 10.0 * smax))

        pos_event = active & ~self.pos_stored & (slew_rate > smax)
        if pos_event.any():
            idx = np.where(self.pos_index >= SLEW_N_EVENTS, 0, self.pos_index)
            rows = self._rows[pos_event]
            self.pos_event_ms[rows, idx[pos_event]] = now_ms
            self.pos_index = np.where(pos_event, idx + 1, self.pos_index)
            self.pos_stored = self.pos_stored | pos_event
            self.neg_stored = self.neg_stored & ~pos_event
        neg_event = active & ~self.neg_stored & (-slew_rate > smax)
        if neg_event.any():
            idx = np.where(self.neg_index >= SLEW_N_EVENTS, 0, self.neg_index)
            rows = self._rows[neg_event]
            self.neg_event_ms[rows, idx[neg_event]] = now_ms
            self.neg_index = np.where(neg_event, idx + 1, self.neg_index)
            self.neg_stored = self.neg_stored | neg_event
            self.pos_stored = self.pos_stored & ~neg_event

        oldest = np.minimum(np.minimum(self.pos_event_ms.min(axis=1), self.neg_event_ms.min(axis=1)), now_ms)
        modifier_input = limited_raw
        age = now_ms - oldest - (SLEW_N_EVENTS + 1) * SLEW_WINDOW_MS
        modifier_input = np.where(age > 0, modifier_input * np.exp(-0.001 * age / self.tau), modifier_input)
        self.modifier_slew_rate = (1.0 - attack_alpha) * self.modifier_slew_rate + attack_alpha * modifier_input
        self.modifier_slew_rate = np.minimum(self.modifier_slew_rate, modifier_input)
        mod = np.where(self.modifier_slew_rate > smax,
                       smax / (smax + SLEW_MODIFIER_GAIN * (self.modifier_slew_rate - smax)), 1.0)
        return np.where(active, mod, 1.0)
