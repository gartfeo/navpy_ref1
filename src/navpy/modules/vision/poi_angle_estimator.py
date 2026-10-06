"""Transactional constant-velocity estimator for angular tracking error.

POI denotes the selected visual reference (a delivery reference in the
delivery scenario), not an authenticated recipient or world trajectory.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PoiAngleEstimatorConfig:
    measurement_sigma: float = 0.02
    accel_sigma: float = 1.5
    initial_angle_sigma: float = 0.08
    initial_rate_sigma: float = 0.5
    max_step_dt: float = 0.25
    robust_nis_knee: float = 9.0
    max_abs_rate: float | None = None

    def __post_init__(self) -> None:
        rate = self.max_abs_rate
        if rate is not None and not (math.isfinite(rate) and rate >= 0.0):
            raise ValueError(
                "max_abs_rate must be None or a finite value >= 0 "
                f"(got {rate!r})"
            )
        if not math.isfinite(self.max_step_dt) or self.max_step_dt <= 0.0:
            raise ValueError("max_step_dt must be finite and > 0")


@dataclass(frozen=True)
class PoiAngleEstimate:
    timestamp_s: float
    yaw_rad: float
    pitch_rad: float
    yaw_rate_rad_s: float
    pitch_rate_rad_s: float
    yaw_sigma_rad: float
    pitch_sigma_rad: float


@dataclass(frozen=True)
class PoiAnglePlan:
    estimate: PoiAngleEstimate
    _state: np.ndarray
    _covariance: np.ndarray


@dataclass(frozen=True)
class _FilterState:
    vector: np.ndarray
    covariance: np.ndarray
    timestamp_s: float


class PoiAngleEstimator:
    """Preview estimator changes and commit only after actuation succeeds."""

    def __init__(self, config: PoiAngleEstimatorConfig | None = None) -> None:
        self._config = config or PoiAngleEstimatorConfig()
        self._state: _FilterState | None = None

    @property
    def snapshot(self) -> PoiAngleEstimate | None:
        state = self._state
        return None if state is None else _snapshot(state)

    def reset(self) -> None:
        self._state = None

    def preview_update(
        self,
        yaw_rad: float,
        pitch_rad: float,
        timestamp_s: float,
        *,
        reset: bool = False,
    ) -> PoiAnglePlan | None:
        timestamp_s = float(timestamp_s)
        state = None if reset else self._state
        if state is not None and timestamp_s <= state.timestamp_s:
            return None
        if state is None:
            return self._initial_plan(yaw_rad, pitch_rad, timestamp_s)
        predicted = _predict_to(state, timestamp_s, self._config)
        matrix = np.array(
            [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
            dtype=np.float64,
        )
        measurement = np.array([yaw_rad, pitch_rad], dtype=np.float64)
        noise_value = self._config.measurement_sigma ** 2
        noise = np.diag([noise_value, noise_value]).astype(np.float64)
        innovation = measurement - matrix @ predicted.vector
        covariance = matrix @ predicted.covariance @ matrix.T + noise
        knee = self._config.robust_nis_knee
        if knee > 0.0:
            nis = float(innovation @ np.linalg.solve(covariance, innovation))
            if nis > knee:
                noise *= nis / knee
                covariance = matrix @ predicted.covariance @ matrix.T + noise
        gain = np.linalg.solve(
            covariance.T,
            (predicted.covariance @ matrix.T).T,
        ).T
        vector = predicted.vector + gain @ innovation
        max_rate = self._config.max_abs_rate
        if max_rate is not None:
            np.clip(vector[2:4], -max_rate, max_rate, out=vector[2:4])
        identity = np.eye(4, dtype=np.float64)
        updated = _FilterState(
            vector,
            (identity - gain @ matrix) @ predicted.covariance,
            timestamp_s,
        )
        return _plan(updated)

    def preview_predict(self, timestamp_s: float) -> PoiAnglePlan | None:
        state = self._state
        if state is None or float(timestamp_s) <= state.timestamp_s:
            return None
        return _plan(_predict_to(state, float(timestamp_s), self._config))

    def commit(self, plan: PoiAnglePlan) -> None:
        self._state = _FilterState(
            plan._state.copy(),
            plan._covariance.copy(),
            plan.estimate.timestamp_s,
        )

    def _initial_plan(
        self,
        yaw_rad: float,
        pitch_rad: float,
        timestamp_s: float,
    ) -> PoiAnglePlan:
        angle_variance = self._config.initial_angle_sigma ** 2
        rate_variance = self._config.initial_rate_sigma ** 2
        state = _FilterState(
            np.array([yaw_rad, pitch_rad, 0.0, 0.0], dtype=np.float64),
            np.diag([
                angle_variance,
                angle_variance,
                rate_variance,
                rate_variance,
            ]).astype(np.float64),
            timestamp_s,
        )
        return _plan(state)


def project_estimate(
    estimate: PoiAngleEstimate,
    lead_time_s: float,
) -> tuple[float, float]:
    lead_s = max(0.0, float(lead_time_s))
    return (
        estimate.yaw_rad + estimate.yaw_rate_rad_s * lead_s,
        estimate.pitch_rad + estimate.pitch_rate_rad_s * lead_s,
    )


def _predict_to(
    state: _FilterState,
    timestamp_s: float,
    config: PoiAngleEstimatorConfig,
) -> _FilterState:
    vector = state.vector.copy()
    covariance = state.covariance.copy()
    remaining_s = timestamp_s - state.timestamp_s
    while remaining_s > 0.0:
        dt_s = min(remaining_s, config.max_step_dt)
        transition = np.array(
            [
                [1.0, 0.0, dt_s, 0.0],
                [0.0, 1.0, 0.0, dt_s],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        process = _process_noise(dt_s, config.accel_sigma)
        vector = transition @ vector
        covariance = transition @ covariance @ transition.T + process
        remaining_s -= dt_s
    return _FilterState(vector, covariance, timestamp_s)


def _process_noise(dt_s: float, accel_sigma: float) -> np.ndarray:
    axis = accel_sigma ** 2 * np.array(
        [
            [0.25 * dt_s ** 4, 0.5 * dt_s ** 3],
            [0.5 * dt_s ** 3, dt_s ** 2],
        ],
        dtype=np.float64,
    )
    noise = np.zeros((4, 4), dtype=np.float64)
    noise[np.ix_([0, 2], [0, 2])] = axis
    noise[np.ix_([1, 3], [1, 3])] = axis
    return noise


def _snapshot(state: _FilterState) -> PoiAngleEstimate:
    return PoiAngleEstimate(
        timestamp_s=state.timestamp_s,
        yaw_rad=float(state.vector[0]),
        pitch_rad=float(state.vector[1]),
        yaw_rate_rad_s=float(state.vector[2]),
        pitch_rate_rad_s=float(state.vector[3]),
        yaw_sigma_rad=float(math.sqrt(max(state.covariance[0, 0], 0.0))),
        pitch_sigma_rad=float(math.sqrt(max(state.covariance[1, 1], 0.0))),
    )


def _plan(state: _FilterState) -> PoiAnglePlan:
    return PoiAnglePlan(
        _snapshot(state),
        state.vector.copy(),
        state.covariance.copy(),
    )


__all__ = [
    "PoiAngleEstimate",
    "PoiAngleEstimator",
    "PoiAngleEstimatorConfig",
    "PoiAnglePlan",
    "project_estimate",
]
