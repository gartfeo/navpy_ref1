"""Transactional state machine for stabilized-gimbal rate tracking."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, replace

from navpy.modules.vision.gimbal_rate_command import (
    GimbalRateCommand,
    GimbalRateCommandLaw,
)
from navpy.modules.vision.gimbal_rate_ports import GimbalRateActuator, GimbalRateLog
from navpy.modules.vision.gimbal_rate_types import (
    GimbalObservationDisposition,
    GimbalRateTrackerConfig,
    GimbalRateUpdate,
    GimbalTrackResult,
    TrackingState,
    idle_result,
)
from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample
from navpy.modules.vision.target_angle_estimator import (
    TargetAngleEstimator,
    TargetAngleEstimatorConfig,
    TargetAnglePlan,
)


@dataclass(frozen=True)
class _TrackerMemory:
    last_source_timestamp_s: float | None
    updates_since_reset: int
    result: GimbalTrackResult


class GimbalRateTracker:
    """Plan, actuate, then atomically commit estimator and tracker state."""

    def __init__(
        self,
        actuator: GimbalRateActuator,
        logger: GimbalRateLog,
        config: GimbalRateTrackerConfig | None = None,
    ) -> None:
        self._actuator = actuator
        self._logger = logger
        self._config = config or GimbalRateTrackerConfig()
        self._estimator = TargetAngleEstimator(
            _estimator_config(self._config)
        )
        self._law = GimbalRateCommandLaw(self._config)
        self._lock = threading.Lock()
        self._memory = _TrackerMemory(None, 0, idle_result())

    @property
    def state(self) -> TrackingState:
        return self._memory.result.state

    @property
    def is_tracking(self) -> bool:
        return self.state is TrackingState.TRACKING

    @property
    def last_result(self) -> GimbalTrackResult:
        return self._memory.result

    def reset(self) -> None:
        with self._lock:
            self._estimator.reset()
            self._memory = _TrackerMemory(None, 0, idle_result())

    def update(
        self,
        sample: GimbalAngularSample,
        *,
        before_actuation: Callable[[], bool] | None = None,
    ) -> GimbalRateUpdate:
        """Consume one fresh source sample; loss policy lives outside."""
        with self._lock:
            return self._update_sample(sample, before_actuation)

    def _update_sample(
        self,
        sample: GimbalAngularSample,
        before_actuation: Callable[[], bool] | None,
    ) -> GimbalRateUpdate:
        memory = self._memory
        timestamp_s = sample.source_timestamp_s
        if (
            memory.last_source_timestamp_s is not None
            and timestamp_s <= memory.last_source_timestamp_s
        ):
            return GimbalRateUpdate(
                memory.result,
                GimbalObservationDisposition.STALE_NOOP,
            )
        plan = self._estimator.preview_update(
            sample.yaw_error_rad,
            sample.pitch_error_rad,
            timestamp_s,
        )
        if plan is None:
            return GimbalRateUpdate(
                memory.result,
                GimbalObservationDisposition.STALE_NOOP,
            )
        if before_actuation is not None and not before_actuation():
            return GimbalRateUpdate(
                memory.result,
                GimbalObservationDisposition.STALE_NOOP,
            )
        command = self._law.command(plan.estimate)
        self._actuator.set_rate(command.yaw_units, command.pitch_units)
        self._estimator.commit(plan)
        update_count = memory.updates_since_reset + 1
        result = _tracking_result(plan, command, update_count)
        previous = memory.result.state
        self._memory = _TrackerMemory(
            timestamp_s,
            update_count,
            result,
        )
        self._log_tracking(sample, plan, result, previous)
        return GimbalRateUpdate(
            result,
            GimbalObservationDisposition.ACCEPTED,
        )

    def _log_tracking(
        self,
        sample: GimbalAngularSample,
        plan: TargetAnglePlan,
        result: GimbalTrackResult,
        previous: TrackingState,
    ) -> None:
        estimate = plan.estimate
        self._logger.debug(
            "TRACKER: "
            f"meas=({sample.yaw_error_rad:.4f},{sample.pitch_error_rad:.4f}) "
            f"est=({estimate.yaw_rad:.4f},{estimate.pitch_rad:.4f}) "
            f"cmd=({result.yaw_rate:.2f},{result.pitch_rate:.2f})"
        )
        if previous is not TrackingState.TRACKING:
            self._logger.info(
                f"GimbalTracker: {previous.value} -> TRACKING"
            )


def _estimator_config(
    config: GimbalRateTrackerConfig,
) -> TargetAngleEstimatorConfig:
    estimator = config.estimator
    if estimator.max_abs_rate is not None:
        return estimator
    return replace(
        estimator,
        max_abs_rate=(
            math.radians(config.max_slew_dps)
            * config.rate_clamp_slew_multiple
        ),
    )


def _tracking_result(
    plan: TargetAnglePlan,
    command: GimbalRateCommand,
    update_count: int,
) -> GimbalTrackResult:
    estimate = plan.estimate
    return GimbalTrackResult(
        TrackingState.TRACKING,
        True,
        command.projected_yaw_rad,
        command.projected_pitch_rad,
        command.yaw_units,
        command.pitch_units,
        estimate.yaw_rate_rad_s,
        estimate.pitch_rate_rad_s,
        update_count >= 2,
    )


__all__ = [
    "GimbalRateTracker",
    "GimbalRateTrackerConfig",
    "GimbalRateUpdate",
    "GimbalTrackResult",
    "TrackingState",
]
