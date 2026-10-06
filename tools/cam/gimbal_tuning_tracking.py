"""Hardware-safe ownership adapter for the production rate tracker."""

from __future__ import annotations

from navpy.modules.vision.gimbal_rate_ports import (
    GimbalRateActuator,
    GimbalRateLog,
)
from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTracker
from navpy.modules.vision.gimbal_rate_types import (
    GimbalRateTrackerConfig,
    GimbalRateUpdate,
    GimbalTrackResult,
)
from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample


class TuningTracker:
    """Pair tracker state with the zero-rate action required on loss."""

    def __init__(
        self,
        actuator: GimbalRateActuator,
        logger: GimbalRateLog,
        config: GimbalRateTrackerConfig,
    ) -> None:
        self._actuator = actuator
        self._tracker = GimbalRateTracker(actuator, logger, config)
        self._loss_handled = True

    @property
    def last_result(self) -> GimbalTrackResult:
        return self._tracker.last_result

    def update(self, sample: GimbalAngularSample) -> GimbalRateUpdate:
        update = self._tracker.update(sample)
        if update.result.has_target:
            self._loss_handled = False
        return update

    def lose_target(self) -> GimbalTrackResult:
        if not self._loss_handled:
            self._command_zero()
            self._loss_handled = True
        return self._tracker.last_result

    def stop(self) -> GimbalTrackResult:
        self._command_zero()
        self._loss_handled = True
        return self._tracker.last_result

    def _command_zero(self) -> None:
        try:
            self._actuator.set_rate(0.0, 0.0)
        finally:
            self._tracker.reset()


def build_tuning_tracker(
    actuator: GimbalRateActuator,
    logger: GimbalRateLog,
    max_rate: float,
) -> TuningTracker:
    return TuningTracker(
        actuator,
        logger,
        GimbalRateTrackerConfig(max_rate=max_rate),
    )


__all__ = ["TuningTracker", "build_tuning_tracker"]
