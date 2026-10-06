"""Public zoom-drive facade over focused hardware transactions."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vision.poi_zoom_drive_strategies import (
    AbsoluteZoomTransaction,
    ContinuousZoomTransaction,
    DiscreteZoomTransaction,
    MinimumZoomTransaction,
    StopZoomTransaction,
    ZoomDriveState,
)
from navpy.modules.vision.poi_zoom_ports import (
    ZoomActuatorPort,
    ZoomCapabilityReader,
    ZoomLogger,
)
from navpy.modules.vision.poi_zoom_readback import ZoomReadbackState
from navpy.modules.vision.poi_zoom_types import (
    ContinuousZoomDecision,
    ZoomCommandPlan,
    ZoomDriveOutcome,
    ZoomStopPlan,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


@dataclass(frozen=True)
class ZoomDriveParts:
    state: ZoomDriveState
    readback: ZoomReadbackState
    minimum: MinimumZoomTransaction
    stop: StopZoomTransaction
    absolute: AbsoluteZoomTransaction
    continuous: ContinuousZoomTransaction
    discrete: DiscreteZoomTransaction


def _build_parts(
    capabilities: ZoomCapabilityReader,
    actuator: ZoomActuatorPort,
    readback: ZoomReadbackState,
    logger: ZoomLogger,
) -> ZoomDriveParts:
    state = ZoomDriveState()
    stop = StopZoomTransaction(actuator, readback, logger, state)
    return ZoomDriveParts(
        state=state,
        readback=readback,
        minimum=MinimumZoomTransaction(capabilities, actuator, readback),
        stop=stop,
        absolute=AbsoluteZoomTransaction(actuator, readback, logger, state, stop),
        continuous=ContinuousZoomTransaction(actuator, readback, logger, state, stop),
        discrete=DiscreteZoomTransaction(actuator, readback, logger),
    )


class ZoomDrive:
    """Apply hardware transactions and expose their committed state."""

    def __init__(
        self,
        capabilities: ZoomCapabilityReader,
        actuator: ZoomActuatorPort,
        readback: ZoomReadbackState,
        logger: ZoomLogger,
    ) -> None:
        self._parts = _build_parts(capabilities, actuator, readback, logger)

    @property
    def continuous_direction(self) -> ZoomTrackingState | None:
        return self._parts.state.continuous_direction

    @property
    def absolute_target(self) -> str | None:
        return self._parts.state.absolute_target

    @absolute_target.setter
    def absolute_target(self, value: str | None) -> None:
        self._parts.state.absolute_target = value

    @property
    def active(self) -> bool:
        return self._parts.state.active

    def clear_without_hold(self) -> None:
        self._parts.state.clear()
        self._parts.readback.clear_command_gate()

    def reset_actuator_source(self) -> None:
        self._parts.state.clear()
        self._parts.readback.reset_source()

    def seek_minimum(self, sample_id: str | None) -> bool:
        return self._parts.minimum.execute(sample_id)

    def prepare_stop(self, *, lifecycle: bool = False) -> ZoomStopPlan | None:
        return self._parts.stop.prepare(lifecycle=lifecycle)

    def commit_stop(
        self,
        plan: ZoomStopPlan,
        *,
        lifecycle: bool = False,
        sample_id: str | None = None,
    ) -> None:
        self._parts.stop.commit(
            plan,
            lifecycle=lifecycle,
            sample_id=sample_id,
        )

    def stop(
        self,
        *,
        reason: str,
        size_px: float | None = None,
        target_pixels: float | None = None,
        current_zoom: float | None = None,
        sample_id: str | None = None,
        lifecycle: bool = False,
    ) -> bool:
        return self._parts.stop.execute(
            reason=reason,
            size_px=size_px,
            target_pixels=target_pixels,
            current_zoom=current_zoom,
            sample_id=sample_id,
            lifecycle=lifecycle,
        )

    def apply_absolute(
        self,
        plan: ZoomCommandPlan,
        *,
        size_px: float,
        target_pixels: float,
        sample_id: str | None = None,
    ) -> ZoomDriveOutcome:
        return self._parts.absolute.execute(
            plan,
            size_px=size_px,
            target_pixels=target_pixels,
            sample_id=sample_id,
        )

    def apply_continuous(
        self,
        decision: ContinuousZoomDecision,
        *,
        size_px: float,
        target_pixels: float,
        current_zoom: float | None,
        sample_id: str | None,
        transition_pending: bool,
    ) -> ZoomDriveOutcome:
        return self._parts.continuous.execute(
            decision,
            size_px=size_px,
            target_pixels=target_pixels,
            current_zoom=current_zoom,
            sample_id=sample_id,
            transition_pending=transition_pending,
        )

    def apply_discrete(
        self,
        plan: ZoomCommandPlan,
        *,
        size_px: float,
        target_pixels: float,
        sample_id: str | None = None,
    ) -> ZoomDriveOutcome:
        return self._parts.discrete.execute(
            plan,
            size_px=size_px,
            target_pixels=target_pixels,
            sample_id=sample_id,
        )


__all__ = ["ZoomDrive"]
