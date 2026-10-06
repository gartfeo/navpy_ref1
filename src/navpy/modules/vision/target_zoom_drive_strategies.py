"""Focused zoom-hardware transactions sharing explicit drive state."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vision.target_zoom_drive_outcomes import (
    failed_outcome,
    result_from_plan,
    zoom_result,
)
from navpy.modules.vision.target_zoom_ports import (
    ZoomActuatorPort,
    ZoomCapabilityReader,
    ZoomLogger,
)
from navpy.modules.vision.target_zoom_readback import ZoomReadbackState
from navpy.modules.vision.target_zoom_types import (
    ContinuousZoomDecision,
    ZoomCommandPlan,
    ZoomDriveOutcome,
    ZoomStopPlan,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


@dataclass
class ZoomDriveState:
    continuous_direction: ZoomTrackingState | None = None
    absolute_target: str | None = None

    @property
    def active(self) -> bool:
        return self.continuous_direction is not None or self.absolute_target is not None

    def clear(self) -> None:
        self.continuous_direction = None
        self.absolute_target = None


class MinimumZoomTransaction:
    def __init__(
        self,
        capabilities: ZoomCapabilityReader,
        actuator: ZoomActuatorPort,
        readback: ZoomReadbackState,
    ) -> None:
        self._capabilities = capabilities
        self._actuator = actuator
        self._readback = readback

    def execute(self, sample_id: str | None) -> bool:
        capabilities = self._capabilities.capabilities
        if not capabilities.available:
            return False
        command = f"{capabilities.minimum:.6f}".rstrip("0").rstrip(".")
        if capabilities.absolute:
            committed = self._actuator.command_absolute(command)
        elif len(capabilities.levels) >= 2:
            committed = self._actuator.command_discrete(capabilities.levels[0])
        else:
            return not capabilities.supported
        if committed:
            self._readback.record_command(sample_id)
        return committed


class StopZoomTransaction:
    def __init__(
        self,
        actuator: ZoomActuatorPort,
        readback: ZoomReadbackState,
        logger: ZoomLogger,
        state: ZoomDriveState,
    ) -> None:
        self._actuator = actuator
        self._readback = readback
        self._logger = logger
        self._state = state

    def prepare(self, *, lifecycle: bool = False) -> ZoomStopPlan | None:
        was_active = self._state.active
        had_continuous = self._state.continuous_direction is not None
        sample_id: str | None = None
        if had_continuous and lifecycle:
            sample = self._readback.sample()
            if sample.invalid:
                return None
            sample_id = sample.value
        if had_continuous and not self._actuator.hold():
            return None
        return ZoomStopPlan(was_active, had_continuous, sample_id)

    def commit(
        self,
        plan: ZoomStopPlan,
        *,
        lifecycle: bool = False,
        sample_id: str | None = None,
    ) -> None:
        self._state.clear()
        if plan.had_continuous:
            if lifecycle:
                self._readback.record_lifecycle_hold(
                    sample_id if sample_id is not None else plan.sample_id
                )
            else:
                self._readback.record_command(sample_id)

    def execute(
        self,
        *,
        reason: str,
        size_px: float | None = None,
        target_pixels: float | None = None,
        current_zoom: float | None = None,
        sample_id: str | None = None,
        lifecycle: bool = False,
    ) -> bool:
        plan = self.prepare(lifecycle=lifecycle)
        if plan is None:
            return False
        self.commit(plan, lifecycle=lifecycle, sample_id=sample_id)
        if size_px is not None and target_pixels is not None:
            display = "?" if current_zoom is None else f"{current_zoom:.1f}"
            self._logger.info(
                f"Zoom HOLD ({reason}; size={size_px:.0f} "
                f"target={target_pixels:.0f} zoom={display})"
            )
        return True


class AbsoluteZoomTransaction:
    def __init__(
        self,
        actuator: ZoomActuatorPort,
        readback: ZoomReadbackState,
        logger: ZoomLogger,
        state: ZoomDriveState,
        stop: StopZoomTransaction,
    ) -> None:
        self._actuator = actuator
        self._readback = readback
        self._logger = logger
        self._state = state
        self._stop = stop

    def execute(
        self,
        plan: ZoomCommandPlan,
        *,
        size_px: float,
        target_pixels: float,
        sample_id: str | None = None,
    ) -> ZoomDriveOutcome:
        if plan.direction is ZoomTrackingState.HOLDING:
            if not self._stop.execute(
                reason=plan.reason,
                size_px=size_px,
                target_pixels=target_pixels,
                current_zoom=plan.current_zoom,
            ):
                return failed_outcome(size_px, target_pixels, plan.current_zoom)
            return ZoomDriveOutcome(result_from_plan(plan, size_px, target_pixels), True)
        if plan.command_key is None:
            return failed_outcome(size_px, target_pixels, plan.current_zoom)
        if plan.command_key != self._state.absolute_target:
            if not self._actuator.command_absolute(plan.command_key):
                return failed_outcome(size_px, target_pixels, plan.current_zoom)
            self._logger.info(
                f"Zoom SET {plan.direction.value} ({plan.reason}) "
                f"cmd={plan.command_key}"
            )
            self._state.absolute_target = plan.command_key
            self._readback.record_command(sample_id)
        self._state.continuous_direction = None
        return ZoomDriveOutcome(result_from_plan(plan, size_px, target_pixels), True)


class ContinuousZoomTransaction:
    def __init__(
        self,
        actuator: ZoomActuatorPort,
        readback: ZoomReadbackState,
        logger: ZoomLogger,
        state: ZoomDriveState,
        stop: StopZoomTransaction,
    ) -> None:
        self._actuator = actuator
        self._readback = readback
        self._logger = logger
        self._state = state
        self._stop = stop

    def execute(
        self,
        decision: ContinuousZoomDecision,
        *,
        size_px: float,
        target_pixels: float,
        current_zoom: float | None,
        sample_id: str | None,
        transition_pending: bool,
    ) -> ZoomDriveOutcome:
        if decision.state is ZoomTrackingState.HOLDING:
            if not self._stop.execute(
                reason=decision.reason,
                size_px=size_px,
                target_pixels=target_pixels,
                current_zoom=current_zoom,
                sample_id=sample_id,
            ):
                state = self._state.continuous_direction or ZoomTrackingState.HOLDING
                return ZoomDriveOutcome(
                    zoom_result(
                        state,
                        "actuator-error",
                        size_px=size_px,
                        target_pixels=target_pixels,
                        current_zoom=current_zoom,
                        transition_pending=True,
                    ),
                    False,
                )
        elif decision.state is not self._state.continuous_direction:
            if not self._actuator.start_continuous(decision.state):
                return failed_outcome(size_px, target_pixels, current_zoom)
            self._state.continuous_direction = decision.state
            self._state.absolute_target = None
            self._readback.record_command(sample_id)
            label = "IN" if decision.state is ZoomTrackingState.ZOOMING_IN else "OUT"
            display = "?" if current_zoom is None else f"{current_zoom:.1f}"
            self._logger.info(f"Zoom {label} ({decision.reason}; zoom={display})")
        result = zoom_result(
            decision.state,
            decision.reason,
            size_px=size_px,
            target_pixels=target_pixels,
            current_zoom=current_zoom,
            at_max_zoom=decision.at_max_zoom,
            transition_pending=transition_pending,
        )
        return ZoomDriveOutcome(result, True)


class DiscreteZoomTransaction:
    def __init__(
        self,
        actuator: ZoomActuatorPort,
        readback: ZoomReadbackState,
        logger: ZoomLogger,
    ) -> None:
        self._actuator = actuator
        self._readback = readback
        self._logger = logger

    def execute(
        self,
        plan: ZoomCommandPlan,
        *,
        size_px: float,
        target_pixels: float,
        sample_id: str | None = None,
    ) -> ZoomDriveOutcome:
        if plan.command_key is not None:
            if not self._actuator.command_discrete(plan.command_key):
                return failed_outcome(size_px, target_pixels, plan.current_zoom)
            self._logger.info(
                f"Zoom {plan.current_zoom} -> {plan.command_key} "
                f"(size={size_px:.0f} target={target_pixels:.0f})"
            )
            self._readback.record_command(sample_id)
        return ZoomDriveOutcome(result_from_plan(plan, size_px, target_pixels), True)


__all__ = [
    "AbsoluteZoomTransaction",
    "ContinuousZoomTransaction",
    "DiscreteZoomTransaction",
    "MinimumZoomTransaction",
    "StopZoomTransaction",
    "ZoomDriveState",
]
