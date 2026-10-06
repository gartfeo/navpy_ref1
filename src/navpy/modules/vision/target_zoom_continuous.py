"""Composition of one continuous-zoom detection tick."""

from __future__ import annotations

from navpy.modules.vision.continuous_zoom_policy import ContinuousZoomPolicy
from navpy.modules.vision.target_zoom_drive import ZoomDrive
from navpy.modules.vision.target_zoom_ports import ZoomCapabilityReader
from navpy.modules.vision.target_zoom_readback import ZoomReadbackState
from navpy.modules.vision.target_zoom_types import (
    ZoomCapabilities,
    ZoomObservation,
    ZoomTrackResult,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


class ContinuousZoomTick:
    def __init__(
        self,
        capabilities: ZoomCapabilityReader,
        readback: ZoomReadbackState,
        drive: ZoomDrive,
        policy: ContinuousZoomPolicy,
    ) -> None:
        self._capabilities = capabilities
        self._readback = readback
        self._drive = drive
        self._policy = policy

    def reset(self) -> None:
        self._policy.reset()

    def update(
        self,
        observation: ZoomObservation,
        capabilities: ZoomCapabilities | None = None,
    ) -> ZoomTrackResult:
        readback = self._readback.capture()
        if readback.fresh.invalid:
            return ZoomTrackResult(
                ZoomTrackingState.HOLDING,
                True,
                size_px=observation.size_px,
                target_pixels=observation.target_pixels,
                reason="readback",
            )
        current = readback.fresh.value
        capabilities = capabilities or self._capabilities.capabilities
        decision = self._policy.plan(
            size_px=observation.size_px,
            target_pixels=observation.target_pixels,
            containment_violation_scale=observation.containment_scale,
            zoom_in_reserve_scale=observation.reserve_scale,
            at_min_zoom=(
                current is not None and current <= capabilities.minimum
            ),
            at_max_zoom=(
                current is not None and current >= capabilities.maximum
            ),
            active_direction=self._drive.continuous_direction,
            sample_advanced=readback.sample_advanced,
            optical_fresh=observation.optical_fresh,
            centered=observation.centered,
            hold_on_decenter=observation.hold_on_decenter,
        )
        pending = (
            self._policy.transition_pending
            and decision.reason in {"confirming", "settling"}
        )
        outcome = self._drive.apply_continuous(
            decision,
            size_px=observation.size_px,
            target_pixels=observation.target_pixels,
            current_zoom=current,
            sample_id=readback.sample_id,
            transition_pending=pending,
        )
        if outcome.committed:
            self._policy.commit()
        else:
            self._policy.reject()
        return outcome.result


__all__ = ["ContinuousZoomTick"]
