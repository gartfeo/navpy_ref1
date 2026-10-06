"""Pure result construction for zoom-drive transactions."""

from __future__ import annotations

from navpy.modules.vision.target_zoom_types import (
    ZoomCommandPlan,
    ZoomDriveOutcome,
    ZoomTrackResult,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


def zoom_result(
    state: ZoomTrackingState,
    reason: str,
    *,
    size_px: float,
    target_pixels: float,
    current_zoom: float | None,
    desired_zoom: float | None = None,
    command_zoom: float | None = None,
    at_max_zoom: bool = False,
    transition_pending: bool = False,
) -> ZoomTrackResult:
    return ZoomTrackResult(
        state=state,
        has_target=True,
        size_px=size_px,
        target_pixels=target_pixels,
        reason=reason,
        current_zoom=current_zoom,
        desired_zoom=desired_zoom,
        command_zoom=command_zoom,
        at_max_zoom=at_max_zoom,
        transition_pending=transition_pending,
    )


def result_from_plan(
    plan: ZoomCommandPlan,
    size_px: float,
    target_pixels: float,
) -> ZoomTrackResult:
    return zoom_result(
        plan.direction,
        plan.reason,
        size_px=size_px,
        target_pixels=target_pixels,
        current_zoom=plan.current_zoom,
        desired_zoom=plan.desired_zoom,
        command_zoom=plan.command_zoom,
        at_max_zoom=plan.at_max_zoom,
    )


def failed_outcome(
    size_px: float,
    target_pixels: float,
    current_zoom: float | None,
) -> ZoomDriveOutcome:
    return ZoomDriveOutcome(
        zoom_result(
            ZoomTrackingState.HOLDING,
            "actuator-error",
            size_px=size_px,
            target_pixels=target_pixels,
            current_zoom=current_zoom,
        ),
        False,
    )


__all__ = ["failed_outcome", "result_from_plan", "zoom_result"]
