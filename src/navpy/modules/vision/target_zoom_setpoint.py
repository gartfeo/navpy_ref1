"""Pure absolute and calibrated-level zoom planning."""

from __future__ import annotations

import math

from navpy.modules.vision.continuous_zoom_rules import TARGET_BAND_RATIO
from navpy.modules.vision.target_zoom_types import (
    ZoomCapabilities,
    ZoomCommandPlan,
    ZoomObservation,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


def clamp_zoom(value: float, capabilities: ZoomCapabilities) -> float:
    return max(capabilities.minimum, min(capabilities.maximum, value))


def quantize_zoom(value: float, capabilities: ZoomCapabilities) -> float:
    step = capabilities.command_step
    quantized = math.floor(value / step + 0.5) * step
    return clamp_zoom(quantized, capabilities)


def format_zoom(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".")


def zoom_matches(
    left: float,
    right: float,
    capabilities: ZoomCapabilities,
) -> bool:
    return format_zoom(quantize_zoom(left, capabilities)) == format_zoom(
        quantize_zoom(right, capabilities)
    )


def plan_absolute(
    observation: ZoomObservation,
    current_zoom: float,
    fresh_zoom: float | None,
    capabilities: ZoomCapabilities,
) -> ZoomCommandPlan:
    direction = ZoomTrackingState.ZOOMING_IN
    reason = "minimum"
    desired = current_zoom * observation.target_pixels / observation.size_px
    full_target = desired
    if (
        observation.containment_scale is not None
        and observation.containment_scale < 1.0
    ):
        direction = ZoomTrackingState.ZOOMING_OUT
        reason = "frame"
        desired = current_zoom * observation.containment_scale
        full_target = None
    elif observation.size_px >= observation.target_pixels:
        return ZoomCommandPlan(
            ZoomTrackingState.HOLDING,
            "above-minimum",
            current_zoom=current_zoom,
        )
    elif (
        observation.reserve_scale is not None
        and observation.reserve_scale
        < observation.target_pixels / observation.size_px
    ):
        reason = "frame-limited"
        desired = current_zoom * observation.reserve_scale

    command = quantize_zoom(desired, capabilities)
    at_max = (
        direction is ZoomTrackingState.ZOOMING_IN
        and full_target is not None
        and full_target > capabilities.maximum
        and fresh_zoom is not None
        and zoom_matches(fresh_zoom, capabilities.maximum, capabilities)
    )
    if zoom_matches(command, current_zoom, capabilities):
        return ZoomCommandPlan(
            ZoomTrackingState.HOLDING,
            "max" if at_max else reason,
            current_zoom=current_zoom,
            desired_zoom=full_target or desired,
            command_zoom=command,
            at_max_zoom=at_max,
        )
    return ZoomCommandPlan(
        direction,
        reason,
        current_zoom=current_zoom,
        desired_zoom=full_target or desired,
        command_zoom=command,
        command_key=format_zoom(command),
        at_max_zoom=at_max,
    )


def plan_widen(
    current_zoom: float,
    capabilities: ZoomCapabilities,
) -> ZoomCommandPlan:
    command = quantize_zoom(capabilities.minimum, capabilities)
    state = (
        ZoomTrackingState.HOLDING
        if zoom_matches(command, current_zoom, capabilities)
        else ZoomTrackingState.ZOOMING_OUT
    )
    return ZoomCommandPlan(
        state,
        "nav-widen",
        current_zoom=current_zoom,
        desired_zoom=capabilities.minimum,
        command_zoom=command,
        command_key=None if state is ZoomTrackingState.HOLDING else format_zoom(command),
    )


def plan_discrete(
    observation: ZoomObservation,
    current_zoom: float,
    fresh_zoom: float | None,
    capabilities: ZoomCapabilities,
) -> ZoomCommandPlan:
    direction = _discrete_direction(observation)
    if direction is None:
        return ZoomCommandPlan(
            ZoomTrackingState.HOLDING,
            "in-band",
            current_zoom=current_zoom,
        )
    numeric = tuple(float(level) for level in capabilities.levels)
    current_index = min(
        range(len(numeric)),
        key=lambda index: abs(numeric[index] - current_zoom),
    )
    delta = 1 if direction is ZoomTrackingState.ZOOMING_IN else -1
    desired_index = current_index + delta
    if not 0 <= desired_index < len(numeric):
        at_max = (
            direction is ZoomTrackingState.ZOOMING_IN
            and fresh_zoom is not None
            and zoom_matches(fresh_zoom, capabilities.maximum, capabilities)
        )
        return ZoomCommandPlan(
            ZoomTrackingState.HOLDING,
            "max" if at_max else "limit",
            current_zoom=current_zoom,
            at_max_zoom=at_max,
        )
    command_key = capabilities.levels[desired_index]
    return ZoomCommandPlan(
        direction,
        "minimum" if direction is ZoomTrackingState.ZOOMING_IN else "above-band",
        current_zoom=current_zoom,
        command_zoom=float(command_key),
        command_key=command_key,
    )


def _discrete_direction(
    observation: ZoomObservation,
) -> ZoomTrackingState | None:
    if observation.size_px < observation.target_pixels:
        return ZoomTrackingState.ZOOMING_IN
    if observation.size_px > observation.target_pixels * (1.0 + TARGET_BAND_RATIO):
        return ZoomTrackingState.ZOOMING_OUT
    return None


__all__ = [
    "clamp_zoom",
    "format_zoom",
    "plan_absolute",
    "plan_discrete",
    "plan_widen",
    "quantize_zoom",
    "zoom_matches",
]
