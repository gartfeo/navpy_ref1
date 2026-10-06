"""Pure decision table for one continuous-zoom observation."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vision.poi_zoom_types import ContinuousZoomDecision
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


ARM_SIZE = "arm-size"
CLEAR_SIZE = "clear-size"
ARM_OUT = "arm-out"
CLEAR_OUT = "clear-out"
TARGET_BAND_RATIO = 0.15


@dataclass(frozen=True)
class ContinuousRuleInput:
    size_px: float
    target_pixels: float
    containment_scale: float | None
    reserve_scale: float | None
    at_min_zoom: bool
    at_max_zoom: bool
    active_direction: ZoomTrackingState | None
    centered: bool
    hold_on_decenter: bool
    size_satisfied: bool
    out_latched: bool


@dataclass(frozen=True)
class ContinuousRuleResult:
    decision: ContinuousZoomDecision
    intents: frozenset[str] = frozenset()


def evaluate_continuous_zoom(
    values: ContinuousRuleInput,
) -> ContinuousRuleResult:
    intents: set[str] = set()
    violation = (
        values.containment_scale is not None
        and values.containment_scale < 1.0
    )
    if values.out_latched or violation:
        if values.at_min_zoom:
            if values.out_latched:
                intents.add(CLEAR_OUT)
            return _result(ZoomTrackingState.HOLDING, "frame", intents)
        if violation:
            if not values.out_latched:
                intents.add(ARM_OUT)
            return _result(ZoomTrackingState.ZOOMING_OUT, "frame", intents)
        if (
            values.reserve_scale is not None
            and values.reserve_scale < 1.0 + TARGET_BAND_RATIO
        ):
            return _result(ZoomTrackingState.ZOOMING_OUT, "frame", intents)
        intents.add(CLEAR_OUT)

    if values.size_px >= values.target_pixels:
        if (
            values.active_direction is ZoomTrackingState.ZOOMING_IN
            and not values.size_satisfied
        ):
            intents.add(ARM_SIZE)
        return _result(ZoomTrackingState.HOLDING, "above-minimum", intents)

    if values.at_max_zoom:
        return ContinuousRuleResult(
            ContinuousZoomDecision(
                ZoomTrackingState.HOLDING,
                "max",
                at_max_zoom=True,
            ),
            frozenset(intents),
        )

    if values.reserve_scale is not None:
        if values.active_direction is ZoomTrackingState.ZOOMING_IN:
            if values.reserve_scale <= 1.0:
                return _result(
                    ZoomTrackingState.HOLDING,
                    "frame-limited",
                    intents,
                )
        elif values.reserve_scale < (
            values.target_pixels / values.size_px * (1.0 + TARGET_BAND_RATIO)
        ):
            return _result(
                ZoomTrackingState.HOLDING,
                "frame-limited",
                intents,
            )

    if (
        values.size_satisfied
        and values.size_px >= values.target_pixels * (1.0 - TARGET_BAND_RATIO)
    ):
        return _result(ZoomTrackingState.HOLDING, "in-band", intents)

    if not values.centered and (
        values.hold_on_decenter
        or values.active_direction is not ZoomTrackingState.ZOOMING_IN
    ):
        return _result(ZoomTrackingState.HOLDING, "centering", intents)

    if values.size_satisfied:
        intents.add(CLEAR_SIZE)
    return _result(ZoomTrackingState.ZOOMING_IN, "minimum", intents)


def _result(
    state: ZoomTrackingState,
    reason: str,
    intents: set[str],
) -> ContinuousRuleResult:
    return ContinuousRuleResult(
        ContinuousZoomDecision(state, reason),
        frozenset(intents),
    )


__all__ = [
    "ARM_OUT",
    "ARM_SIZE",
    "CLEAR_OUT",
    "CLEAR_SIZE",
    "TARGET_BAND_RATIO",
    "ContinuousRuleInput",
    "ContinuousRuleResult",
    "evaluate_continuous_zoom",
]
