"""Two-sample transition gate for the continuous zoom decision table."""

from __future__ import annotations

from navpy.modules.vision.continuous_zoom_rules import (
    ARM_OUT,
    ARM_SIZE,
    CLEAR_OUT,
    CLEAR_SIZE,
    ContinuousRuleInput,
    TARGET_BAND_RATIO,
    evaluate_continuous_zoom,
)
from navpy.modules.vision.target_zoom_types import ContinuousZoomDecision
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


class ContinuousZoomPolicy:
    """Confirm every state/latch transition on two independent samples."""

    def __init__(self) -> None:
        self._size_satisfied = False
        self._out_latched = False
        self._pending: tuple[object, ...] | None = None
        self._ready: tuple[tuple[object, ...], frozenset[str]] | None = None

    def reset(self) -> None:
        self._size_satisfied = False
        self._out_latched = False
        self._pending = None
        self._ready = None

    @property
    def transition_pending(self) -> bool:
        return self._pending is not None

    def decide(
        self,
        *,
        size_px: float,
        target_pixels: float,
        containment_violation_scale: float | None,
        zoom_in_reserve_scale: float | None,
        at_min_zoom: bool,
        at_max_zoom: bool,
        active_direction: ZoomTrackingState | None,
        sample_advanced: bool,
        optical_fresh: bool,
        centered: bool,
        hold_on_decenter: bool = False,
    ) -> ContinuousZoomDecision:
        decision = self.plan(
            size_px=size_px,
            target_pixels=target_pixels,
            containment_violation_scale=containment_violation_scale,
            zoom_in_reserve_scale=zoom_in_reserve_scale,
            at_min_zoom=at_min_zoom,
            at_max_zoom=at_max_zoom,
            active_direction=active_direction,
            sample_advanced=sample_advanced,
            optical_fresh=optical_fresh,
            centered=centered,
            hold_on_decenter=hold_on_decenter,
        )
        self.commit()
        return decision

    def plan(
        self,
        *,
        size_px: float,
        target_pixels: float,
        containment_violation_scale: float | None,
        zoom_in_reserve_scale: float | None,
        at_min_zoom: bool,
        at_max_zoom: bool,
        active_direction: ZoomTrackingState | None,
        sample_advanced: bool,
        optical_fresh: bool,
        centered: bool,
        hold_on_decenter: bool = False,
    ) -> ContinuousZoomDecision:
        self._ready = None
        result = evaluate_continuous_zoom(
            ContinuousRuleInput(
                size_px=size_px,
                target_pixels=target_pixels,
                containment_scale=containment_violation_scale,
                reserve_scale=zoom_in_reserve_scale,
                at_min_zoom=at_min_zoom,
                at_max_zoom=at_max_zoom,
                active_direction=active_direction,
                centered=centered,
                hold_on_decenter=hold_on_decenter,
                size_satisfied=self._size_satisfied,
                out_latched=self._out_latched,
            )
        )
        current = active_direction or ZoomTrackingState.HOLDING
        changes = result.decision.state is not current or bool(result.intents)
        if not changes:
            if sample_advanced and optical_fresh:
                self._pending = None
            return result.decision
        if not sample_advanced or not optical_fresh:
            return self._continuation(active_direction, "settling")
        key = (
            result.decision.state,
            result.decision.reason,
            result.decision.at_max_zoom,
            result.intents,
        )
        if self._pending != key:
            self._pending = key
            return self._continuation(active_direction, "confirming")
        self._ready = (key, result.intents)
        return result.decision

    def commit(self) -> None:
        if self._ready is None:
            return
        _key, intents = self._ready
        self._pending = None
        self._ready = None
        self._apply(intents)

    def reject(self) -> None:
        self._ready = None

    @staticmethod
    def _continuation(
        active_direction: ZoomTrackingState | None,
        reason: str,
    ) -> ContinuousZoomDecision:
        return ContinuousZoomDecision(
            active_direction or ZoomTrackingState.HOLDING,
            reason,
        )

    def _apply(self, intents: frozenset[str]) -> None:
        if ARM_SIZE in intents:
            self._size_satisfied = True
        if CLEAR_SIZE in intents:
            self._size_satisfied = False
        if ARM_OUT in intents:
            self._out_latched = True
        if CLEAR_OUT in intents:
            self._out_latched = False


__all__ = [
    "ContinuousZoomDecision",
    "ContinuousZoomPolicy",
    "TARGET_BAND_RATIO",
]
