"""Confirmation timing, zoom, cooldown, and bounded re-ask policies."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.nav_constants import (
    CONFIRM_DWELL_MARGIN_SEC,
    CONFIRM_MAX_DWELL_SEC,
    CONFIRM_REACQUIRE_ABORT_SEC,
    MAX_CONFIRM_GATE_RESETS,
    TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC,
)
from navpy.modules.nav.nav_state import ConfirmGateState, GeoHoldState
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.nav.target_retry import TargetRetryState
from navpy.modules.vision.detector_ports import ZoomControlPort
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class ConfirmationTimingPolicy:
    """Own raw confirmation clocks and zoom re-acquisition budgets."""

    def __init__(
        self,
        state: ConfirmGateState,
        geo_hold: GeoHoldState,
        args: NavArgs,
        zoom: ZoomControlPort,
        confirmation_manager: ConfirmationManager,
        logger: ILogger,
        decision_clock_s: Callable[[], float],
    ) -> None:
        self._state = state
        self._geo_hold = geo_hold
        self._args = args
        self._zoom = zoom
        self._confirmation_manager = confirmation_manager
        self._logger = logger
        self._clock_s = decision_clock_s

    def gate_timed_out(self) -> bool:
        if self._state.entered_at is None:
            return False
        return (
            self._clock_s() - self._state.entered_at
            >= self._args.confirm_gate_timeout_sec
        )

    def update_loss_tracking(self, present: bool) -> None:
        if present:
            self._state.loss_started_at = None
        elif self._state.loss_started_at is None:
            self._state.loss_started_at = self._clock_s()

    def max_dwell_sec(self) -> float:
        wait_s = self._args.confirm_wait_time_sec or 0.0
        return max(
            CONFIRM_MAX_DWELL_SEC,
            wait_s + CONFIRM_DWELL_MARGIN_SEC,
        )

    def review_dwell_s(self, wall_s: Callable[[], float]) -> float:
        if self._state.review_started_at is None:
            self._state.review_started_at = wall_s()
        return wall_s() - self._state.review_started_at

    def loss_elapsed_s(self) -> float:
        if self._state.loss_started_at is None:
            return 0.0
        return self._clock_s() - self._state.loss_started_at

    def reacquire_timed_out(self) -> bool:
        if self._state.loss_started_at is None:
            return False
        return self.loss_elapsed_s() >= self.reacquire_bound_s()

    def reacquire_bound_s(self) -> float:
        return (
            TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC
            if self._geo_hold.active
            else CONFIRM_REACQUIRE_ABORT_SEC
        )

    def reacquire_bound_label(self) -> str:
        return "geo-hold" if self._geo_hold.active else "pre-request"

    def active_zoom_result(
        self,
        target: DetectedObject | None = None,
    ) -> ZoomTrackResult | None:
        active = target or self._confirmation_manager.active_target
        obj_id = get_target_task_id(active) if active is not None else None
        return self._zoom.get_zoom_result(obj_id)

    def active_zoom_stable(
        self,
        zoom_result: ZoomTrackResult | None,
    ) -> bool:
        if zoom_result is None:
            return self._zoom.is_zoom_stable
        return zoom_result.is_stable

    def refresh_timer_on_reacquire(self) -> None:
        result = self.active_zoom_result()
        if result is None:
            return
        has_target = result.has_target
        if has_target and not self._state.zoom_had_target:
            self._handle_zoom_acquired()
        self._state.zoom_had_target = has_target

    def _handle_zoom_acquired(self) -> None:
        if self._state.zoom_seen_target:
            if self._state.resets_used < MAX_CONFIRM_GATE_RESETS:
                self._state.entered_at = self._clock_s()
                self._state.resets_used += 1
                self._logger.info(
                    "CONFIRM gate timer reset on re-acquire "
                    f"({self._state.resets_used}/{MAX_CONFIRM_GATE_RESETS})",
                    key="nav",
                )
            else:
                self._logger.warning(
                    "CONFIRM gate reset cap reached "
                    f"({MAX_CONFIRM_GATE_RESETS}): subsequent re-acquires "
                    "will not extend the timeout",
                    key="nav",
                    dest=LogStatusDest.DRONE,
                )
        self._state.zoom_seen_target = True


class TargetRetryPolicy:
    """Join the retry state owner to ConfirmationManager status reopening."""

    def __init__(
        self,
        retry: TargetRetryState,
        confirmation_manager: ConfirmationManager,
    ) -> None:
        self._retry = retry
        self._confirmation_manager = confirmation_manager

    def target_cooldown_key(
        self,
        target: DetectedObject,
    ) -> tuple[object, ...] | None:
        return self._retry.identity_key(target)

    def register_target_cooldown(self, target: DetectedObject) -> None:
        self._retry.register_cooldown(target)

    def is_in_target_cooldown(self, target: DetectedObject) -> bool:
        return self._retry.is_cooling_down(target)

    def can_reask(self, target: DetectedObject) -> bool:
        return self._retry.can_reask(target)

    def begin_reask(self, target: DetectedObject) -> None:
        self._retry.begin_reask(target)
        self._confirmation_manager.clear_status(target)

    def clear_reask(self, target_id: int | None) -> None:
        self._retry.clear_reask(target_id)


__all__ = ["ConfirmationTimingPolicy", "TargetRetryPolicy"]
