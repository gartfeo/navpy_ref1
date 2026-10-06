"""Outer visual-loss policy for gimbal tracking sessions."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalHardware,
    GimbalLossPolicy,
)
from navpy.modules.navigation.gimbal_neutral_return import GimbalNeutralReturn
from navpy.modules.navigation.gimbal_tracking_constants import MODE_LOCK
from navpy.modules.navigation.gimbal_zoom_control import GimbalZoomController
from navpy.modules.vision.gimbal_rate_types import TrackingState


_COMMAND_ERRORS = (AttributeError, OSError, RuntimeError, TypeError, ValueError)


@dataclass(frozen=True)
class LossRecoveryResult:
    recentered: bool
    holding: bool
    last_track_time: float | None
    pointing_state: TrackingState


class GimbalLossRecovery:
    """Apply timed loss behavior without reading pose or target truth."""

    def __init__(
        self,
        hardware: GimbalHardware,
        policy: GimbalLossPolicy,
        zoom: GimbalZoomController,
        neutral: GimbalNeutralReturn | None,
    ) -> None:
        self._hardware = hardware
        self._policy = policy
        self._zoom = zoom
        self._neutral = neutral

    def prepare_reacquire(self, recentered: bool) -> bool:
        if not recentered:
            return True
        try:
            self._hardware.gimbal.set_motion_mode(MODE_LOCK)
        except _COMMAND_ERRORS as exc:
            self._warn(f"re-acquire LOCK failed: {exc}")
            return False
        self._log("re-acquired after recentre, mode=LOCK")
        return True

    def accept_target(
        self,
        now: float,
        previous_holding: bool,
    ) -> LossRecoveryResult:
        if previous_holding:
            self._log("re-acquired after hold, mode=LOCK")
        return LossRecoveryResult(False, False, now, TrackingState.TRACKING)

    def advance_loss(
        self,
        now: float,
        last_track_time: float | None,
        previous_recentered: bool,
        previous_holding: bool,
    ) -> LossRecoveryResult:
        if last_track_time is None:
            return LossRecoveryResult(
                previous_recentered,
                previous_holding,
                None,
                TrackingState.IDLE,
            )
        elapsed = max(0.0, now - last_track_time)
        holding = self._stop_stale_rate(
            elapsed,
            previous_recentered,
            previous_holding,
        )
        if elapsed < self._policy.hold_sec:
            return LossRecoveryResult(
                previous_recentered,
                holding,
                last_track_time,
                TrackingState.COASTING,
            )
        if elapsed < self._policy.repoint_sec:
            return LossRecoveryResult(
                previous_recentered,
                holding,
                last_track_time,
                TrackingState.HOLDING if holding else TrackingState.COASTING,
            )
        return self._recenter(
            elapsed,
            last_track_time,
            previous_recentered,
            holding,
        )

    def _stop_stale_rate(
        self,
        elapsed: float,
        recentered: bool,
        holding: bool,
    ) -> bool:
        if not holding and not recentered:
            try:
                if self._neutral is None:
                    raise RuntimeError("neutral command capability unavailable")
                self._neutral.hold()
            except _COMMAND_ERRORS as exc:
                self._warn(f"zero-rate hold failed: {exc}")
                return False
            if not self._policy.preserve_zoom_during_loss:
                self._zoom.reset_to_min()
            holding = True
            zoom_note = (
                "zoom kept"
                if self._policy.preserve_zoom_during_loss
                else "zoom reset"
            )
            self._log(f"zero-rate hold (lost {elapsed:.1f}s), {zoom_note}")
        return holding

    def _recenter(
        self,
        elapsed: float,
        last_track_time: float,
        recentered: bool,
        holding: bool,
    ) -> LossRecoveryResult:
        if recentered:
            return LossRecoveryResult(
                True,
                False,
                last_track_time,
                TrackingState.HOLDING,
            )
        try:
            if self._neutral is None:
                raise RuntimeError("neutral command capability unavailable")
            self._neutral.execute()
        except _COMMAND_ERRORS as exc:
            self._warn(f"recentre hardware call failed: {exc}")
            state = TrackingState.HOLDING if holding else TrackingState.COASTING
            return LossRecoveryResult(False, holding, last_track_time, state)
        self._zoom.reset_to_min()
        self._log(
            f"return to search (lost {elapsed:.1f}s), "
            "mode=FOLLOW, zoom reset"
        )
        return LossRecoveryResult(
            True,
            False,
            last_track_time,
            TrackingState.HOLDING,
        )

    def _log(self, message: str) -> None:
        self._hardware.logger.info(
            f"GimbalNavigation({self._hardware.mount.name}): {message}"
        )

    def _warn(self, message: str) -> None:
        self._hardware.logger.warning(
            f"GimbalNavigation({self._hardware.mount.name}): {message}"
        )


__all__ = ["GimbalLossRecovery", "LossRecoveryResult"]
