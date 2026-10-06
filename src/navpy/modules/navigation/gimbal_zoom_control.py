"""Zoom control and detection-session zoom memory."""

from __future__ import annotations

from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalDetectionMemory,
    GimbalSessionCommandGate,
    GimbalSessionFence,
    GimbalTrackers,
)
from navpy.modules.vision.gimbal_rate_types import (
    GimbalTrackResult,
    TrackingState,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_zoom_ports import ZoomLogger
from navpy.modules.vision.target_zoom_orchestrator import TargetZoomTracker
from navpy.modules.vision.target_zoom_types import ZoomStopPlan, ZoomTrackResult


class GimbalZoomController:
    """Own zoom commands and detection-session zoom memory."""

    def __init__(
        self,
        logger: ZoomLogger,
        mount_name: str,
        trackers: GimbalTrackers,
        gate: GimbalSessionCommandGate,
        fence: GimbalSessionFence,
        detection: GimbalDetectionMemory,
    ) -> None:
        self._logger = logger
        self._mount_name = mount_name
        self._trackers = trackers
        self._gate = gate
        self._fence = fence
        self._detection = detection

    @property
    def is_stable(self) -> bool:
        tracker = self._trackers.zoom
        return True if tracker is None else tracker.is_zoom_stable

    @property
    def result(self) -> ZoomTrackResult | None:
        tracker = self._trackers.zoom
        return None if tracker is None else tracker.last_result

    def set_size_demand(self, enabled: bool) -> None:
        tracker = self._trackers.zoom
        if tracker is not None:
            tracker.set_size_demand(enabled)

    def start_session(self) -> bool:
        plan = self.prepare_session_start()
        if plan is None:
            return False
        self.commit_session_start(plan)
        return True

    def prepare_session_start(self) -> ZoomStopPlan | None:
        tracker = self._trackers.zoom
        if tracker is None:
            return ZoomStopPlan(False, False)
        return tracker.prepare_session_start()

    def commit_session_start(self, plan: ZoomStopPlan) -> None:
        tracker = self._trackers.zoom
        if tracker is not None:
            tracker.commit_session_start(plan)

    def freeze_at_min(self) -> bool:
        with self._gate.lock:
            with self._fence.lock:
                self._detection.terminal_zoom_frozen_at_min = True
            tracker = self._trackers.zoom
            if tracker is None:
                return True
            tracker.set_size_demand(False)
            if tracker.reset_to_min():
                return True
            self._warn("terminal zoom freeze failed")
            return False

    def reset_to_min(self) -> bool:
        tracker = self._trackers.zoom
        if tracker is None:
            return True
        if tracker.reset_to_min():
            return True
        self._warn("zoom reset to hardware minimum failed")
        return False

    def update(
        self,
        target: DetectedObject | None,
        cached_target: DetectedObject | None,
        terminal_zoom_frozen: bool,
        pointing: GimbalTrackResult | None,
    ) -> DetectedObject | None:
        tracker = self._trackers.zoom
        if tracker is None or terminal_zoom_frozen:
            return cached_target
        if self._trackers.rate is None:
            cached_target = target if target is not None else cached_target
            zoom_input = target
        elif (
            target is not None
            and pointing is not None
            and pointing.state is TrackingState.TRACKING
        ):
            cached_target = target
            zoom_input = target
        else:
            zoom_input = None
        tracker.update(zoom_input, pointing=pointing)
        return cached_target

    def _warn(self, message: str) -> None:
        self._logger.warning(
            f"GimbalNavigation({self._mount_name}): {message}"
        )


__all__ = ["GimbalZoomController"]
