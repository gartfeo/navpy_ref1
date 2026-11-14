"""Zoom control and detection-session zoom memory.

With fixed zoom configured (HITL step 18, ``fixed_step_zoom_drive.py``) the fixed-step drive is the only zoom
writer: no POI zoom tracker is built, and every reset or freeze commands the widest calibrated step.
"""

from __future__ import annotations

from navpy.core.command.gimbal.gimbal_navigation_state import (
    GimbalDetectionMemory,
    GimbalSessionCommandGate,
    GimbalSessionFence,
    GimbalTrackers,
)
from navpy.core.perception.pointing.gimbal_rate_types import (
    GimbalTrackResult,
    TrackingState,
)
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.zoom.fixed_step_zoom_drive import FixedStepZoomDrive
from navpy.core.perception.zoom.poi_zoom_ports import ZoomLogger
from navpy.core.perception.zoom.poi_zoom_orchestrator import PoiZoomTracker
from navpy.core.perception.zoom.poi_zoom_types import ZoomStopPlan, ZoomTrackResult


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
        fixed: FixedStepZoomDrive | None = None,
    ) -> None:
        self._logger = logger
        self._mount_name = mount_name
        self._trackers = trackers
        self._gate = gate
        self._fence = fence
        self._detection = detection
        self._fixed = fixed

    @property
    def drives_zoom(self) -> bool:
        return self._fixed is not None or self._trackers.zoom is not None

    @property
    def is_stable(self) -> bool:
        if self._fixed is not None:
            return self._fixed.is_stable
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
                self._detection.guidance_zoom_frozen_at_min = True
            if self._fixed is not None:
                if self._fixed.command_widest("freeze"):
                    return True
                self._warn("guidance fixed zoom freeze failed")
                return False
            tracker = self._trackers.zoom
            if tracker is None:
                return True
            tracker.set_size_demand(False)
            if tracker.reset_to_min():
                return True
            self._warn("guidance zoom freeze failed")
            return False

    def reset_to_min(self) -> bool:
        if self._fixed is not None:
            if self._fixed.command_widest("reset"):
                return True
            self._warn("fixed zoom reset to the widest step failed")
            return False
        tracker = self._trackers.zoom
        if tracker is None:
            return True
        if tracker.reset_to_min():
            return True
        self._warn("zoom reset to hardware minimum failed")
        return False

    def update(
        self,
        poi: DetectedObject | None,
        cached_poi: DetectedObject | None,
        guidance_zoom_frozen: bool,
        pointing: GimbalTrackResult | None,
    ) -> DetectedObject | None:
        if self._fixed is not None:
            return self._update_fixed(poi, cached_poi, guidance_zoom_frozen, pointing)
        tracker = self._trackers.zoom
        if tracker is None or guidance_zoom_frozen:
            return cached_poi
        if self._trackers.rate is None:
            cached_poi = poi if poi is not None else cached_poi
            zoom_input = poi
        elif (
            poi is not None
            and pointing is not None
            and pointing.state is TrackingState.TRACKING
        ):
            cached_poi = poi
            zoom_input = poi
        else:
            zoom_input = None
        tracker.update(zoom_input, pointing=pointing)
        return cached_poi

    def _update_fixed(
        self,
        poi: DetectedObject | None,
        cached_poi: DetectedObject | None,
        frozen: bool,
        pointing: GimbalTrackResult | None,
    ) -> DetectedObject | None:
        fixed = self._fixed
        if fixed is None:
            return cached_poi
        # Same gating as the POI tracker: with rate tracking, only a tracked POI counts as a measurement.
        if self._trackers.rate is None:
            cached_poi = poi if poi is not None else cached_poi
            zoom_input = poi
        elif poi is not None and pointing is not None and pointing.state is TrackingState.TRACKING:
            cached_poi = poi
            zoom_input = poi
        else:
            zoom_input = None
        fixed.update(zoom_input, frozen=frozen)
        return cached_poi

    def _warn(self, message: str) -> None:
        self._logger.warning(
            f"GimbalNavigation({self._mount_name}): {message}"
        )


__all__ = ["GimbalZoomController"]
