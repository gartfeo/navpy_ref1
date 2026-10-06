"""Detection-session arm and disarm transactions."""

from __future__ import annotations

from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalDetectionMemory,
    GimbalGeoMemory,
    GimbalHardware,
    GimbalSessionCommandGate,
    GimbalSessionFence,
    GimbalTrackers,
)
from navpy.modules.navigation.gimbal_neutral_return import GimbalNeutralReturn
from navpy.modules.navigation.gimbal_tracking_constants import MODE_LOCK
from navpy.modules.navigation.gimbal_zoom_control import GimbalZoomController


class GimbalDetectionLifecycle:
    """Arm and disarm mutually exclusive visual detection sessions."""

    def __init__(
        self,
        hardware: GimbalHardware,
        trackers: GimbalTrackers,
        gate: GimbalSessionCommandGate,
        fence: GimbalSessionFence,
        detection: GimbalDetectionMemory,
        geo: GimbalGeoMemory,
        zoom: GimbalZoomController,
        neutral: GimbalNeutralReturn | None,
    ) -> None:
        self._hardware = hardware
        self._trackers = trackers
        self._gate = gate
        self._fence = fence
        self._detection = detection
        self._geo = geo
        self._zoom = zoom
        self._neutral = neutral

    def start(self, obj_id: int) -> None:
        if self._trackers.rate is None and self._trackers.zoom is None:
            return
        with self._gate.lock:
            with self._fence.lock:
                previous_id = self._detection.tracking_obj_id
            rate_tracker = self._trackers.rate
            if rate_tracker is not None:
                self._hardware.gimbal.set_motion_mode(MODE_LOCK)
            zoom_plan = self._zoom.prepare_session_start()
            if zoom_plan is None:
                raise OSError("failed to stop the prior zoom session")
            if rate_tracker is not None:
                rate_tracker.reset()
            self._zoom.commit_session_start(zoom_plan)
            with self._fence.lock:
                self._geo.clear()
                self._clear_detection(obj_id)
                self._fence.generation += 1
        mode = "LOCK" if self._trackers.rate is not None else "ZOOM-ONLY"
        self._hardware.logger.info(
            f"GimbalNavigation({self._hardware.mount.name}): start_tracking "
            f"obj_id={obj_id} (prev={previous_id}, mode={mode})"
        )

    def stop(self, to_neutral: bool = True) -> None:
        with self._gate.lock:
            with self._fence.lock:
                previous_id = self._detection.tracking_obj_id
            if previous_id is None:
                return
            if self._trackers.rate is not None:
                self._stop_rate(to_neutral)
            if not self._zoom.reset_to_min():
                return
            with self._fence.lock:
                if self._detection.tracking_obj_id != previous_id:
                    return
                self._clear_detection(None)
                self._fence.generation += 1
        self._log_stop(previous_id, to_neutral)

    def _stop_rate(self, to_neutral: bool) -> None:
        if self._neutral is None:
            rate_tracker = self._trackers.rate
            try:
                self._hardware.gimbal.set_rate(0.0, 0.0)
            finally:
                if rate_tracker is not None:
                    rate_tracker.reset()
        elif to_neutral:
            self._neutral.execute()
        else:
            self._neutral.hold()

    def _clear_detection(self, tracking_obj_id: int | None) -> None:
        self._detection.tracking_obj_id = tracking_obj_id
        self._detection.last_track_time = None
        self._detection.recentered = False
        self._detection.holding = False
        self._detection.last_tracked_for_zoom = None
        self._detection.terminal_zoom_frozen_at_min = False

    def _log_stop(self, previous_id: int, to_neutral: bool) -> None:
        if to_neutral:
            detail = (
                f"neutral_pitch={self._neutral.pitch_deg if self._neutral else 0.0} "
                "mode=FOLLOW"
            )
        else:
            detail = "keep_attitude=True zero_rate=True"
        self._hardware.logger.info(
            f"GimbalNavigation({self._hardware.mount.name}): stop_tracking "
            f"obj_id={previous_id} {detail}"
        )


__all__ = ["GimbalDetectionLifecycle"]
