"""Known-geolocation gimbal session lifecycle."""

from __future__ import annotations

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalDetectionMemory,
    GimbalGeoMemory,
    GimbalHardware,
    GimbalSessionCommandGate,
    GimbalSessionFence,
)
from navpy.modules.navigation.gimbal_neutral_return import GimbalNeutralReturn
from navpy.modules.navigation.gimbal_tracking_constants import (
    GIMBAL_COMMAND_ERRORS,
    MODE_LOCK,
)
from navpy.modules.navigation.gimbal_zoom_control import GimbalZoomController


class GimbalGeoLifecycle:
    """Start, swap, and stop mutually exclusive known-geo sessions."""

    def __init__(
        self,
        hardware: GimbalHardware,
        gate: GimbalSessionCommandGate,
        fence: GimbalSessionFence,
        detection: GimbalDetectionMemory,
        geo: GimbalGeoMemory,
        zoom: GimbalZoomController,
        neutral: GimbalNeutralReturn | None,
    ) -> None:
        self._hardware = hardware
        self._gate = gate
        self._fence = fence
        self._detection = detection
        self._geo = geo
        self._zoom = zoom
        self._neutral = neutral

    def start(
        self,
        target_loc: Location | None,
        geo_ref: GeoRefCalc | None,
    ) -> None:
        if self._neutral is None:
            self._warn(
                "start_geo_tracking ignored - no rate tracker "
                "(no neutral source for stop cleanup)"
            )
            return
        if target_loc is None or geo_ref is None:
            self._warn(
                "start_geo_tracking ignored - missing argument "
                f"(target_loc={target_loc}, geo_ref={geo_ref})"
            )
            return
        with self._gate.lock:
            with self._fence.lock:
                if self._detection.tracking_obj_id is not None:
                    raise RuntimeError(
                        f"GimbalNavigation({self._hardware.mount.name}): cannot "
                        "start geo tracking while detection is armed "
                        f"(obj_id={self._detection.tracking_obj_id})"
                    )
                previous_target = self._geo.target
            is_swap = previous_target is not None
            if not is_swap:
                self._hardware.gimbal.set_motion_mode(MODE_LOCK)
            with self._fence.lock:
                self._geo.target = target_loc
                self._geo.geo_ref = geo_ref
                self._geo.zoom_key = None
                self._geo.last_ray_log_key = None
                self._fence.generation += 1
        if is_swap:
            self._log(
                "start_geo_tracking swap "
                f"(prev_target={previous_target}, target={target_loc})"
            )
            return
        self._log(f"start_geo_tracking target={target_loc} mode=LOCK")

    def stop(self) -> None:
        with self._gate.lock:
            with self._fence.lock:
                previous_target = self._geo.target
                reset_zoom = self._geo.zoom_key is not None
                self._geo.clear()
                self._fence.generation += 1
            if previous_target is None:
                return
            if reset_zoom:
                self._zoom.reset_to_min()
            try:
                self._neutral.execute()
            except GIMBAL_COMMAND_ERRORS as exc:
                self._warn(f"geo return_to_neutral failed: {exc}")
        self._log(
            f"stop_geo_tracking prev_target={previous_target} mode=FOLLOW"
        )

    def _log(self, message: str) -> None:
        self._hardware.logger.info(
            f"GimbalNavigation({self._hardware.mount.name}): {message}"
        )

    def _warn(self, message: str) -> None:
        self._hardware.logger.warning(
            f"GimbalNavigation({self._hardware.mount.name}): {message}"
        )


__all__ = ["GimbalGeoLifecycle"]
