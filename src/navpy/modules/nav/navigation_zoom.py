"""Best-effort optical controls for navigation transitions."""

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.detector_ports import ZoomControlPort


class ZoomController:
    """Recognition optics plus fail-closed final-approach zoom control."""

    def __init__(self, zoom: ZoomControlPort, logger: ILogger) -> None:
        self._zoom = zoom
        self._logger = logger

    def set_recognition_demand(self, enabled: bool) -> None:
        try:
            self._zoom.set_zoom_size_demand(enabled)
        except Exception as error:  # noqa: BLE001 - non-critical optics
            self._logger.warning(f"set_zoom_size_demand failed: {error}")

    def freeze_final_approach_wide(self) -> bool:
        try:
            frozen = self._zoom.freeze_final_approach_zoom_at_min()
        except Exception as error:  # noqa: BLE001 - non-critical optics
            self._logger.warning(f"freeze_final_approach_zoom_at_min failed: {error}")
            return False
        if not frozen:
            self._logger.warning("freeze_final_approach_zoom_at_min was not committed")
            return False
        return True


__all__ = ["ZoomController"]
