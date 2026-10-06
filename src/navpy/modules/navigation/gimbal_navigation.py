"""Public boundary for visual and known-geo gimbal capabilities."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.gimbal_navigation_composition import (
    build_gimbal_navigation,
)
from navpy.modules.navigation.gimbal_navigation_facets import (
    GimbalDetectionFacet,
    GimbalGeoFacet,
    GimbalNavigationStatusFacet,
    GimbalVisualFacet,
    GimbalZoomFacet,
)
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.navigation.gimbal_tracking_constants import (
    MODE_FOLLOW,
    MODE_LOCK,
)
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTracker
from navpy.modules.vision.target_zoom_orchestrator import TargetZoomTracker
from navpy.modules.vision.target_zoom_types import TargetZoomTrackerConfig


class GimbalNavigation(
    GimbalNavigationStatusFacet,
    GimbalDetectionFacet,
    GimbalGeoFacet,
    GimbalVisualFacet,
    GimbalZoomFacet,
):
    """One-field facade over segregated gimbal capability owners."""

    def __init__(
        self,
        mount: CameraMount,
        logger: ILogger,
        tracking: GimbalTrackingSetup | None = None,
        zoom_config: TargetZoomTrackerConfig | None = None,
        *,
        rate_tracker: GimbalRateTracker | None = None,
        zoom_tracker: TargetZoomTracker | None = None,
        neutral_pitch_deg: float | None = None,
    ) -> None:
        self._parts = build_gimbal_navigation(
            mount,
            logger,
            tracking,
            zoom_config,
            rate_tracker=rate_tracker,
            zoom_tracker=zoom_tracker,
            neutral_pitch_deg=neutral_pitch_deg,
        )

__all__ = ["GimbalNavigation", "MODE_FOLLOW", "MODE_LOCK"]
