"""Construction of visual and known-geo gimbal capability owners."""

from __future__ import annotations

from dataclasses import dataclass

import pymap3d

from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.gimbal_geo_commander import GimbalGeoCommander
from navpy.modules.navigation.gimbal_geo_diagnostics import GeoRayDiagnosticLogger
from navpy.modules.navigation.gimbal_geo_lifecycle import GimbalGeoLifecycle
from navpy.modules.navigation.gimbal_geo_acquisition import GeoAcquisitionZoom
from navpy.modules.navigation.gimbal_geo_session import GimbalGeoTracking
from navpy.modules.navigation.gimbal_geo_zoom_selector import GeoZoomSelector
from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalDetectionMemory,
    GimbalGeoMemory,
    GimbalNavigationStatus,
    GimbalHardware,
    GimbalSessionCommandGate,
    GimbalSessionFence,
    GimbalTrackingSetup,
    GimbalTrackers,
)
from navpy.modules.navigation.gimbal_neutral_return import GimbalNeutralReturn
from navpy.modules.navigation.gimbal_detection_lifecycle import (
    GimbalDetectionLifecycle,
)
from navpy.modules.navigation.gimbal_loss_recovery import GimbalLossRecovery
from navpy.modules.navigation.gimbal_visual_tracking import GimbalVisualTracking
from navpy.modules.navigation.gimbal_zoom_control import GimbalZoomController
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.geo_ray_diagnostics import compute_geo_ray_diagnostic
from navpy.modules.vision.gimbal_rate_tracker import (
    GimbalRateTracker,
    GimbalRateTrackerConfig,
)
from navpy.modules.vision.poi_zoom_orchestrator import PoiZoomTracker
from navpy.modules.vision.poi_zoom_types import PoiZoomTrackerConfig
from navpy.modules.vision.vision_camera_calibration import (
    read_camera_zoom_calibration,
)
from navpy.modules.vision.vision_class_profile import get_class_detect_size


@dataclass(frozen=True)
class GimbalNavigationParts:
    detection: GimbalDetectionLifecycle
    visual: GimbalVisualTracking
    geo: GimbalGeoTracking
    zoom: GimbalZoomController
    status: GimbalNavigationStatus


def _build_trackers(
    mount: CameraMount,
    logger: ILogger,
    tracking: GimbalTrackingSetup | None,
    zoom_config: PoiZoomTrackerConfig | None,
    rate_tracker: GimbalRateTracker | None,
    zoom_tracker: PoiZoomTracker | None,
) -> tuple[GimbalTrackingSetup, GimbalHardware, GimbalTrackers]:
    active_tracking = tracking or GimbalTrackingSetup(
        GimbalRateTrackerConfig()
    )
    hardware = GimbalHardware(mount, mount.gimbal, logger)
    if rate_tracker is None and tracking is not None:
        rate_tracker = GimbalRateTracker(mount.gimbal, logger, tracking.rate)
        logger.info(
            f"GimbalNavigation({mount.name}): Gimbal tracking enabled "
            f"(correction_bw={tracking.rate.correction_bw}, "
            f"max_rate={tracking.rate.max_rate})"
        )
    if zoom_tracker is None and zoom_config is not None:
        zoom_tracker = PoiZoomTracker(mount, logger, zoom_config)
        logger.info(
            f"GimbalNavigation({mount.name}): Zoom tracking enabled "
            f"(min_zoom={zoom_tracker.min_zoom}, "
            f"max_zoom={zoom_tracker.max_zoom}, "
            f"target_pixels={zoom_config.target_pixels})"
        )
    return (
        active_tracking,
        hardware,
        GimbalTrackers(rate_tracker, zoom_tracker, zoom_config),
    )


def _build_geo_tracking(
    mount: CameraMount,
    logger: ILogger,
    hardware: GimbalHardware,
    trackers: GimbalTrackers,
    command_gate: GimbalSessionCommandGate,
    fence: GimbalSessionFence,
    detection_memory: GimbalDetectionMemory,
    geo_memory: GimbalGeoMemory,
    zoom: GimbalZoomController,
    neutral: GimbalNeutralReturn | None,
) -> GimbalGeoTracking:
    selector = GeoZoomSelector(
        hardware,
        get_class_detect_size,
        lambda: read_camera_zoom_calibration(mount.camera),
    )
    acquisition = GeoAcquisitionZoom(
        hardware,
        trackers,
        command_gate,
        fence,
        geo_memory,
        lambda *args, **kwargs: pymap3d.geodetic2ned(*args, **kwargs),
        selector,
    )
    diagnostics = GeoRayDiagnosticLogger(
        hardware,
        fence,
        geo_memory,
        lambda **kwargs: compute_geo_ray_diagnostic(**kwargs),
        lambda: logger.is_enabled_for(CacheLogLevel.DEBUG),
    )
    return GimbalGeoTracking(
        GimbalGeoLifecycle(
            hardware,
            command_gate,
            fence,
            detection_memory,
            geo_memory,
            zoom,
            neutral,
        ),
        GimbalGeoCommander(
            hardware,
            command_gate,
            fence,
            geo_memory,
            diagnostics,
        ),
        acquisition,
    )


def build_gimbal_navigation(
    mount: CameraMount,
    logger: ILogger,
    tracking: GimbalTrackingSetup | None,
    zoom_config: PoiZoomTrackerConfig | None,
    *,
    rate_tracker: GimbalRateTracker | None,
    zoom_tracker: PoiZoomTracker | None,
    neutral_pitch_deg: float | None,
) -> GimbalNavigationParts:
    active, hardware, trackers = _build_trackers(
        mount,
        logger,
        tracking,
        zoom_config,
        rate_tracker,
        zoom_tracker,
    )
    command_gate = GimbalSessionCommandGate()
    fence = GimbalSessionFence()
    detection_memory = GimbalDetectionMemory()
    geo_memory = GimbalGeoMemory()
    zoom = GimbalZoomController(
        logger,
        mount.name,
        trackers,
        command_gate,
        fence,
        detection_memory,
    )
    active_rate_tracker = trackers.rate
    neutral = (
        None
        if active_rate_tracker is None
        else GimbalNeutralReturn(
            hardware.gimbal,
            active_rate_tracker,
            (
                mount.get_gimbal_data().att.pitch
                if neutral_pitch_deg is None
                else float(neutral_pitch_deg)
            ),
        )
    )
    loss = GimbalLossRecovery(hardware, active.loss, zoom, neutral)
    detection = GimbalDetectionLifecycle(
        hardware,
        trackers,
        command_gate,
        fence,
        detection_memory,
        geo_memory,
        zoom,
        neutral,
    )
    visual = GimbalVisualTracking(
        trackers,
        command_gate,
        fence,
        detection_memory,
        zoom,
        loss,
    )
    geo = _build_geo_tracking(
        mount,
        logger,
        hardware,
        trackers,
        command_gate,
        fence,
        detection_memory,
        geo_memory,
        zoom,
        neutral,
    )
    status = GimbalNavigationStatus(
        trackers,
        fence,
        detection_memory,
        geo_memory,
        active.loss,
    )
    return GimbalNavigationParts(detection, visual, geo, zoom, status)


__all__ = ["GimbalNavigationParts", "build_gimbal_navigation"]
