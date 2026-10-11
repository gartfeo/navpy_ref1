"""Tracker backend construction from settings."""

from __future__ import annotations

from typing import Optional

from navpy.core.kernel.device import DeviceT
from navpy.plugins.tracking.boxmot.tracker_backends_boxmot import (
    BotSortTrackerBackend,
    StrongSortTrackerBackend,
)
from navpy.plugins.tracking.tracker_config import tracker_config_from_settings
from navpy.plugins.tracking.kalman.custom_tracker_backend import CustomTrackerBackend
from navpy.core.ports.tracker_backends import TrackerBackend


def create_tracker_backend(
        settings: Optional[dict],
        *,
        detector_device: DeviceT = "auto",
        detector_conf: float = 0.25,
        logger=None,
) -> TrackerBackend:
    config = tracker_config_from_settings(settings, detector_conf=detector_conf)
    backend = config.runtime.backend
    if backend in ("custom", "kalman", "mot"):
        return CustomTrackerBackend(config)
    if backend in ("botsort", "bot-sort", "bot_sort"):
        return BotSortTrackerBackend(config, detector_device=detector_device, logger=logger)
    if backend in ("strongsort", "strong-sort", "strong_sort"):
        return StrongSortTrackerBackend(config, detector_device=detector_device, logger=logger)
    raise ValueError(
        "tracker backend must be one of: custom, botsort, strongsort "
        f"(got {config.runtime.backend!r})"
    )
