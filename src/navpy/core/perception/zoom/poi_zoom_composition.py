"""Composition root for the split POI-zoom controller."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.perception.zoom.continuous_zoom_policy import ContinuousZoomPolicy
from navpy.core.perception.zoom.poi_zoom_actuator_epoch import (
    PoiZoomActuatorEpoch,
)
from navpy.core.perception.zoom.poi_zoom_continuous import ContinuousZoomTick
from navpy.core.perception.zoom.poi_zoom_drive import ZoomDrive
from navpy.core.perception.zoom.poi_zoom_lifecycle import PoiZoomLifecycle
from navpy.core.perception.zoom.poi_zoom_mount_adapters import MountZoomAdapter
from navpy.core.perception.zoom.poi_zoom_ports import ZoomLogger, ZoomMountPort
from navpy.core.perception.zoom.poi_zoom_public_view import PoiZoomPublicView
from navpy.core.perception.zoom.poi_zoom_readback import ZoomReadbackState
from navpy.core.perception.zoom.poi_zoom_session import PoiZoomSession
from navpy.core.perception.zoom.poi_zoom_types import PoiZoomTrackerConfig
from navpy.core.perception.zoom.poi_zoom_update import (
    PoiZoomUpdate,
    PoiZoomWarningState,
)


@dataclass(frozen=True)
class PoiZoomParts:
    view: PoiZoomPublicView
    lifecycle: PoiZoomLifecycle
    updater: PoiZoomUpdate


def build_poi_zoom(
    mount: ZoomMountPort,
    logger: ZoomLogger,
    config: PoiZoomTrackerConfig | None,
) -> PoiZoomParts:
    adapter = MountZoomAdapter(mount, logger)
    readback = ZoomReadbackState(adapter)
    drive = ZoomDrive(adapter, adapter, readback, logger)
    continuous = ContinuousZoomTick(
        adapter,
        readback,
        drive,
        ContinuousZoomPolicy(),
    )
    session = PoiZoomSession(config or PoiZoomTrackerConfig())
    warnings = PoiZoomWarningState()
    epoch = PoiZoomActuatorEpoch(adapter, drive, continuous, session)
    return PoiZoomParts(
        view=PoiZoomPublicView(adapter, drive, session),
        lifecycle=PoiZoomLifecycle(
            drive,
            readback,
            continuous,
            session,
            warnings,
            epoch,
        ),
        updater=PoiZoomUpdate(
            adapter,
            adapter,
            readback,
            drive,
            continuous,
            session,
            warnings,
            logger,
            epoch,
        ),
    )


__all__ = ["PoiZoomParts", "build_poi_zoom"]
