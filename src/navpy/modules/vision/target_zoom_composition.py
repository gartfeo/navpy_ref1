"""Composition root for the split target-zoom controller."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vision.continuous_zoom_policy import ContinuousZoomPolicy
from navpy.modules.vision.target_zoom_actuator_epoch import (
    TargetZoomActuatorEpoch,
)
from navpy.modules.vision.target_zoom_continuous import ContinuousZoomTick
from navpy.modules.vision.target_zoom_drive import ZoomDrive
from navpy.modules.vision.target_zoom_lifecycle import TargetZoomLifecycle
from navpy.modules.vision.target_zoom_mount_adapters import MountZoomAdapter
from navpy.modules.vision.target_zoom_ports import ZoomLogger, ZoomMountPort
from navpy.modules.vision.target_zoom_public_view import TargetZoomPublicView
from navpy.modules.vision.target_zoom_readback import ZoomReadbackState
from navpy.modules.vision.target_zoom_session import TargetZoomSession
from navpy.modules.vision.target_zoom_types import TargetZoomTrackerConfig
from navpy.modules.vision.target_zoom_update import (
    TargetZoomUpdate,
    TargetZoomWarningState,
)


@dataclass(frozen=True)
class TargetZoomParts:
    view: TargetZoomPublicView
    lifecycle: TargetZoomLifecycle
    updater: TargetZoomUpdate


def build_target_zoom(
    mount: ZoomMountPort,
    logger: ZoomLogger,
    config: TargetZoomTrackerConfig | None,
) -> TargetZoomParts:
    adapter = MountZoomAdapter(mount, logger)
    readback = ZoomReadbackState(adapter)
    drive = ZoomDrive(adapter, adapter, readback, logger)
    continuous = ContinuousZoomTick(
        adapter,
        readback,
        drive,
        ContinuousZoomPolicy(),
    )
    session = TargetZoomSession(config or TargetZoomTrackerConfig())
    warnings = TargetZoomWarningState()
    epoch = TargetZoomActuatorEpoch(adapter, drive, continuous, session)
    return TargetZoomParts(
        view=TargetZoomPublicView(adapter, drive, session),
        lifecycle=TargetZoomLifecycle(
            drive,
            readback,
            continuous,
            session,
            warnings,
            epoch,
        ),
        updater=TargetZoomUpdate(
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


__all__ = ["TargetZoomParts", "build_target_zoom"]
