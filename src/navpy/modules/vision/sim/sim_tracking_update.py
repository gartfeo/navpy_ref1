"""Simulator detection-to-gimbal tracking update."""

from __future__ import annotations

from typing import Optional

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.sim_camera_ports import TrackingCameraPort
from navpy.modules.vision.sim.sim_runtime_ports import (
    DebugSink,
    TrackingRuntimePort,
    TrackLossDiagnoser,
)


class SimTrackingUpdater:
    """Apply one detection through exact tracking and diagnostic ports."""

    def __init__(
        self,
        tracking: TrackingRuntimePort,
        camera: TrackingCameraPort,
        debug: DebugSink,
        diagnose_track_loss: TrackLossDiagnoser,
    ) -> None:
        self._tracking = tracking
        self._camera = camera
        self._debug = debug
        self._diagnose_track_loss = diagnose_track_loss

    def update(
        self,
        detected_targets: list[DetectedObject],
        camera_location: Location,
        uas_attitude: Attitude,
        timestamp_s: float,
    ) -> None:
        tracking_id = self._tracking.tracking_obj_id
        if tracking_id is None:
            return
        tracked_target = next(
            (
                detection
                for detection in detected_targets
                if detection.identity.obj_id == tracking_id
            ),
            None,
        )
        self._log_tracking(
            tracking_id,
            tracked_target,
            detected_targets,
            camera_location,
            uas_attitude,
        )
        self._tracking.apply_detection_update(tracked_target, timestamp_s)

    def _log_tracking(
        self,
        tracking_id: int,
        tracked_target: Optional[DetectedObject],
        detected_targets: list[DetectedObject],
        camera_location: Location,
        uas_attitude: Attitude,
    ) -> None:
        gimbal_data = self._camera.read_gimbal()
        camera_matrix = self._camera.read_matrix()
        center_x = float(camera_matrix[0, 2])
        center_y = float(camera_matrix[1, 2])
        gimbal_body, uas, verify = self._tracking_context(
            gimbal_data,
            uas_attitude,
        )
        if tracked_target is not None:
            self._debug(
                f"TRACK: IN_FRAME obj={tracking_id} "
                f"px=({tracked_target.pixel.u_px:.0f},{tracked_target.pixel.v_px:.0f}) "
                f"off=({tracked_target.pixel.u_px - center_x:.0f},"
                f"{tracked_target.pixel.v_px - center_y:.0f}) "
                f"{gimbal_body} {uas} {verify}"
            )
            return
        detection_ids = [target.identity.obj_id for target in detected_targets]
        reason = self._diagnose_track_loss(
            tracking_id,
            camera_location,
            uas_attitude,
            camera_matrix,
            gimbal_data,
        )
        self._debug(
            f"TRACK: LOST obj={tracking_id} reason={reason} "
            f"detected={detection_ids} "
            f"{gimbal_body} {uas} {verify}"
        )

    @staticmethod
    def _tracking_context(
        gimbal_data: GimbalData,
        uas_attitude: Attitude,
    ) -> tuple[str, str, str]:
        gimbal_body = (
            f"g_body=(y={gimbal_data.att.yaw:.1f},"
            f"p={gimbal_data.att.pitch:.1f},"
            f"r={gimbal_data.att.roll:.1f})"
        )
        uas = (
            f"uav=(y={uas_attitude.yaw:.1f},p={uas_attitude.pitch:.1f},"
            f"r={uas_attitude.roll:.1f})"
        )
        verify = (
            f"body+uav=(y={gimbal_data.att.yaw + uas_attitude.yaw:.1f},"
            f"p={gimbal_data.att.pitch + uas_attitude.pitch:.1f})"
        )
        return gimbal_body, uas, verify


__all__ = ["SimTrackingUpdater"]
