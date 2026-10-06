"""MAVLink message construction and emission for camera mounts."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterable, Protocol

import numpy as np
from pymavlink.dialects.v20.ardupilotmega import (
    GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
    MAVLink_camera_fov_status_message,
    MAVLink_camera_settings_message,
    MAVLink_gimbal_device_attitude_status_message,
)

from navpy.modules.vision.mavlink_camera_components import (
    CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID,
)
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation

if TYPE_CHECKING:
    from navpy.modules.vision.camera_mount import CameraMount
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
    from navpy.modules.vision.vision_profile_types import CameraMountSpec


class GimbalMavlinkPublisher(Protocol):
    def send_mavlink_message(
        self,
        message: object,
        *,
        source_component: int | None = None,
    ) -> None: ...


class GimbalTelemetryLogger(Protocol):
    def warning(self, message: str) -> None: ...


@dataclass(frozen=True)
class GimbalTelemetrySource:
    mount: "CameraMount"
    device_id: int


class GimbalTelemetryEmitter:
    """Translate mount readback into gimbal and camera MAVLink messages."""

    def __init__(
        self,
        vehicle: GimbalMavlinkPublisher,
        mount_specs: Iterable["CameraMountSpec"],
        logger: GimbalTelemetryLogger,
    ) -> None:
        self._vehicle = vehicle
        self._logger = logger
        self._sources = [
            GimbalTelemetrySource(spec.mount, spec.gimbal_device_id)
            for spec in mount_specs
        ]
        self._boot_monotonic_s = time.monotonic()
        self._warned_errors: set[tuple[int, str, str]] = set()
        self._last_optics_sample_ids: dict[int, str] = {}

    def publish_once(self) -> None:
        for source in self._sources:
            try:
                message = self._build_gimbal_message(source)
                self._vehicle.send_mavlink_message(message)
                self._publish_camera_optics(
                    source,
                    message.q,
                    message.time_boot_ms,
                )
            except Exception as error:
                self._warn_once(source, error)

    def _build_gimbal_message(
        self,
        source: GimbalTelemetrySource,
    ) -> MAVLink_gimbal_device_attitude_status_message:
        gimbal = source.mount.get_gimbal_data()
        return MAVLink_gimbal_device_attitude_status_message(
            target_system=0,
            target_component=0,
            time_boot_ms=self._time_boot_ms(),
            flags=GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
            q=_attitude_quaternion_wxyz(gimbal),
            angular_velocity_x=math.nan,
            angular_velocity_y=math.nan,
            angular_velocity_z=math.nan,
            failure_flags=0,
            delta_yaw=math.nan,
            delta_yaw_velocity=math.nan,
            gimbal_device_id=source.device_id,
        )

    def _publish_camera_optics(
        self,
        source: GimbalTelemetrySource,
        quaternion: list[float],
        time_boot_ms: int,
    ) -> None:
        optics = source.mount.get_live_optics()
        if optics is None:
            return
        if optics.sample_from_hardware:
            if self._last_optics_sample_ids.get(source.device_id) == optics.sample_id:
                return
            self._last_optics_sample_ids[source.device_id] = optics.sample_id

        source_component = CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID[source.device_id]
        self._vehicle.send_mavlink_message(
            MAVLink_camera_fov_status_message(
                time_boot_ms=time_boot_ms,
                lat_camera=0,
                lon_camera=0,
                alt_camera=0,
                lat_image=0,
                lon_image=0,
                alt_image=0,
                q=quaternion,
                hfov=math.degrees(optics.fov_h_rad),
                vfov=math.degrees(optics.fov_v_rad),
            ),
            source_component=source_component,
        )
        zoom_level = _coerce_positive_float(optics.zoom_level)
        if zoom_level is None:
            return
        self._vehicle.send_mavlink_message(
            MAVLink_camera_settings_message(
                time_boot_ms=time_boot_ms,
                mode_id=0,
                zoomLevel=zoom_level,
                focusLevel=0.0,
            ),
            source_component=source_component,
        )

    def _time_boot_ms(self) -> int:
        return int(
            (time.monotonic() - self._boot_monotonic_s) * 1000
        ) & 0xFFFFFFFF

    def _warn_once(
        self,
        source: GimbalTelemetrySource,
        error: Exception,
    ) -> None:
        key = (source.device_id, type(error).__name__, str(error))
        if key in self._warned_errors:
            return
        self._warned_errors.add(key)
        self._logger.warning(
            f"Gimbal telemetry publish failed for device "
            f"{source.device_id}: {error}"
        )


def _attitude_quaternion_wxyz(gimbal: "GimbalData") -> list[float]:
    euler = get_euler_by_sequence(gimbal.att, gimbal.g_seq)
    rotation = Rotation.from_euler(gimbal.g_seq, euler, degrees=gimbal.degrees)
    return _rotation_matrix_to_quaternion_wxyz(rotation.as_matrix())


def _coerce_positive_float(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result) or result <= 0.0:
        return None
    return result


def _rotation_matrix_to_quaternion_wxyz(matrix: np.ndarray) -> list[float]:
    m = matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return [w / norm, x / norm, y / norm, z / norm]


__all__ = [
    "GimbalMavlinkPublisher",
    "GimbalTelemetryEmitter",
    "GimbalTelemetryLogger",
    "GimbalTelemetrySource",
]
