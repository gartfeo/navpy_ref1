"""Link, arming, and preflight-health read model."""
from __future__ import annotations

from typing import Literal

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_SYS_STATUS_PREARM_CHECK,
    MAV_SYS_STATUS_SENSOR_3D_ACCEL,
    MAV_SYS_STATUS_SENSOR_3D_GYRO,
    MAV_SYS_STATUS_SENSOR_3D_MAG,
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
    MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
)

from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.message_store import MessageStore


PREARM_STATE_NO_SYS_STATUS = "no_sys_status"
PREARM_STATE_NOT_REPORTED = "not_reported"
PREARM_STATE_CHECKS_DISABLED = "checks_disabled"
PREARM_STATE_FAILED = "failed"
PREARM_STATE_OK = "ok"
PrearmCheckState = Literal[
    "no_sys_status",
    "not_reported",
    "checks_disabled",
    "failed",
    "ok",
]

PREFLIGHT_SENSOR_BITS = {
    "gyro": MAV_SYS_STATUS_SENSOR_3D_GYRO,
    "accel": MAV_SYS_STATUS_SENSOR_3D_ACCEL,
    "mag": MAV_SYS_STATUS_SENSOR_3D_MAG,
    "abs_pressure": MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
    "rc": MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
}


class PreflightHealth:
    def __init__(
        self,
        messages: MessageStore,
        heartbeat_state: HeartbeatState,
        packet_loss: PacketLossTracker,
    ) -> None:
        self._messages = messages
        self._heartbeat = heartbeat_state
        self._packet_loss = packet_loss

    @property
    def link_ok(self) -> bool:
        return self._heartbeat.link_ok

    @property
    def link_quality(self) -> int:
        return self._packet_loss.quality if self.link_ok else 0

    @property
    def prearm_check_state(self) -> PrearmCheckState:
        status = self._messages.message("SYS_STATUS")
        if status is None:
            return PREARM_STATE_NO_SYS_STATUS
        present = getattr(status, "onboard_control_sensors_present", 0)
        if not present & MAV_SYS_STATUS_PREARM_CHECK:
            return PREARM_STATE_NOT_REPORTED
        enabled = getattr(status, "onboard_control_sensors_enabled", 0)
        if not enabled & MAV_SYS_STATUS_PREARM_CHECK:
            return PREARM_STATE_CHECKS_DISABLED
        health = getattr(status, "onboard_control_sensors_health", 0)
        return (
            PREARM_STATE_OK
            if health & MAV_SYS_STATUS_PREARM_CHECK
            else PREARM_STATE_FAILED
        )

    @property
    def prearm_ok(self) -> bool | None:
        state = self.prearm_check_state
        if state == PREARM_STATE_OK:
            return True
        if state == PREARM_STATE_FAILED:
            return False
        return None

    @property
    def sensor_health(self) -> dict[str, bool | None]:
        status = self._messages.message("SYS_STATUS")
        if status is None:
            return {name: None for name in PREFLIGHT_SENSOR_BITS}
        present = getattr(status, "onboard_control_sensors_present", 0)
        enabled = getattr(status, "onboard_control_sensors_enabled", 0)
        health = getattr(status, "onboard_control_sensors_health", 0)
        return {
            name: (
                bool(health & bit)
                if present & bit and enabled & bit
                else None
            )
            for name, bit in PREFLIGHT_SENSOR_BITS.items()
        }

    @property
    def airspeed_present(self) -> bool | None:
        status = self._messages.message("SYS_STATUS")
        if status is None:
            return None
        present = getattr(status, "onboard_control_sensors_present", 0)
        return bool(present & MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)

    @property
    def airspeed_healthy(self) -> bool | None:
        status = self._messages.message("SYS_STATUS")
        if status is None:
            return None
        bit = MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
        present = getattr(status, "onboard_control_sensors_present", 0)
        enabled = getattr(status, "onboard_control_sensors_enabled", 0)
        health = getattr(status, "onboard_control_sensors_health", 0)
        return bool(health & bit) if present & bit and enabled & bit else None

    @property
    def ekf_status(self) -> dict[str, int | float] | None:
        report = self._messages.message("EKF_STATUS_REPORT")
        if report is None:
            return None
        return {
            "flags": int(getattr(report, "flags", 0)),
            "velocity_variance": float(getattr(report, "velocity_variance", 0.0)),
            "pos_horiz_variance": float(getattr(report, "pos_horiz_variance", 0.0)),
            "pos_vert_variance": float(getattr(report, "pos_vert_variance", 0.0)),
            "compass_variance": float(getattr(report, "compass_variance", 0.0)),
            "terrain_alt_variance": float(getattr(report, "terrain_alt_variance", 0.0)),
        }

    @property
    def rc3_raw(self) -> int | None:
        channels = self._messages.message("RC_CHANNELS")
        return getattr(channels, "chan3_raw", None)

    @property
    def is_armed(self) -> bool:
        heartbeat = self._heartbeat.message
        return bool(
            heartbeat is not None
            and heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED
        )
