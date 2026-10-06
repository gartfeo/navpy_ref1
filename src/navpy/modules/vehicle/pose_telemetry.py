"""Atomic attitude, simulator-pose, and command diagnostic views."""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from types import SimpleNamespace

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.message_store import MessageSample, MessageStore


def attitude_sample_from_message(
    message: MAVLink_message,
    receipt_time_s: float,
) -> SimpleNamespace:
    return SimpleNamespace(
        attitude=Attitude(
            math.degrees(message.pitch),
            math.degrees(message.yaw),
            math.degrees(message.roll),
        ),
        time_boot_s=float(message.time_boot_ms) * 1e-3,
        receipt_time_s=float(receipt_time_s),
        body_rates_rad_s=(
            float(message.rollspeed),
            float(message.pitchspeed),
            float(message.yawspeed),
        ),
    )


def copy_attitude_sample(sample: SimpleNamespace) -> SimpleNamespace:
    attitude = sample.attitude
    return SimpleNamespace(
        attitude=Attitude(attitude.pitch, attitude.yaw, attitude.roll),
        time_boot_s=sample.time_boot_s,
        receipt_time_s=sample.receipt_time_s,
        body_rates_rad_s=tuple(sample.body_rates_rad_s),
    )


def sim_state_coordinates_deg(
    message: MAVLink_message,
) -> tuple[float, float] | None:
    latitude_int = getattr(message, "lat_int", None)
    longitude_int = getattr(message, "lon_int", None)
    explicit = (
        isinstance(latitude_int, int)
        and not isinstance(latitude_int, bool)
        and isinstance(longitude_int, int)
        and not isinstance(longitude_int, bool)
        and (latitude_int != 0 or longitude_int != 0)
    )
    try:
        if explicit:
            latitude = float(latitude_int) * 1e-7
            longitude = float(longitude_int) * 1e-7
        else:
            latitude = float(message.lat)
            longitude = float(message.lon)
            if abs(latitude) > 90.0 or abs(longitude) > 180.0:
                latitude *= 1e-7
                longitude *= 1e-7
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    if not (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    ):
        return None
    return latitude, longitude


def simulator_truth_source_time_s(message: MAVLink_message) -> float | None:
    """SIM_STATE ``time_us`` (navlink extension) as seconds, or None.

    The field carries the FDM state-sample time on the autopilot clock —
    the same clock as ATTITUDE ``time_boot_ms``. Absent field (stock
    firmware), 0 (declared invalid), or garbage all mean "no source stamp".
    """
    time_us = getattr(message, "time_us", None)
    if isinstance(time_us, bool) or not isinstance(time_us, (int, float)):
        return None
    try:
        value = float(time_us) * 1e-6
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(value) or value <= 0.0:
        return None
    return value


def simulator_truth_pose_from_sample(
    sample: MessageSample,
) -> SimpleNamespace | None:
    message = sample.message
    coordinates = sim_state_coordinates_deg(message)
    if coordinates is None:
        return None
    latitude, longitude = coordinates
    values = (
        latitude,
        longitude,
        float(message.alt),
        float(message.roll),
        float(message.pitch),
        float(message.yaw),
    )
    if not all(math.isfinite(value) for value in values):
        return None
    return SimpleNamespace(
        location=Location(latitude, longitude, values[2], is_absolute=True),
        attitude=Attitude(
            pitch=math.degrees(values[4]),
            yaw=math.degrees(values[5]),
            roll=math.degrees(values[3]),
        ),
        receipt_time_s=sample.receipt_time_s,
        source_time_s=simulator_truth_source_time_s(message),
    )


class CommandDiagnostics:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._attitude_target = None

    def update_attitude_target(self, target: SimpleNamespace) -> None:
        with self._lock:
            self._attitude_target = target

    def attitude_target(self) -> SimpleNamespace | None:
        with self._lock:
            return self._attitude_target


class PoseTelemetry:
    def __init__(
        self,
        message_store: MessageStore,
        command_diagnostics: CommandDiagnostics,
        link_identity_source: "Callable[[], str | None] | None" = None,
    ) -> None:
        self._messages = message_store
        self._diagnostics = command_diagnostics
        self._link_identity_source = link_identity_source

    @property
    def link_identity(self) -> str | None:
        """Identity of the LIVE link this telemetry reads from.

        Produced by a provider over the open connection's endpoint and the
        heartbeat-CONFIRMED remote system — never from operator
        configuration, and None until the remote has actually spoken. The
        capture-lookup calibration binds to this value; while it is None
        every READ/EXPOSURE source stays non-atomic.
        """
        if self._link_identity_source is None:
            return None
        return self._link_identity_source()

    def attitude_history(self, count: int) -> tuple[SimpleNamespace, ...]:
        """Newest-last defensive copies of recent ATTITUDE samples."""
        samples = self._messages.recent("ATTITUDE", count)
        return tuple(
            attitude_sample_from_message(sample.message, sample.receipt_time_s)
            for sample in samples
        )

    @property
    def attitude(self) -> Attitude | None:
        message = self._messages.message("ATTITUDE")
        if not message:
            return None
        return Attitude(
            math.degrees(message.pitch),
            math.degrees(message.yaw),
            math.degrees(message.roll),
        )

    @property
    def attitude_sample(self) -> SimpleNamespace | None:
        sample = self._messages.latest("ATTITUDE")
        if sample is None:
            return None
        return copy_attitude_sample(
            attitude_sample_from_message(sample.message, sample.receipt_time_s)
        )

    @property
    def simulator_truth_pose(self) -> SimpleNamespace | None:
        sample = self._messages.latest("SIM_STATE")
        return simulator_truth_pose_from_sample(sample) if sample else None

    @property
    def attitude_target_debug(self) -> SimpleNamespace | None:
        target = self._diagnostics.attitude_target()
        if target is None:
            return None
        return SimpleNamespace(
            roll=target.roll,
            pitch=target.pitch,
            yaw=target.yaw,
            requested_yaw=target.requested_yaw,
            thrust=target.thrust,
            type_mask=target.type_mask,
            age_ms=(time.time() - target.timestamp_s) * 1000.0,
        )

    @property
    def nav_controller_output_debug(self) -> SimpleNamespace | None:
        sample = self._messages.latest("NAV_CONTROLLER_OUTPUT")
        if sample is None:
            return None
        message = sample.message
        return SimpleNamespace(
            nav_roll=getattr(message, "nav_roll", None),
            nav_pitch=getattr(message, "nav_pitch", None),
            nav_bearing=getattr(message, "nav_bearing", None),
            target_bearing=getattr(message, "target_bearing", None),
            wp_dist=getattr(message, "wp_dist", None),
            age_ms=(time.time() - sample.receipt_time_s) * 1000.0,
        )
