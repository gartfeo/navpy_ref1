"""Feed the vehicle's position stream to the sim dock contact sensor."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import replace
from typing import Any, Optional

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.subscription import Subscription
from navpy.sim.world.dock_contact_sensor import PositionSample, SimDockContactSensor

POSITION_MESSAGE = "GLOBAL_POSITION_INT"
ATTITUDE_MESSAGE = "ATTITUDE"


def position_sample(message: Any) -> Optional[PositionSample]:
    """Aircraft time, AMSL position, speed and height over home of one
    GLOBAL_POSITION_INT."""
    try:
        values = (
            message.time_boot_ms, message.lat, message.lon, message.alt,
            message.vx, message.vy, message.vz,
        )
    except AttributeError:
        return None
    if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
        return None
    boot_ms, lat_e7, lon_e7, alt_mm, vx_cms, vy_cms, vz_cms = values
    relative_mm = getattr(message, "relative_alt", None)
    relative_alt_m = (
        relative_mm / 1e3
        if isinstance(relative_mm, int) and not isinstance(relative_mm, bool)
        else None
    )
    return PositionSample(
        float(boot_ms & 0xFFFFFFFF) * 1e-3,
        Location(lat_e7 / 1e7, lon_e7 / 1e7, alt_mm / 1e3, is_absolute=True),
        math.sqrt(vx_cms ** 2 + vy_cms ** 2 + vz_cms ** 2) / 100.0,
        relative_alt_m=relative_alt_m,
    )


def attitude_sample(message: Any) -> Optional[tuple[float, Attitude]]:
    """Aircraft time and finite attitude (degrees) of one ATTITUDE, else None."""
    try:
        boot_ms = message.time_boot_ms
        angles = (message.roll, message.pitch, message.yaw)
    except AttributeError:
        return None
    if isinstance(boot_ms, bool) or not isinstance(boot_ms, int):
        return None
    if any(
        isinstance(angle, bool)
        or not isinstance(angle, (int, float))
        or not math.isfinite(angle)
        for angle in angles
    ):
        return None
    roll, pitch, yaw = angles
    return (
        float(boot_ms & 0xFFFFFFFF) * 1e-3,
        Attitude.from_radians(pitch=pitch, yaw=yaw, roll=roll),
    )


class SimDockContactFeed:
    """Feed every telemetry position update to the sensor.

    Subscribed to the vehicle's own position stream, so sampling is
    independent of detector visibility and of the detection source mode.
    The newest ATTITUDE only labels a position sample for display (the
    closest-approach ghost); contact never depends on it.
    """

    def __init__(
        self,
        sensor: SimDockContactSensor,
        subscribe: Callable[[str, Callable[[Any], None]], Subscription],
        attitude_max_age_s: float,
    ) -> None:
        self._sensor = sensor
        self._attitude_max_age_s = attitude_max_age_s
        self._attitude: Optional[tuple[float, Attitude]] = None
        self._subscriptions = (
            subscribe(ATTITUDE_MESSAGE, self.on_attitude),
            subscribe(POSITION_MESSAGE, self.on_message),
        )

    def on_attitude(self, message: Any) -> None:
        # One reference swap: the position callback reads it whole.
        self._attitude = attitude_sample(message)

    def on_message(self, message: Any) -> None:
        sample = position_sample(message)
        if sample is not None:
            self._sensor.sample(self._with_attitude(sample))

    def _with_attitude(self, sample: PositionSample) -> PositionSample:
        """Label the sample with an attitude of about the same aircraft time.

        Within the bound on either side; a stale, future (clock reset) or
        missing attitude labels nothing.
        """
        attitude = self._attitude
        if attitude is None:
            return sample
        attitude_time_s, value = attitude
        if abs(sample.aircraft_time_s - attitude_time_s) > self._attitude_max_age_s:
            return sample
        return replace(sample, attitude=value)

    def close(self) -> None:
        for subscription in self._subscriptions:
            subscription.cancel()


__all__ = [
    "ATTITUDE_MESSAGE",
    "POSITION_MESSAGE",
    "SimDockContactFeed",
    "attitude_sample",
    "position_sample",
]
