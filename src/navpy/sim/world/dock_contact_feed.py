"""Feed the vehicle's position stream to the sim dock contact sensor."""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any, Optional

from navpy.core.kernel.location import Location
from navpy.core.ports.subscription import Subscription
from navpy.sim.world.dock_contact_sensor import PositionSample, SimDockContactSensor

POSITION_MESSAGE = "GLOBAL_POSITION_INT"


def position_sample(message: Any) -> Optional[PositionSample]:
    """Aircraft time, AMSL position and speed of one GLOBAL_POSITION_INT."""
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
    return PositionSample(
        float(boot_ms & 0xFFFFFFFF) * 1e-3,
        Location(lat_e7 / 1e7, lon_e7 / 1e7, alt_mm / 1e3, is_absolute=True),
        math.sqrt(vx_cms ** 2 + vy_cms ** 2 + vz_cms ** 2) / 100.0,
    )


class SimDockContactFeed:
    """Feed every telemetry position update to the sensor.

    Subscribed to the vehicle's own position stream, so sampling is
    independent of detector visibility and of the detection source mode.
    """

    def __init__(
        self,
        sensor: SimDockContactSensor,
        subscribe: Callable[[str, Callable[[Any], None]], Subscription],
    ) -> None:
        self._sensor = sensor
        self._subscription = subscribe(POSITION_MESSAGE, self.on_message)

    def on_message(self, message: Any) -> None:
        sample = position_sample(message)
        if sample is not None:
            self._sensor.sample(sample)

    def close(self) -> None:
        self._subscription.cancel()


__all__ = ["POSITION_MESSAGE", "SimDockContactFeed", "position_sample"]
