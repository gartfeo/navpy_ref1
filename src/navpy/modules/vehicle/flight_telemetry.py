"""Vehicle kinematics and air-data telemetry views."""
from __future__ import annotations

import numpy as np

from navpy.modules.common.models.wind import Wind
from navpy.modules.vehicle.message_store import MessageStore


class FlightTelemetry:
    def __init__(self, message_store: MessageStore) -> None:
        self._messages = message_store

    @property
    def velocity(self) -> tuple[float, float, float] | None:
        position = self._messages.message("GLOBAL_POSITION_INT")
        if not position:
            return None
        return position.vx / 100.0, position.vy / 100.0, position.vz / 100.0

    @property
    def ground_speed(self) -> float | None:
        hud = self._messages.message("VFR_HUD")
        return hud.groundspeed if hud else None

    @property
    def ground_speed_ned(self) -> np.ndarray | None:
        """Measured NED velocity; compass heading cannot supply ground track."""
        local = self._messages.message("LOCAL_POSITION_NED")
        if local:
            return np.array([local.vx, local.vy, local.vz], dtype=float)
        global_position = self._messages.message("GLOBAL_POSITION_INT")
        if global_position:
            return 0.01 * np.array(
                [global_position.vx, global_position.vy, global_position.vz],
                dtype=float,
            )
        return None

    @property
    def air_speed(self) -> float | None:
        hud = self._messages.message("VFR_HUD")
        return hud.airspeed if hud else None

    @property
    def climb_rate(self) -> float | None:
        hud = self._messages.message("VFR_HUD")
        return hud.climb if hud else None

    @property
    def throttle_pct(self) -> float | None:
        hud = self._messages.message("VFR_HUD")
        return hud.throttle if hud else None

    @property
    def wind(self) -> Wind | None:
        wind = self._messages.message("WIND")
        return Wind(wind.direction, wind.speed, wind.speed_z) if wind else None

    @property
    def heading(self) -> float | None:
        hud = self._messages.message("VFR_HUD")
        return getattr(hud, "heading", None) if hud else None
