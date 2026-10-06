"""Battery and GPS telemetry views."""
from __future__ import annotations

from navpy.modules.vehicle.message_store import MessageStore


class PowerGpsTelemetry:
    def __init__(self, message_store: MessageStore) -> None:
        self._messages = message_store

    @property
    def battery_level(self) -> int | None:
        battery = self._messages.message("BATTERY_STATUS")
        return battery.battery_remaining if battery else None

    @property
    def battery_voltage(self) -> float | None:
        status = self._messages.message("SYS_STATUS")
        if status and getattr(status, "voltage_battery", 0) > 0:
            return status.voltage_battery / 1000.0
        battery = self._messages.message("BATTERY_STATUS")
        if battery and battery.voltages:
            total = sum(value for value in battery.voltages if 0 < value < 65535)
            return total / 1000.0 if total > 0 else None
        return None

    @property
    def battery_current(self) -> float | None:
        status = self._messages.message("SYS_STATUS")
        if status and getattr(status, "current_battery", -1) >= 0:
            return status.current_battery / 100.0
        battery = self._messages.message("BATTERY_STATUS")
        if battery and getattr(battery, "current_battery", -1) >= 0:
            return battery.current_battery / 100.0
        return None

    @property
    def gps_fix_type(self) -> int | None:
        gps = self._messages.message("GPS_RAW_INT")
        return gps.fix_type if gps else None

    @property
    def gps_satellites(self) -> int | None:
        gps = self._messages.message("GPS_RAW_INT")
        return gps.satellites_visible if gps else None

    @property
    def gps_hacc(self) -> float | None:
        gps = self._messages.message("GPS_RAW_INT")
        if not gps:
            return None
        horizontal_accuracy = getattr(gps, "h_acc", None)
        if horizontal_accuracy is not None and horizontal_accuracy > 0:
            return horizontal_accuracy / 1000.0
        eph = getattr(gps, "eph", None)
        return None if eph is None or eph == 65535 else eph / 100.0
