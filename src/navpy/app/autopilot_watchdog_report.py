"""The autopilot's NavPy watchdog acting, as NavPy sees it (docs/design/navpy_recovery.md, step 8).

``scripts/lua/nav_watchdog.lua`` holds the UAV when NavPy's heartbeat stops reaching the autopilot. It reports its
stage as NAMED_VALUE_FLOAT ``NAVWD`` once a second and each action as a "NavPy watchdog: ..." status text, both
broadcast to every link, NavPy's included. A NavPy that is alive while its heartbeat does not get through (or one
restarted during the hold) sees them: each stage change into or out of a hold or RTL, and each action text, is a
recovery report (``ILogger.recovery``, source ``autopilot``), so the eval counts the autopilot's recovery too (Codex
review of step 8, finding 3). It runs on the MAVLink reader thread; a recovery report never blocks.
"""

from __future__ import annotations

import math

from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_AUTOPILOT1

from navpy.core.kernel.logging.logger_api import ILogger
from navpy.core.ports.vehicle_contract import IVehicle

NAVWD_NAME = "NAVWD"
LUA_TEXT_PREFIX = "NavPy watchdog: "
SOURCE = "autopilot"
#: nav_watchdog.lua's STATE_* values; HOLD and RTL are its recoveries.
NAVWD_STATES = {-1: "off", 0: "watching", 1: "hold", 2: "RTL"}
NAVWD_ACTING = frozenset({1, 2})


class AutopilotWatchdogReport:
    def __init__(self, logger: ILogger, target_system: int) -> None:
        self._logger = logger
        self._target_system = target_system
        self._state: int | None = None      # the reader thread's only

    def on_named_value(self, message: object) -> None:
        if not self._own_autopilot(message) or _text(getattr(message, "name", "")) != NAVWD_NAME:
            return
        value = getattr(message, "value", None)
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return
        state = int(round(value))
        previous, self._state = self._state, state
        if state != previous and (state in NAVWD_ACTING or previous in NAVWD_ACTING):
            self._logger.recovery(f"autopilot NavPy watchdog: {NAVWD_STATES.get(state, state)}", SOURCE)

    def on_status_text(self, message: object) -> None:
        text = _text(getattr(message, "text", ""))
        if self._own_autopilot(message) and text.startswith(LUA_TEXT_PREFIX):
            self._logger.recovery(f"autopilot {text}", SOURCE)

    def _own_autopilot(self, message: object) -> bool:
        try:
            return (message.get_srcSystem() == self._target_system
                    and message.get_srcComponent() == MAV_COMP_ID_AUTOPILOT1)
        except AttributeError:
            return False


def _text(value: object) -> str:
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    return str(value).rstrip("\x00")


class AutopilotWatchdogSubscriptions:
    def __init__(self, vehicle: IVehicle, logger: ILogger) -> None:
        report = AutopilotWatchdogReport(logger, vehicle.target_system)
        self._subscriptions = [vehicle.on_message("NAMED_VALUE_FLOAT", report.on_named_value)]
        try:
            self._subscriptions.append(vehicle.on_message("STATUSTEXT", report.on_status_text))
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        subscriptions, self._subscriptions = self._subscriptions, []
        for subscription in subscriptions:
            subscription.cancel()


__all__ = ["AutopilotWatchdogReport", "AutopilotWatchdogSubscriptions", "NAVWD_NAME"]
