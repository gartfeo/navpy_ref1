"""Build the ArduPilot rally table that sends every RTL to the default delivery hub.

ArduPlane RTL flies to the nearest rally point instead of HOME and loiters there
(``AP_Rally::calc_best_rally_or_home_location``). One rally point at the DDH
therefore turns every RTL (NavPy's escalation, the Lua watchdog, the autopilot's
own failsafes) into "return to the hub and circle it" (owner ruling 2026-11-09).
The table travels as ``MAV_MISSION_TYPE_RALLY`` through the mission protocol.
"""
from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_RALLY_POINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAV_MISSION_TYPE_RALLY,
    MAVLink_mission_item_int_message,
)

#: ArduPilot stores a rally altitude as int16 metres (MissionItemProtocol_Rally
#: refuses anything outside it with MAV_MISSION_INVALID_PARAM7).
_RALLY_ALT_MIN_M = -32768
_RALLY_ALT_MAX_M = 32767

#: Rally parameters written with the DDH rally point.
#: RALLY_INCL_HOME=0: HOME is never a candidate, so an RTL near the launch point
#: still goes to the hub. RALLY_LIMIT_KM=0: "the closest rally point is always
#: used" (ArduPilot's documented meaning of 0). Plane's default is 5 km, and past
#: the limit AP_Rally::find_nearest_rally_point falls back to HOME, so a hub
#: farther than 5 km from where RTL starts would be ignored. The limit guards
#: against a leftover rally point from another site; here every mission upload
#: replaces or clears the table, so no stale point remains to guard against.
RALLY_PARAM_WRITES: tuple[tuple[str, float], ...] = (
    ("RALLY_INCL_HOME", 0.0),
    ("RALLY_LIMIT_KM", 0.0),
)


def rally_altitude_m(altitude_m: float) -> int:
    """The rally altitude ArduPilot stores: whole metres above HOME.

    Raises ValueError when it does not fit ArduPilot's int16 field, so the
    caller reports it rather than sending an item the vehicle would refuse.
    """
    alt = int(round(float(altitude_m)))
    if not _RALLY_ALT_MIN_M <= alt <= _RALLY_ALT_MAX_M:
        raise ValueError(f"rally altitude {altitude_m} m is outside ArduPilot's int16 range")
    return alt


def build_rally_items(hub: dict, altitude_m: float, target_system: int) -> list:
    """One ``MAV_CMD_NAV_RALLY_POINT`` at the DDH, ``altitude_m`` above HOME.

    The frame is relative to HOME, the same reference as ``RTL_ALTITUDE``, which
    the upload sets to the same assignment altitude.
    """
    return [
        MAVLink_mission_item_int_message(
            target_system=target_system,
            target_component=0,
            seq=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_NAV_RALLY_POINT,
            current=0,
            autocontinue=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x=int(round(hub["lat"] * 1e7)),
            y=int(round(hub["lon"] * 1e7)),
            z=float(rally_altitude_m(altitude_m)),
            mission_type=MAV_MISSION_TYPE_RALLY,
        )
    ]
