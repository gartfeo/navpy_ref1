"""The position fixes one vehicle reported between two telemetry broadcasts.

A broadcast snapshot carries only the newest position. The map trail drew
straight lines between snapshots, so at SITL speedup (5 Hz wall broadcast,
10x sim) its points were ~50 m apart and could miss a dock the aircraft flew
through by ~25 m. The telemetry loop drains this buffer each broadcast and
sends every fix as the vehicle's ``path`` whenever it broadcasts that vehicle;
an unchanged vehicle keeps its fixes for its next broadcast.
"""
from __future__ import annotations

import collections

from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_AUTOPILOT1, MAVLink_message
from navpy.core.kernel import runtime_time as rt

# Fixes kept between two drains. One 0.2 s broadcast at 10x speedup holds
# about 10 GLOBAL_POSITION_INT fixes; the bound only stops growth while a
# vehicle is not broadcast (standing still, or no WebSocket loop), keeping the
# newest fixes.
POSITION_PATH_MAX_FIXES = 200


class PositionPath:
    """Collect GLOBAL_POSITION_INT fixes on the reader thread; drain on the loop."""

    def __init__(self, sys_id: int, max_fixes: int = POSITION_PATH_MAX_FIXES) -> None:
        self._sys_id = sys_id
        self._fixes: collections.deque = collections.deque(maxlen=max_fixes)
        self._lock = rt.Lock()

    def on_global_position(self, msg: MAVLink_message) -> None:
        # In GCS mode the shared link delivers every system's traffic to every
        # vehicle: keep only this vehicle's own autopilot. Messages without a
        # MAVLink source (test / non-mav paths) are kept.
        if hasattr(msg, "get_srcSystem") and (
            msg.get_srcSystem() != self._sys_id
            or msg.get_srcComponent() != MAV_COMP_ID_AUTOPILOT1
        ):
            return
        fix = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1e3,
            "alt_rel": msg.relative_alt / 1e3,
        }
        with self._lock:
            self._fixes.append(fix)

    def drain(self) -> list[dict]:
        """Every fix since the last drain, oldest first."""
        with self._lock:
            fixes = list(self._fixes)
            self._fixes.clear()
        return fixes
