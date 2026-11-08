"""Companion liveness from the companion heartbeats each MAVLink bus records.

The companion heartbeat carries the same system id as the aircraft's own;
MavBus keeps the two apart by MAV_TYPE (MAV_TYPE_ONBOARD_CONTROLLER lands in
``companion_heartbeats``, everything else in ``heartbeats``), so the system
id is a safe key here.
"""
from __future__ import annotations

from typing import Iterable, Optional

from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus
from gcs.backend.vehicle_status import COMPANION_DOWN_WINDOW_S, COMPANION_OK_WINDOW_S


def newest_companion_heartbeat(buses: Iterable[MavBus], sys_id: int) -> Optional[float]:
    """Most recent companion-heartbeat time for *sys_id* across *buses*."""
    newest: Optional[float] = None
    for bus in buses:
        ts = bus.companion_heartbeats.get(sys_id)
        if ts is not None and (newest is None or ts > newest):
            newest = ts
    return newest


def heartbeat_within(buses: Iterable[MavBus], sys_id: int, timeout: float, now: float) -> bool:
    """Whether any bus saw a companion heartbeat for *sys_id* within *timeout*."""
    for bus in buses:
        ts = bus.companion_heartbeats.get(sys_id)
        if ts is not None and now - ts <= timeout:
            return True
    return False


def status_from_newest(newest: Optional[float], now: float) -> str:
    """Tri-state companion liveness: ``"ok"`` | ``"checking"`` | ``"down"``.

    See COMPANION_OK_WINDOW_S / COMPANION_DOWN_WINDOW_S for the rationale: a
    brief gap in the 4 Hz companion heartbeat on the shared lossy link is
    ``"checking"`` (transient, not faulty) rather than flapping straight to
    ``"down"``.
    """
    if newest is None:
        return "down"
    age = now - newest
    if age <= COMPANION_OK_WINDOW_S:
        return "ok"
    if age <= COMPANION_DOWN_WINDOW_S:
        return "checking"
    return "down"
