"""Listener for onboard magnetometer-calibration telemetry.

Subscribes to MAG_CAL_PROGRESS / MAG_CAL_REPORT on each VehicleMav and
rebroadcasts a unified ``compass_cal_progress`` message to WebSocket clients
so the GCS can render per-compass progress, success, and failure.

Callbacks fire in MavBus reader-loop daemon threads, so broadcasts are
scheduled onto the asyncio loop via run_coroutine_threadsafe() — the same
bridge used by TaskConfirmListener / TaskAssignListener.

Ported wholesale from feat/uav-compass-cal (INTEG-04, D-04) — no dev-side
collision.
"""
from __future__ import annotations

import asyncio
import logging

from navpy.modules.vehicle.vehicle_mav import VehicleMav
from gcs.backend.broadcast import manager as ws_manager

log = logging.getLogger(__name__)


def _progress_payload(sys_id: int, msg) -> dict:
    """Build a ``compass_cal_progress`` payload from a MAG_CAL_PROGRESS msg."""
    return {
        "type": "compass_cal_progress",
        "sys_id": sys_id,
        "compass_id": int(getattr(msg, "compass_id", 0)),
        "pct": int(getattr(msg, "completion_pct", 0)),
        "cal_status": int(getattr(msg, "cal_status", 0)),
        "report": False,
    }


def _report_payload(sys_id: int, msg) -> dict:
    """Build a ``compass_cal_progress`` payload from a MAG_CAL_REPORT msg.

    A report marks the compass terminal (success/failure); ``pct`` is pinned to
    100 so a finished compass never shows a stale sub-100 bar, and ``fitness`` /
    ``autosaved`` carry the result detail the UI surfaces.
    """
    return {
        "type": "compass_cal_progress",
        "sys_id": sys_id,
        "compass_id": int(getattr(msg, "compass_id", 0)),
        "pct": 100,
        "cal_status": int(getattr(msg, "cal_status", 0)),
        "report": True,
        "fitness": float(getattr(msg, "fitness", 0.0)),
        "autosaved": int(getattr(msg, "autosaved", 0)),
    }


class CompassCalListener:
    """Rebroadcasts MAG_CAL telemetry from VehicleMav connections to the UI."""

    def __init__(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def register_vehicle(self, sys_id: int, vehicle: VehicleMav) -> None:
        """Register MAG_CAL callbacks on a VehicleMav instance."""
        vehicle.on_message(
            "MAG_CAL_PROGRESS",
            lambda msg, sid=sys_id: self._on_progress(sid, msg),
        )
        vehicle.on_message(
            "MAG_CAL_REPORT",
            lambda msg, sid=sys_id: self._on_report(sid, msg),
        )
        log.info("CompassCalListener registered for vehicle %d", sys_id)

    def _on_progress(self, sys_id: int, msg) -> None:
        self._broadcast(_progress_payload(sys_id, msg))

    def _on_report(self, sys_id: int, msg) -> None:
        payload = _report_payload(sys_id, msg)
        log.info(
            "Compass cal report V%d compass=%d status=%d fitness=%.2f",
            sys_id, payload["compass_id"], payload["cal_status"],
            payload["fitness"],
        )
        self._broadcast(payload)

    def _broadcast(self, payload: dict) -> None:
        """Thread-safe broadcast via the asyncio event loop."""
        asyncio.run_coroutine_threadsafe(ws_manager.broadcast(payload), self._loop)
