"""Async telemetry broadcast loop.

Polls VehicleManager snapshots and broadcasts changes via WebSocket.
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import TYPE_CHECKING

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.launch_controller import launch_controller
from gcs.backend.settings_store import settings_store
from gcs.backend.diagnostics import emit

if TYPE_CHECKING:
    from gcs.backend.vehicle_manager import VehicleManager

log = logging.getLogger(__name__)


# Ignore sub-threshold RC jitter so a live transmitter doesn't rebroadcast
# telemetry every cycle; real stick/switch movement during calibration is far
# larger than this.
_RC_CHANGE_THRESHOLD_US = 2


def _has_changed(prev: dict, curr: dict) -> bool:
    """Check if telemetry has meaningful change."""
    if curr.get("status_texts"):
        return True
    if _gimbals_changed(prev.get("gimbals"), curr.get("gimbals")):
        return True
    if _rc_channels_changed(prev.get("rc_channels"), curr.get("rc_channels")):
        return True
    for key in ("battery", "voltage", "current", "mode", "armed", "lat",
                "lon", "alt", "heading", "ground_speed", "gps_fix",
                "gps_sats", "gps_hacc", "link_ok", "link_quality",
                "companion_ok", "companion_status", "mission_progress", "roll", "pitch",
                "prearm_ok", "prearm_check_state", "is_probing", "mission_uploaded",
                "mission_total", "mission_download_progress", "swarm"):
        p, c = prev.get(key), curr.get(key)
        if p is None and c is None:
            continue
        if p is None or c is None:
            return True
        if isinstance(p, float) and isinstance(c, float):
            if key in ("lat", "lon"):
                if abs(p - c) > 1e-6:
                    return True
            elif abs(p - c) > 0.5:
                return True
        elif p != c:
            return True
    return False


def _gimbals_changed(prev, curr) -> bool:
    prev = prev or {}
    curr = curr or {}
    if set(prev.keys()) != set(curr.keys()):
        return True

    for device_id in curr:
        p = prev.get(device_id) or {}
        c = curr.get(device_id) or {}
        for key in (
            "device_id",
            "flags",
            "failure_flags",
            "stale",
            "fov_h_rad",
            "fov_v_rad",
            "optics_stale",
            "zoom_level",
        ):
            if p.get(key) != c.get(key):
                return True
        if list(p.get("q") or []) != list(c.get("q") or []):
            return True
    return False


def _rc_channels_changed(prev, curr) -> bool:
    """True when any RC channel moved by more than the jitter threshold.

    Drives live broadcast during radio calibration. Appearance/disappearance of
    the RC link, or a change in channel count, always counts as a change.
    """
    if prev is None and curr is None:
        return False
    if prev is None or curr is None:
        return True
    pc = prev.get("channels") or []
    cc = curr.get("channels") or []
    if len(pc) != len(cc):
        return True
    for a, b in zip(pc, cc):
        if a is None and b is None:
            continue
        if a is None or b is None:
            return True
        if abs(a - b) > _RC_CHANGE_THRESHOLD_US:
            return True
    return False


def _check_disarm_after_guided(prev: dict, curr: dict) -> bool:
    """Detect disarm following an armed sample whose mode was GUIDED."""
    return (prev.get("armed") is True
            and curr.get("armed") is False
            and prev.get("mode") == "GUIDED")


class TelemetryLoop:
    """Owns the async telemetry broadcast, heartbeat polling, and change detection."""

    def __init__(self, vehicle_mgr: VehicleManager):
        self._mgr = vehicle_mgr
        self._task: asyncio.Task | None = None
        self._stop = asyncio.Event()

    async def start(self):
        """Start the async telemetry broadcast loop."""
        self._stop.clear()
        self._task = asyncio.create_task(self._loop())
        log.info("Telemetry broadcast loop started")

    async def stop(self):
        """Stop the telemetry loop."""
        self._stop.set()
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        log.info("Telemetry broadcast loop stopped")

    async def _loop(self):
        """Periodically broadcast vehicle telemetry to all WS clients."""
        last_broadcast: dict[int, dict] = {}
        last_active: set[int] = set()
        last_seen: set[int] = set()
        last_sim_broadcast: float = 0.0
        while not self._stop.is_set():
            try:
                snapshots = self._mgr.get_all_snapshots()
                active_ids = {s["sys_id"] for s in snapshots}

                # Feed telemetry to launch controller when prepared or running
                if launch_controller.is_prepared or launch_controller.is_running:
                    for snap in snapshots:
                        launch_controller.update_telemetry(
                            snap["sys_id"],
                            snap.get("alt_rel") or 0.0,
                            snap.get("armed", False),
                            snap.get("climb") or 0.0,
                            snap.get("throttle") or 0.0,
                        )

                # Only broadcast if there's meaningful change
                changed = []
                disarm_events = []
                cal_step_events = []
                for snap in snapshots:
                    sid = snap["sys_id"]
                    prev = last_broadcast.get(sid)
                    if prev is None or prev.get("link_ok") != snap.get("link_ok"):
                        emit("vehicle_link_state", source="backend", sys_id=sid,
                             outcome="connected" if snap.get("link_ok") else "lost")
                    if prev is not None and _check_disarm_after_guided(prev, snap):
                        disarm_events.append({
                            "type": "disarm_after_guided",
                            "sys_id": sid,
                            "lat": snap.get("lat"),
                            "lon": snap.get("lon"),
                            "alt": snap.get("alt"),
                            "ts": time.time(),
                        })
                    # Accel-cal prompts/results are relayed as discrete events,
                    # independent of telemetry change detection, so the wizard
                    # advances even when nothing else about the vehicle changed.
                    for evt in snap.get("accel_cal_events", []):
                        cal_step_events.append({
                            "type": "accel_cal_step",
                            "sys_id": sid,
                            "status": evt.get("status"),
                            "step": evt.get("step"),
                            "prompt_text": evt.get("prompt_text"),
                            "ts": time.time(),
                        })
                    if prev is None or _has_changed(prev, snap):
                        changed.append(snap)
                        last_broadcast[sid] = snap

                # Prune last_broadcast for removed vehicles
                for sid in last_active - active_ids:
                    last_broadcast.pop(sid, None)
                    emit("vehicle_final_state", source="backend", sys_id=sid,
                         outcome="disconnected")

                seen_ids = set(self._mgr.get_seen_ids())

                # Broadcast disarm notifications before telemetry
                for evt in disarm_events:
                    await ws_manager.broadcast(evt)

                # Broadcast accel-cal wizard steps before telemetry
                for evt in cal_step_events:
                    await ws_manager.broadcast(evt)

                # Broadcast when vehicle data changed OR the active/seen set changed
                if changed or active_ids != last_active or seen_ids != last_seen:
                    last_active = active_ids
                    last_seen = seen_ids
                    await ws_manager.broadcast({
                        "type": "telemetry",
                        "vehicles": changed,
                        "active_ids": sorted(active_ids),
                        "seen_ids": sorted(seen_ids),
                        "ts": time.time(),
                    })

                # Periodic NavPy sim status broadcast (~2s) when sim_mode on
                now = time.time()
                if now - last_sim_broadcast >= 2.0:
                    settings = settings_store.get()
                    if settings.simulation.sim_mode:
                        from gcs.backend import navpy_sim_runtime
                        from gcs.backend.routes.navpy_sim import broadcast_sim_status
                        await asyncio.to_thread(
                            navpy_sim_runtime.ensure_auto_navpy_sim_running,
                            active_ids,
                            settings,
                        )
                        await broadcast_sim_status()
                    last_sim_broadcast = now
            except Exception as e:
                log.error("Telemetry loop error: %s", e)

            await asyncio.sleep(settings_store.get().connection.ws_broadcast_interval_s)
