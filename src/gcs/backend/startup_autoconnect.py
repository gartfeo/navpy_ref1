"""Backend startup auto-connect to this chat's running SITL (no browser needed)."""
from __future__ import annotations

import asyncio
import logging

from gcs.backend.sim_mode_params import ensure_sim_mode_params
from gcs.backend.vehicle_manager import vehicle_mgr

log = logging.getLogger(__name__)

STARTUP_AUTOCONNECT_ATTEMPTS = 24
STARTUP_AUTOCONNECT_SCAN_TIMEOUT_S = 5.0
STARTUP_AUTOCONNECT_RETRY_S = 2.0


def _start_esp32_sim() -> None:
    """Start the ESP32 sim as the connect and discover routes do, so Launch is
    enabled without a manual connect; a failure is logged, never fatal."""
    try:
        from gcs.backend.routes.control import auto_start_esp32_sim
        auto_start_esp32_sim()
    except Exception as exc:
        log.warning("Startup auto-connect ESP32 simulator start failed: %s", exc)


def _ensure_sim_mode_params(sys_id: int) -> None:
    """Give this chat's own SITL vehicle the sim mode-switch params, once per
    startup and before NavPy starts; a failure is logged, never fatal."""
    entry = vehicle_mgr.get_vehicle(sys_id)
    if entry is None:
        return
    try:
        ensure_sim_mode_params(entry.vehicle, sys_id)
    except Exception as exc:  # noqa: BLE001 - connecting goes on
        log.warning("Vehicle %d: sim mode params check failed: %s", sys_id, exc)


async def startup_autoconnect() -> None:
    """In launcher-managed sim mode, connect to THIS chat's SITL on startup so
    vehicles appear without waiting for a browser.

    Uses the chat's GCS monitor UDP port — the port the running SITL's router
    publishes — and retries quietly until SITL is up. Idempotent: ``add_vehicle``
    returns the existing entry if a vehicle is already connected (e.g. a browser
    also auto-connected), so this never double-connects.
    """
    from gcs.backend import instance_ports as ip
    from gcs.backend.settings_store import settings_store

    n = ip.chat_index()
    if n is None:
        return
    settings = settings_store.get()
    if not settings.simulation.sim_mode or not settings.connection.auto_connect:
        return

    _start_esp32_sim()
    device = ip.monitor_device(n)
    expected_sysids = set(ip.sysids_for_chat(n))
    connected_sysids = {
        sid for sid in vehicle_mgr.vehicles
        if sid in expected_sysids
    }
    navpy_started_sysids: set[int] = set()
    mode_params_checked: set[int] = set()
    loop = asyncio.get_running_loop()
    for _ in range(STARTUP_AUTOCONNECT_ATTEMPTS):  # ~2 min of quiet retries while SITL boots
        try:
            entries = await loop.run_in_executor(
                None, vehicle_mgr.discover_and_connect,
                device, STARTUP_AUTOCONNECT_SCAN_TIMEOUT_S,
            )
        except Exception as exc:
            log.debug("Startup auto-connect scan failed: %s", exc)
            entries = []

        for entry in entries:
            sys_id = getattr(entry, "sys_id", None)
            if sys_id in expected_sysids:
                connected_sysids.add(sys_id)
        connected_sysids.update(
            sid for sid in vehicle_mgr.vehicles
            if sid in expected_sysids
        )

        to_start = sorted(connected_sysids - navpy_started_sysids)
        for sys_id in sorted(connected_sysids - mode_params_checked):
            mode_params_checked.add(sys_id)
            await asyncio.to_thread(_ensure_sim_mode_params, sys_id)
        if to_start:
            try:
                from gcs.backend.routes.navpy_sim import auto_start_navpy_sim
                for sys_id in to_start:
                    if await asyncio.to_thread(auto_start_navpy_sim, sys_id):
                        navpy_started_sysids.add(sys_id)
            except Exception as exc:
                log.warning("Startup auto-connect NavPy start failed: %s", exc)

        missing = expected_sysids - connected_sysids
        if connected_sysids and not missing:
            log.info(
                "Startup auto-connect: %d/%d vehicle(s) on %s",
                len(connected_sysids), len(expected_sysids), device,
            )
            return
        await asyncio.sleep(STARTUP_AUTOCONNECT_RETRY_S)

    if connected_sysids:
        log.info(
            "Startup auto-connect: %d/%d vehicle(s) on %s; missing sys_ids %s",
            len(connected_sysids), len(expected_sysids), device, sorted(expected_sysids - connected_sysids),
        )
    else:
        log.info("Startup auto-connect: no vehicles found on %s", device)
