"""Vehicle connection management endpoints."""
from __future__ import annotations

import asyncio
import logging
from typing import Optional

from fastapi import APIRouter

from gcs.backend.models import VehicleStatus
from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.diagnostics import emit

log = logging.getLogger(__name__)
router = APIRouter()


@router.get("", response_model=list[VehicleStatus])
async def list_vehicles():
    """List all connected vehicles with their status."""
    snapshots = vehicle_mgr.get_all_snapshots()
    return [VehicleStatus(**s) for s in snapshots]


@router.post("/connect")
async def connect_vehicle(device: str, sys_id: int, name: Optional[str] = None):
    """Connect to a new vehicle."""
    try:
        emit("vehicle_connect_started", source="backend", sys_id=sys_id)
        loop = asyncio.get_event_loop()
        entry = await loop.run_in_executor(
            None, vehicle_mgr.add_vehicle, device, sys_id, name,
        )
        # Auto-start NavPy sim if sim mode is on
        from gcs.backend.routes.navpy_sim import auto_start_navpy_sim
        await asyncio.to_thread(auto_start_navpy_sim, entry.sys_id)
        # Auto-start ESP32 simulator if sim mode + container launch
        from gcs.backend.routes.control import auto_start_esp32_sim
        auto_start_esp32_sim()
        emit("vehicle_connect_finished", source="backend", sys_id=entry.sys_id,
             outcome="success")
        return {"status": "connected", "sys_id": entry.sys_id, "name": entry.name}
    except Exception as e:
        emit("vehicle_connect_finished", source="backend", sys_id=sys_id,
             outcome="failure", error_code=e.__class__.__name__)
        from fastapi import HTTPException
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/scan")
async def scan_vehicles(device: str, timeout: float = 5.0):
    """Scan a MAVLink connection for heartbeats without connecting.

    Returns found vehicles as available. Use /connect to connect individually.
    """
    try:
        loop = asyncio.get_event_loop()
        found = await loop.run_in_executor(
            None, vehicle_mgr.scan, device, timeout,
        )
        emit("vehicle_scan_finished", source="backend", outcome="success",
             vehicle_count=len(found))
        return {
            "status": "ok",
            "found": len(found),
            "vehicles": found,
        }
    except Exception as e:
        from fastapi import HTTPException
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/discover")
async def discover_vehicles(device: str, timeout: float = 5.0):
    """Scan a MAVLink connection for heartbeats and auto-connect all found vehicles."""
    try:
        loop = asyncio.get_event_loop()
        entries = await loop.run_in_executor(
            None, vehicle_mgr.discover_and_connect, device, timeout,
        )
        # Mission probes now run asynchronously off the connect path (see
        # VehicleManager._probe_mission_async), so mission validity is not known
        # at discover-return time. The old inline fleet cross-validation here was
        # log-only and unused by the frontend; per-vehicle mission state now
        # flows in via telemetry snapshots as each probe completes, and the
        # launch path validates authoritatively before launch.
        # Auto-start NavPy sim for each discovered vehicle
        from gcs.backend.routes.navpy_sim import auto_start_navpy_sim
        for entry in entries:
            await asyncio.to_thread(auto_start_navpy_sim, entry.sys_id)
        # Auto-start ESP32 simulator if sim mode + container launch
        from gcs.backend.routes.control import auto_start_esp32_sim
        auto_start_esp32_sim()
        emit("vehicle_discovery_finished", source="backend", outcome="success",
             vehicle_count=len(entries), sys_ids=[entry.sys_id for entry in entries])
        return {
            "status": "ok",
            "found": len(entries),
            "vehicles": [
                {"sys_id": e.sys_id, "name": e.name} for e in entries
            ],
        }
    except Exception as e:
        from fastapi import HTTPException
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/disconnect")
async def disconnect_vehicle(sys_id: int):
    """Disconnect a vehicle and stop its GCS-managed NavPy companion.

    Explicit operator disconnect releases the vehicle's sim companion:
    stop_instance kills the subprocess and clears auto-managed intent, so an
    orphaned companion cannot keep its serial0 link and a later re-discovery
    cannot silently resume restarts. Gated on `removed` so an unknown sys_id
    stays a full no-op. Transient link maintenance uses reconnect_vehicle,
    which never goes through this route.

    Intent is cleared synchronously BEFORE the first await: the watchdog
    runs on this loop and snapshots the managed set, so clearing here means
    no interleaving can copy the sysid into its wanted list and restart the
    dying process as an unmanaged orphan. Only the blocking process kill
    (up to ~7s terminate+kill wait) is offloaded to the executor so the
    event loop never stalls. `stopped` means a tracked instance record was
    removed — the process may already have exited (crashed companion);
    either way the status changed, so broadcast.
    """
    removed = vehicle_mgr.remove_vehicle(sys_id)
    from gcs.backend import navpy_sim_runtime as runtime
    from gcs.backend.routes.navpy_sim import broadcast_sim_status
    for rid in removed:
        await asyncio.to_thread(runtime.clear_managed_intent, rid)
    loop = asyncio.get_event_loop()
    stopped = [
        rid for rid in removed
        if await loop.run_in_executor(None, runtime.stop_instance_process, rid)
    ]
    for rid in stopped:
        emit("navpy_operation_finished", source="backend", sys_id=rid,
             outcome="stopped")
    if stopped:
        await broadcast_sim_status()
    emit("vehicle_disconnect_finished", source="backend", sys_id=sys_id,
         sys_ids=removed, outcome="success")
    return {"status": "disconnected", "sys_id": sys_id, "removed": removed}
