"""Blocking fence download/readback composition for the HTTP endpoint."""
from __future__ import annotations

import asyncio
import logging
from fastapi import HTTPException
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from gcs.backend.planner.fence_builder import parse_fence_items
from gcs.backend.routes.mission_upload_helpers import read_fence_params

log = logging.getLogger("gcs.backend.routes.missions")


async def download_fence_snapshot(vehicle: VehicleMav, sys_id: int) -> dict:
    """Read the fence table and raw mode parameters off the event loop."""
    loop = asyncio.get_event_loop()
    try:
        items = await loop.run_in_executor(None, vehicle.download_fence)
    except Exception as exc:  # noqa: BLE001 - never surface a raw 500 trace
        log.warning("Fence download from vehicle %d failed: %s", sys_id, exc)
        raise HTTPException(status_code=502, detail=f"Fence download failed: {exc}") from exc

    # Blocking MAVLink reads: keep them off the event loop. An unreadable
    # parameter degrades this vehicle to "unknown mode"; it does not fail the
    # ring the download already returned.
    params = await loop.run_in_executor(None, lambda: read_fence_params(vehicle))

    items = items or []
    parsed = parse_fence_items(items)
    return {
        "vertices": parsed["vertices"],
        "exclusions": parsed["exclusions"],
        "total_items": len(items),
        "params": params,
        "params_readback": "ok" if params is not None else "failed",
    }
