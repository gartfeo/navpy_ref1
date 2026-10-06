"""Per-vehicle AAS parameter read/write endpoints."""
from __future__ import annotations

import asyncio
import logging

from fastapi import APIRouter, HTTPException

from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.aas_params import AAS_PARAM_MAP, AAS_BOOL_FIELDS, AAS_INT_FIELDS
from gcs.backend.full_params import full_param_cache

log = logging.getLogger(__name__)
router = APIRouter()


@router.get("/{sys_id}/params")
async def get_vehicle_params(sys_id: int):
    """Read AAS parameters from a connected vehicle via MAVLink."""
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")

    loop = asyncio.get_event_loop()
    params = {}
    for field_name, mav_name in AAS_PARAM_MAP.items():
        value = await loop.run_in_executor(
            None, entry.vehicle.get_parameter, mav_name,
        )
        if value is not None:
            if field_name in AAS_BOOL_FIELDS:
                params[field_name] = value > 0
            elif field_name in AAS_INT_FIELDS:
                params[field_name] = int(value)
            else:
                params[field_name] = value
    return {"sys_id": sys_id, "params": params}


@router.put("/{sys_id}/params")
async def set_vehicle_params(sys_id: int, body: dict):
    """Write AAS parameters to a connected vehicle via MAVLink."""
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")

    loop = asyncio.get_event_loop()
    results = {}
    params = body.get("params", body)
    written_mav_names: list[str] = []
    for field_name, value in params.items():
        mav_name = AAS_PARAM_MAP.get(field_name)
        if not mav_name:
            continue
        if field_name in AAS_BOOL_FIELDS:
            mav_value = 1.0 if value else 0.0
        else:
            mav_value = float(value)
        ok = await loop.run_in_executor(
            None, entry.vehicle.set_parameter, mav_name, mav_value,
        )
        results[field_name] = ok
        written_mav_names.append(mav_name)

    # Cross-route invalidation (D6/D7): AAS writes touch params that may
    # also live in the full-param snapshot. Invalidate even on partial
    # failure — the autopilot may have stored a value before the echo
    # timed out.
    if written_mav_names:
        full_param_cache.invalidate_keys(sys_id, written_mav_names)
    return {"sys_id": sys_id, "results": results}
