"""Per-vehicle AAS parameter read/write endpoints."""
from __future__ import annotations

import asyncio
import logging
import math
from typing import Any

from fastapi import APIRouter, HTTPException

from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.aas_params import AAS_PARAM_MAP, AAS_BOOL_FIELDS, AAS_INT_FIELDS
from gcs.backend.full_params import full_param_cache
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.parameter_exchange import ExchangePolicy, ParameterExchange

log = logging.getLogger(__name__)
router = APIRouter()

#: Sends per parameter exchange (``docs/design/radio_budget.md``, todo 103): a
#: provisional budget from an estimated 44 % per-attempt failure at the
#: ``stress`` byte loss, 0.44^12 < 1e-4 if attempts are independent.
GCS_PARAM_ATTEMPTS = 12
#: A request's server budget when it names none: callers keep their HTTP
#: timeout above it. Lockstep counts it in sim time.
DEFAULT_PARAM_REQUEST_S = 55.0


def _policy(timeout_s: object) -> ExchangePolicy:
    raw = DEFAULT_PARAM_REQUEST_S if timeout_s is None else timeout_s
    try:
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise TypeError(raw)
        budget = float(raw)  # an oversized JSON integer overflows here
    except (TypeError, OverflowError):
        budget = math.nan
    if not math.isfinite(budget) or budget <= 0:
        raise HTTPException(status_code=400, detail="timeout_s must be a finite positive number")
    return ExchangePolicy(GCS_PARAM_ATTEMPTS, rt.monotonic() + budget, stop_on_silence=True)


def _result(exchange: ParameterExchange) -> bool | dict[str, Any]:
    """``true`` when confirmed; otherwise why (``unconfirmed`` may still be stored)."""
    if exchange.confirmed:
        return True
    return {"ok": False, "outcome": exchange.outcome.value,
            "reason": None if exchange.reason is None else exchange.reason.value}


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
    """Write AAS parameters to a connected vehicle via MAVLink.

    The body's optional ``timeout_s`` bounds the whole request (every lock and
    echo wait); each result is ``true`` or a failure with its outcome.
    """
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")

    policy = _policy(body.get("timeout_s"))
    loop = asyncio.get_event_loop()
    results: dict[str, bool | dict[str, Any]] = {}
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
        exchange = await loop.run_in_executor(
            None, entry.vehicle.write_parameter, mav_name, mav_value, policy,
        )
        results[field_name] = _result(exchange)
        if exchange.attempts:
            written_mav_names.append(mav_name)

    # Cross-route invalidation (D6/D7): AAS writes touch params that may
    # also live in the full-param snapshot. Invalidate every attempted write,
    # confirmed or not — the autopilot may have stored a value before the
    # echo timed out.
    if written_mav_names:
        full_param_cache.invalidate_keys(sys_id, written_mav_names)
    return {"sys_id": sys_id, "results": results}


@router.get("/{sys_id}/params/named")
async def read_named_params(sys_id: int, names: str, timeout_s: float | None = None) -> dict[str, Any]:
    """Read parameters by name, each fresh from the vehicle (never the cache).

    ``names`` is comma separated. The lockstep eval reads its command bounds
    here, since serial-step refuses MAVFTP; wall mode keeps the full MAVFTP
    snapshot. The answer has the full snapshot's ``params`` record shape. A
    name the vehicle does not answer (missing, or no reply in time) is an
    error that lists it. ``timeout_s`` bounds the whole request, as for writes.
    """
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")
    requested = [name.strip().upper() for name in names.split(",") if name.strip()]
    if not requested or len(set(requested)) != len(requested):
        raise HTTPException(status_code=400, detail="names must be distinct and non-empty")
    policy = _policy(timeout_s)
    loop = asyncio.get_running_loop()
    records, unanswered = [], []
    for name in requested:
        exchange = await loop.run_in_executor(None, entry.vehicle.read_parameter, name, policy)
        if exchange.confirmed:
            records.append({"name": name, "value": exchange.value})
        else:
            unanswered.append(name)
    if unanswered:
        raise HTTPException(status_code=502, detail={"unanswered": unanswered})
    return {"sys_id": sys_id, "params": records}
