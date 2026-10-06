"""Full per-vehicle parameter list endpoints (Mission Planner-style)."""
from __future__ import annotations

import logging
from typing import Optional

import asyncio
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.full_params import (
    ArmedWriteRejected,
    FullParamError,
    ParamChange,
    full_param_cache,
)
from gcs.backend.param_readonly import is_readonly
from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.diagnostics import emit

log = logging.getLogger(__name__)
router = APIRouter()


class _ParamChangeBody(BaseModel):
    name: str = Field(..., min_length=1, max_length=16)
    value: float | int


class _WriteBatchBody(BaseModel):
    changes: list[_ParamChangeBody]
    armed_token: Optional[str] = None


def _record_to_dict(rec) -> dict:
    return {
        "name": rec.name,
        "value": rec.value,
        "ap_type": rec.ap_type,
        "default": rec.default,
        "default_known": rec.default_known,
        "flags": rec.flags,
        "read_only": is_readonly(rec.name),
    }


def _snapshot_to_dict(snapshot, sys_id: int, *, stale: bool = False) -> dict:
    return {
        "sys_id": sys_id,
        "fetched_at_unix_s": snapshot.fetched_at_unix_s,
        "with_defaults": snapshot.with_defaults,
        "num_params": snapshot.num_params,
        "total_params": snapshot.total_params,
        "params": [_record_to_dict(r) for r in snapshot.pck.records],
        "stale": stale,
    }


def _change_names(body: _WriteBatchBody) -> list[str]:
    return [c.name.upper() for c in body.changes]


@router.get("/{sys_id}/parameters")
async def get_full_parameters(
    request: Request,
    sys_id: int,
    refresh: bool = False,
):
    """Return the full parameter snapshot for a vehicle.

    Always fetches with `?withdefaults=1` to keep the cache representation
    consistent across callers (per Codex post-step finding 4 for Step 3).
    """
    entry = vehicle_mgr.get_vehicle(sys_id)
    request_id = request.headers.get("x-gcs-request-id")
    cache_hit = not refresh and full_param_cache.peek_fresh(sys_id) is not None
    emit("parameter_request_started", source="backend", sys_id=sys_id,
         request_id=request_id, cached=cache_hit,
         phase="refresh" if refresh else "cache_allowed")
    if not entry:
        emit("parameter_request_finished", source="backend", sys_id=sys_id,
             request_id=request_id, outcome="not_connected")
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")
    loop = asyncio.get_running_loop()
    last_bucket = [-1]

    def _broadcast_progress(event: dict | None) -> None:
        if not isinstance(event, dict):
            return
        payload = {
            "type": "full_param_progress",
            "sys_id": sys_id,
            "bytes_read": event.get("bytes_read", 0),
            "size_estimate": event.get("size_estimate"),
            "total_bytes": event.get("total_bytes"),
            "done": bool(event.get("done", False)),
        }
        read = max(0, int(event.get("bytes_read", 0) or 0))
        estimate = max(0, int(event.get("total_bytes") or event.get("size_estimate") or 0))
        bucket = 4 if event.get("done") else (read * 4 // estimate if estimate else 0)
        if bucket > last_bucket[0]:
            last_bucket[0] = bucket
            emit("parameter_request_progress", source="backend", sys_id=sys_id,
                 request_id=request_id, bytes_read=read, size_estimate=estimate,
                 phase="done" if event.get("done") else "downloading")
        fut = asyncio.run_coroutine_threadsafe(
            ws_manager.broadcast(payload),
            loop,
        )

        def _log_error(done_fut) -> None:
            try:
                done_fut.result()
            except Exception:
                log.debug(
                    "full_params progress broadcast failed for sys_id=%d",
                    sys_id,
                    exc_info=True,
                )

        fut.add_done_callback(_log_error)

    try:
        snapshot = await full_param_cache.get_or_fetch(
            sys_id,
            entry.vehicle,
            entry.device,
            refresh=refresh,
            progress_callback=_broadcast_progress,
        )
    except Exception as exc:
        log.exception("full_params GET failed for sys_id=%d", sys_id)
        emit("parameter_request_finished", source="backend", sys_id=sys_id,
             request_id=request_id, outcome="failure", error_code=exc.__class__.__name__)
        raise HTTPException(status_code=502, detail=str(exc))
    emit("parameter_request_finished", source="backend", sys_id=sys_id,
         request_id=request_id, outcome="success", record_count=snapshot.num_params,
         stale=False)
    return _snapshot_to_dict(snapshot, sys_id)


@router.put("/{sys_id}/parameters")
async def put_full_parameters(sys_id: int, body: _WriteBatchBody, request: Request):
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")
    changes = [ParamChange(name=c.name.upper(), value=c.value) for c in body.changes]
    request_id = request.headers.get("x-gcs-request-id")
    emit("parameter_write_started", source="backend", sys_id=sys_id,
         request_id=request_id, total=len(changes))

    async def _broadcast_write_progress(event: dict) -> None:
        # write_batch awaits this from the event loop, so broadcast directly.
        await ws_manager.broadcast({
            "type": "full_param_write_progress",
            "sys_id": sys_id,
            "written": int(event.get("written", 0)),
            "total": int(event.get("total", 0)),
            "done": bool(event.get("done", False)),
        })

    try:
        result = await full_param_cache.write_batch(
            sys_id, entry.vehicle, changes,
            armed_token=body.armed_token,
            progress_callback=_broadcast_write_progress,
        )
    except ArmedWriteRejected as exc:
        log.warning(
            "full_params PUT rejected for sys_id=%d status=409 names=%s: %s",
            sys_id, _change_names(body), exc,
        )
        emit("parameter_write_finished", source="backend", sys_id=sys_id,
             request_id=request_id, outcome="rejected", rejected_count=len(changes), reason="armed_token_required")
        raise HTTPException(status_code=409, detail=str(exc))
    except FullParamError as exc:
        log.warning(
            "full_params PUT bad request for sys_id=%d status=400 names=%s: %s",
            sys_id, _change_names(body), exc,
        )
        emit("parameter_write_finished", source="backend", sys_id=sys_id,
             request_id=request_id, outcome="rejected", rejected_count=len(changes), reason="invalid_request")
        raise HTTPException(status_code=400, detail=str(exc))
    except Exception as exc:
        log.exception("full_params PUT failed for sys_id=%d", sys_id)
        emit("parameter_write_finished", source="backend", sys_id=sys_id,
             request_id=request_id, outcome="failure", rejected_count=len(changes), error_code=exc.__class__.__name__)
        raise HTTPException(status_code=502, detail=str(exc))
    accepted = sum(1 for item in result.results.values() if item.ok)
    emit("parameter_write_finished", source="backend", sys_id=sys_id,
         request_id=request_id, outcome="success" if accepted == len(changes) else "partial",
         accepted_count=accepted, rejected_count=len(changes) - accepted,
         stale=result.snapshot_stale)
    return {
        "sys_id": sys_id,
        "snapshot_stale": result.snapshot_stale,
        "results": {
            name: {"ok": cr.ok, "value": cr.value, "error": cr.error}
            for name, cr in result.results.items()
        },
    }


@router.post("/{sys_id}/parameters/arm-token")
async def issue_arm_token(sys_id: int):
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")
    token = full_param_cache.issue_armed_token(sys_id)
    return {
        "sys_id": sys_id,
        "nonce": token.nonce,
        "expires_at_unix_s": token.expires_at_unix_s,
    }


@router.delete("/{sys_id}/parameters/cache")
async def drop_cache(sys_id: int):
    full_param_cache.invalidate(sys_id)
    return {"sys_id": sys_id, "ok": True}
