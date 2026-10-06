"""Routes for starting/stopping NavPy simulation instances."""
from __future__ import annotations

import asyncio
import logging
from collections import Counter
from typing import Annotated, Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from gcs.backend import navpy_sim_runtime as runtime
from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.settings_store import settings_store
from gcs.backend.diagnostics import emit

log = logging.getLogger(__name__)

router = APIRouter()
_last_diagnostic_status: dict[int, tuple[bool, int | None]] = {}
MavSystemId = Annotated[int, Field(strict=True, ge=1, le=255)]
MissionIndex = Annotated[int, Field(strict=True, ge=0, le=65535)]


class NavpyStartRequest(BaseModel):
    sys_id: MavSystemId
    # Optional/ignored: the connection is derived authoritatively from the chat
    # (see start route) so a manual Run matches auto-start. Kept for compatibility.
    connection: str | None = None


class NavpyStartAllRequest(BaseModel):
    instances: list[NavpyStartRequest]


class NavpyExpectedParams(BaseModel):
    targ_wps: MissionIndex
    nav_last_wp: MissionIndex


class NavpyRestartReadyInstance(BaseModel):
    sys_id: MavSystemId
    expected_params: NavpyExpectedParams


class NavpyRestartReadyRequest(BaseModel):
    instances: list[NavpyRestartReadyInstance]


def _ensure_sim_mode(settings=None) -> None:
    if settings is None:
        settings = settings_store.get()
    if not settings.simulation.sim_mode:
        raise HTTPException(400, "NavPy sim only available in sim mode")


def _get_or_create_mgr():
    return runtime.get_or_create_mgr()


async def broadcast_sim_status() -> None:
    """Push current NavPy sim status to all WebSocket clients."""
    if not runtime.has_manager():
        return
    statuses = runtime.get_all_status()
    current_ids = {int(status["sys_id"]) for status in statuses}
    for sys_id in sorted(set(_last_diagnostic_status) - current_ids):
        _last_diagnostic_status.pop(sys_id, None)
        emit("navpy_process_state", source="backend", sys_id=sys_id,
             running=False, outcome="stopped")
    for status in statuses:
        sys_id = int(status["sys_id"])
        state = (bool(status.get("running")), status.get("exit_code"))
        if _last_diagnostic_status.get(sys_id) != state:
            _last_diagnostic_status[sys_id] = state
            emit("navpy_process_state", source="backend", sys_id=sys_id,
                 running=state[0], exit_code=state[1],
                 outcome="running" if state[0] else "stopped")
    await ws_manager.broadcast({
        "type": "navpy_sim_status",
        "instances": statuses,
    })


@router.post("/navpy-sim/start")
async def start_navpy_instance(req: NavpyStartRequest) -> dict[str, Any]:
    """Start a single NavPy simulation instance."""
    settings = settings_store.get()
    _ensure_sim_mode(settings)
    # Derive the connection from the chat/settings (same source auto-start uses),
    # so a manual Run always targets the right SITL instance regardless of what
    # the client sent.
    connection = runtime.sim_connection_from_settings(req.sys_id, settings)
    try:
        inst = await asyncio.to_thread(
            runtime.managed_start_instance,
            req.sys_id,
            connection,
            settings,
        )
    except ValueError as exc:
        raise HTTPException(409, str(exc))
    await broadcast_sim_status()
    emit("navpy_operation_finished", source="backend", sys_id=req.sys_id,
         outcome="started")
    return {"status": "started", "sys_id": req.sys_id, "pid": inst.process.pid}


@router.post("/navpy-sim/start-all")
async def start_all_navpy(req: NavpyStartAllRequest) -> dict[str, Any]:
    """Start multiple NavPy simulation instances."""
    settings = settings_store.get()
    _ensure_sim_mode(settings)
    # results is keyed by sys_id, so duplicates would silently overwrite each
    # other (a started process could be reported as an error). Reject them.
    duplicates = sorted(
        sys_id for sys_id, count in Counter(i.sys_id for i in req.instances).items()
        if count > 1)
    if duplicates:
        raise HTTPException(422, f"Duplicate sys_ids in request: {duplicates}")
    results = {}
    for item in req.instances:
        key = str(item.sys_id)
        try:
            connection = runtime.sim_connection_from_settings(item.sys_id, settings)
            inst = await asyncio.to_thread(
                runtime.managed_start_instance,
                item.sys_id,
                connection,
                settings,
            )
            results[key] = {"status": "started", "pid": inst.process.pid}
        except ValueError as exc:
            results[key] = {"status": "error", "detail": str(exc)}
        except Exception as exc:
            # One item's spawn failure must not abort the batch: the remaining
            # instances still start, and the status broadcast + batch
            # diagnostic below still run.
            log.warning("Failed to start NavPy sim for sys_id=%d: %s",
                        item.sys_id, exc)
            results[key] = {"status": "error", "detail": str(exc)}
    await broadcast_sim_status()
    accepted = sum(1 for result in results.values() if result["status"] == "started")
    emit("navpy_batch_finished", source="backend", vehicle_count=len(results),
         accepted_count=accepted, rejected_count=len(results) - accepted,
         outcome="success" if accepted == len(results) else "partial")
    return {"status": "started", "results": results}


@router.post("/navpy-sim/stop/{sys_id}")
async def stop_navpy_instance(sys_id: int) -> dict[str, Any]:
    """Stop a single NavPy simulation instance."""
    _ensure_sim_mode()
    # No has_manager() pre-check: stop_instance() discards the managed-intent
    # entry before checking the manager, so an explicit Stop suppresses a
    # later auto-heal restart even if the manager is gone.
    if not await asyncio.to_thread(runtime.stop_instance, sys_id):
        raise HTTPException(404, f"No running instance for sys_id {sys_id}")
    await broadcast_sim_status()
    emit("navpy_operation_finished", source="backend", sys_id=sys_id,
         outcome="stopped")
    return {"status": "stopped", "sys_id": sys_id}


@router.post("/navpy-sim/stop-all")
async def stop_all_navpy() -> dict[str, Any]:
    """Stop all NavPy simulation instances."""
    _ensure_sim_mode()
    await asyncio.to_thread(runtime.stop_all_instances)
    await broadcast_sim_status()
    return {"status": "stopped_all"}


@router.post("/navpy-sim/restart-ready")
async def restart_navpy_ready(req: NavpyRestartReadyRequest) -> dict[str, Any]:
    """Restart the exact configured companions and await hydrated runtimes."""
    settings = settings_store.get()
    _ensure_sim_mode(settings)
    if not 1 <= len(req.instances) <= 3:
        raise HTTPException(422, "restart-ready requires one to three instances")
    duplicates = sorted(
        sys_id
        for sys_id, count in Counter(item.sys_id for item in req.instances).items()
        if count > 1
    )
    if duplicates:
        raise HTTPException(422, f"Duplicate sys_ids in request: {duplicates}")
    expected = {
        item.sys_id: {
            "targ_wps": item.expected_params.targ_wps,
            "nav_last_wp": item.expected_params.nav_last_wp,
        }
        for item in req.instances
    }
    try:
        instances = await asyncio.to_thread(
            runtime.restart_configured_navpy_sim,
            expected,
            settings,
        )
    except ValueError as exc:
        await broadcast_sim_status()
        raise HTTPException(409, str(exc)) from exc
    except TimeoutError as exc:
        await broadcast_sim_status()
        raise HTTPException(504, str(exc)) from exc
    except RuntimeError as exc:
        await broadcast_sim_status()
        raise HTTPException(503, str(exc)) from exc
    await broadcast_sim_status()
    emit(
        "navpy_batch_finished",
        source="backend",
        vehicle_count=len(instances),
        accepted_count=len(instances),
        rejected_count=0,
        outcome="ready",
    )
    return {"status": "ready", "instances": instances}


@router.get("/navpy-sim/status")
async def navpy_sim_status() -> dict[str, Any]:
    """Return status of all NavPy simulation instances."""
    return {"instances": runtime.get_all_status()}


def _get_sim_connection(sys_id: int) -> str:
    """Derive the SITL connection string for a given sys_id from settings."""
    return runtime.sim_connection(sys_id)


def _get_sim_connection_from_settings(sys_id: int, settings) -> str:
    """Derive the SITL connection string for a given sys_id from a settings snapshot."""
    return runtime.sim_connection_from_settings(sys_id, settings)


def auto_start_navpy_sim(sys_id: int) -> bool:
    """Start a NavPy sim instance for *sys_id* if sim_mode is enabled.

    Gated on the vehicle still being connected: connect/discover (and startup
    auto-connect) call this after an executor await, so a disconnect issued in
    that window must not let the stale completion re-start the companion and
    re-adopt managed intent (issue #163 race). The gate lives here — the route
    layer owns lifecycle intent — so bare runtime.auto_start_navpy_sim stays
    policy-neutral.
    """
    from gcs.backend.vehicle_manager import vehicle_mgr
    if vehicle_mgr.get_vehicle(sys_id) is None:
        return False
    return runtime.auto_start_navpy_sim(sys_id)


def restart_running_navpy_sim(settings=None) -> list[int]:
    """Restart running NavPy sim instances using a settings snapshot."""
    return runtime.restart_running_navpy_sim(settings)


def stop_all_navpy_if_running() -> None:
    """Shutdown helper called from lifespan."""
    runtime.stop_all_navpy_if_running()
