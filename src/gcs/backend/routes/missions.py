"""Mission download/upload endpoints."""
from __future__ import annotations

import asyncio
import logging

from fastapi import APIRouter, HTTPException, Request

from gcs.backend.planner.waypoint_builder import upload_mission_with_retry
from gcs.backend.planner.fence_builder import (
    upload_fence_with_retry,
)
from gcs.backend.mission_validator import parse_mission_items
from gcs.backend.models import UploadRequest, UploadProgress, FencePlan, FenceOutcome
from gcs.backend.settings_store import settings_store
from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.full_params import full_param_cache
from gcs.backend.diagnostics import emit
from gcs.backend.routes.mission_fence_download import download_fence_snapshot
from gcs.backend.routes.mission_upload_helpers import (
    assignment_geometry, apply_fence, fence_outcome_model,
)

log = logging.getLogger(__name__)
router = APIRouter()

async def _apply_fence(loop, entry, sys_id: int, fence: FencePlan | None) -> dict | None:
    return await apply_fence(
        loop, entry, sys_id, fence, upload_fence=upload_fence_with_retry,
        cache=full_param_cache,
    )


def _fence_outcome_model(status: dict | None) -> FenceOutcome | None:
    return fence_outcome_model(status)


@router.get("/{sys_id}/mission")
async def download_mission(sys_id: int, request: Request):
    """Download the current mission from a connected vehicle."""
    entry = vehicle_mgr.get_vehicle(sys_id)
    request_id = request.headers.get("x-gcs-request-id")
    emit("mission_request_started", source="backend", sys_id=sys_id, request_id=request_id)
    if not entry:
        emit("mission_request_finished", source="backend", sys_id=sys_id, request_id=request_id,
             outcome="not_connected")
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")

    # Return cached probe result when available (avoids redundant MAVLink download)
    if entry.cached_mission:
        emit("mission_request_finished", source="backend", sys_id=sys_id, request_id=request_id,
             outcome="success", cached=True,
             total=len(entry.cached_mission.get("waypoints") or []))
        return entry.cached_mission

    # Download AND parse under the vehicle's mission lock (in the executor) so a
    # concurrent connect-time probe / download can't clear the shared loader
    # mid-parse. parse_mission_items reads the live loader.
    # Track the exact list we last published so our clear only nulls OUR
    # progress, never a concurrent probe's live count (see vehicle_manager).
    _last = [None]
    _milestone = [-1]

    def _progress(current: int, total: int) -> None:
        p = [current, total]
        _last[0] = p
        entry.mission_download_progress = p
        bucket = 4 if total and current >= total else (current * 4 // total if total else 0)
        if bucket > _milestone[0]:
            _milestone[0] = bucket
            emit("mission_request_progress", source="backend", sys_id=sys_id,
                 request_id=request_id, current=current, total=total)

    def _download_and_parse():
        try:
            with entry.vehicle.mission_lock:
                # Re-check the cache under the lock. The connect-time async probe
                # is kicked before this GET even arrives; if it was still
                # downloading when we did the pre-lock check above, we blocked
                # here until it finished and cached. Reuse that result instead of
                # running a SECOND full download for the same vehicle — which the
                # operator sees as the progress bar jumping backwards (16/17 →
                # 1/17 → …) as the redundant download restarts the counter.
                if entry.cached_mission:
                    emit("mission_backend_finished", source="backend", sys_id=sys_id,
                         request_id=request_id, outcome="success", cached=True,
                         total=len(entry.cached_mission.get("waypoints") or []))
                    return entry.cached_mission
                count = entry.vehicle.download_mission(on_progress=_progress)
                if count == 0:
                    emit("mission_backend_finished", source="backend", sys_id=sys_id,
                         request_id=request_id, outcome="failure", reason="empty_or_download_failed")
                    return None
                parsed = parse_mission_items(entry.vehicle, count, sys_id)
                emit("mission_backend_finished", source="backend", sys_id=sys_id,
                     request_id=request_id, outcome="success",
                     total=len(parsed.get("waypoints") or []))
                return parsed
        except Exception as exc:
            emit("mission_backend_finished", source="backend", sys_id=sys_id,
                 request_id=request_id, outcome="failure",
                 error_code=exc.__class__.__name__)
            raise
        finally:
            if entry.mission_download_progress is _last[0]:
                entry.mission_download_progress = None

    loop = asyncio.get_event_loop()
    parsed = await loop.run_in_executor(None, _download_and_parse)
    if parsed is None:
        emit("mission_request_finished", source="backend", sys_id=sys_id, request_id=request_id,
             outcome="failure", reason="empty_or_download_failed")
        raise HTTPException(status_code=404, detail="No mission on vehicle or download failed")

    emit("mission_request_finished", source="backend", sys_id=sys_id, request_id=request_id,
         outcome="success", total=len(parsed.get("waypoints") or []))
    return parsed


@router.get("/{sys_id}/fence")
async def download_fence(sys_id: int):
    """Download the polygon geofence (inclusion ring + exclusion keep-outs) from
    a connected vehicle.

    Mirrors the fence upload path: the vehicle download returns the raw FENCE
    table items, which :func:`parse_fence_items` groups back into the inclusion
    ring + exclusion rings. An empty fence returns 200 with empty lists; any
    MAVLink/download error returns 502 (never a raw 500 trace).

    ``params`` carries this vehicle's four raw fence parameters, read fresh, so
    the caller can tell an enabled fence from an auto-enable-on-takeoff one
    from a genuinely off one. ``params_readback`` is ``failed`` when they could
    not be read: unknown, never an invented 0. An empty fence table with a
    successful readback is a KNOWN-empty fence, which is not the same thing.
    """
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        raise HTTPException(status_code=404, detail=f"Vehicle {sys_id} not connected")

    return await download_fence_snapshot(entry.vehicle, sys_id)


@router.post("/upload")
async def upload_missions(req: UploadRequest):
    """Upload mission waypoints to assigned vehicles."""
    results = []

    # Safe climb-out height that ends NAV_TAKEOFF, shared by all assigned UAVs.
    takeoff_altitude_m = settings_store.get().flight.takeoff_altitude_m

    for assignment in req.assignments:
        entry = vehicle_mgr.get_vehicle(assignment.sys_id)
        if not entry:
            results.append(UploadProgress(
                sys_id=assignment.sys_id,
                progress=0.0,
                done=True,
                error=f"Vehicle {assignment.sys_id} not connected",
            ))
            continue

        # Broadcast upload start
        await ws_manager.broadcast({
            "type": "upload_progress",
            "sys_id": assignment.sys_id,
            "progress": 0.1,
            "done": False,
        })

        fence_status = None
        try:
            track, poly, corr, lp, dt = assignment_geometry(assignment)

            loop = asyncio.get_event_loop()

            async def _broadcast_stage(stage: str, attempt: int):
                await ws_manager.broadcast({
                    "type": "upload_progress",
                    "sys_id": assignment.sys_id,
                    "stage": stage,
                    "attempt": attempt,
                })

            async def _broadcast_wp(wp_sent: int, wp_total: int):
                await ws_manager.broadcast({
                    "type": "upload_progress",
                    "sys_id": assignment.sys_id,
                    "stage": "uploading",
                    "wp_sent": wp_sent,
                    "wp_total": wp_total,
                    "progress": wp_sent / wp_total if wp_total else 0,
                })

            def on_progress(stage: str, attempt: int):
                asyncio.run_coroutine_threadsafe(_broadcast_stage(stage, attempt), loop)

            def on_wp_progress(wp_sent: int, wp_total: int):
                asyncio.run_coroutine_threadsafe(_broadcast_wp(wp_sent, wp_total), loop)

            result = await loop.run_in_executor(None, lambda: upload_mission_with_retry(
                entry.vehicle, track, assignment.altitude_m,
                corridor_count=assignment.corridor_count,
                search_pattern=assignment.search_pattern,
                polygon=poly,
                corridor_backbone=corr,
                launch_point=lp,
                dock_classes=assignment.dock_classes or None,
                corridor_altitude_m=assignment.corridor_altitude_m,
                fallback_delivery_location=dt,
                takeoff_altitude_m=takeoff_altitude_m,
                on_progress=on_progress,
                on_wp_progress=on_wp_progress,
            ))

            # If link lost, try reconnecting and retrying once
            if result.link_lost:
                await ws_manager.broadcast({
                    "type": "upload_progress",
                    "sys_id": assignment.sys_id,
                    "stage": "reconnecting",
                    "attempt": result.attempts,
                })
                new_entry = await loop.run_in_executor(
                    None, lambda: vehicle_mgr.reconnect_vehicle(assignment.sys_id),
                )
                if new_entry:
                    entry = new_entry
                    result = await loop.run_in_executor(None, lambda: upload_mission_with_retry(
                        entry.vehicle, track, assignment.altitude_m,
                        corridor_count=assignment.corridor_count,
                        search_pattern=assignment.search_pattern,
                        polygon=poly,
                        corridor_backbone=corr,
                        launch_point=lp,
                        dock_classes=assignment.dock_classes or None,
                        corridor_altitude_m=assignment.corridor_altitude_m,
                        fallback_delivery_location=dt,
                        takeoff_altitude_m=takeoff_altitude_m,
                        max_retries=2,
                        on_progress=on_progress,
                        on_wp_progress=on_wp_progress,
                    ))

            if result.success:
                entry.mission_uploaded = True
                entry.probed_search_pattern = assignment.search_pattern
                entry.cached_mission = None  # invalidate probe cache
                entry.vehicle.set_parameter("RTL_ALTITUDE", float(assignment.altitude_m))
                full_param_cache.invalidate_keys(assignment.sys_id, ["RTL_ALTITUDE"])
                # Apply the shared geofence (if any) after the mission upload.
                fence_status = await _apply_fence(loop, entry, assignment.sys_id, req.fence)

            progress = UploadProgress(
                sys_id=assignment.sys_id,
                progress=1.0 if result.success else 0.0,
                done=True,
                error=result.error,
                fence=_fence_outcome_model(fence_status),
            )
        except Exception as e:
            log.error("Upload to vehicle %d failed: %s", assignment.sys_id, e)
            progress = UploadProgress(
                sys_id=assignment.sys_id,
                progress=0.0,
                done=True,
                error=str(e),
            )

        results.append(progress)

        # Broadcast upload completion
        await ws_manager.broadcast({
            "type": "upload_progress",
            "sys_id": assignment.sys_id,
            "stage": "complete" if progress.error is None else "failed",
            "progress": progress.progress,
            "done": progress.done,
            "error": progress.error,
            **({"fence": fence_status} if fence_status is not None else {}),
        })

    all_ok = all(r.error is None for r in results)
    # Fence truth is reported separately from mission truth: a vehicle with no
    # fence attempt (none requested, or its mission failed) cannot make this
    # false, and a failed fence cannot make the mission look failed.
    fence_ok = all(r.fence is None or r.fence.applied for r in results)
    return {
        "status": "complete" if all_ok else "partial_failure",
        "results": results,
        "fence_ok": fence_ok,
    }
