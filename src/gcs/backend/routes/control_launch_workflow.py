"""Launch and restart orchestration with explicit readiness and action dependencies."""
from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Protocol
from fastapi import HTTPException
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM
from navpy.core.ports.flight_mode import FlightMode
from gcs.backend.vehicle_manager import VehicleManager
from gcs.backend.broadcast import ConnectionManager
from gcs.backend.settings_store import SettingsStore
from gcs.backend.models import LaunchRequest

log = logging.getLogger("gcs.backend.routes.control")

from gcs.backend.routes.control_readiness import (
    _readiness_gates_from_settings, ekf_recheck_for_session,
)

async def _broadcast_new_mission(ws_manager: ConnectionManager, sys_ids: list[int]) -> None:
    """A new mission ends the old one's confirmation cards and map history on every client."""
    await ws_manager.broadcast({
        "type": "task_confirm_reset",
        "sys_ids": list(sys_ids),
        "reason": "new_mission",
    })


async def launch_sequence(
    req: LaunchRequest,
    *,
    vehicle_mgr: VehicleManager,
    settings_store: SettingsStore,
    _maybe_auto_preflight_cal: Callable[..., Awaitable[list[int]]],
    _check_launch_readiness: Callable[..., list[str]],
    _container_launch: Callable[..., Awaitable[dict]],
    _bungee_launch: Callable[..., Awaitable[dict]],
    ws_manager: ConnectionManager,
) -> dict:
    """Launch vehicles. Container mode uses prepare(); bungee mode does AUTO+ARM."""
    log.info("[launch] /launch called: sys_ids=%s, force=%s", req.sys_ids, req.force)
    settings = settings_store.get()
    if settings.launch.launch_type == "container":
        # Container membership is a backend-owned safety decision. A stale UI
        # must not be able to omit a connected aircraft by submitting a roster
        # derived from plan zones or configuration counts.
        connected_sys_ids = list(vehicle_mgr.vehicles)
        if not connected_sys_ids:
            raise HTTPException(status_code=409, detail="No vehicles connected")
        if req.sys_ids != connected_sys_ids:
            log.warning(
                "[launch] replacing requested container roster %s with connected fleet %s",
                req.sys_ids, connected_sys_ids,
            )
        req = req.model_copy(update={"sys_ids": connected_sys_ids})
    # Optional auto preflight calibration on disarmed vehicles before the
    # readiness gate/arming, so a fresh cal can clear prearm before launch.
    # pitot_covered gates baro/airspeed re-zero on pitot-equipped vehicles.
    await _maybe_auto_preflight_cal(req.sys_ids, settings, req.pitot_covered)
    # Readiness gate: block launch unless force override. Operator-tunable
    # gates (thresholds + per-check enable) come from launch settings.
    if not req.force:
        gates = _readiness_gates_from_settings(settings.launch)
        issues = _check_launch_readiness(req.sys_ids, gates)
        if issues:
            log.warning("[launch] /launch BLOCKED by readiness checks: %s", issues)
            raise HTTPException(status_code=409, detail="; ".join(issues))

    # Mission upload gate — accept either a GCS-uploaded mission or
    # a mission already on the vehicle (mission_items_count > 0).
    missing = []
    search_patterns = set()
    for sys_id in req.sys_ids:
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            missing.append(sys_id)
            continue
        has_mission = entry.mission_uploaded or (entry.vehicle.mission_items_count or 0) > 0
        # Re-probe if vehicle has items on board but wasn't verified at connect.
        # probe_existing_mission does a synchronous MAVLink download, so run
        # in executor to avoid blocking the event loop.
        if not entry.mission_uploaded and (entry.vehicle.mission_items_count or 0) > 0:
            log.info("[launch] V%d has %d mission items but not verified, re-probing",
                     sys_id, entry.vehicle.mission_items_count)
            loop = asyncio.get_running_loop()
            probe = await loop.run_in_executor(
                None, vehicle_mgr.probe_existing_mission, sys_id,
            )
            has_mission = probe.valid
        if not has_mission:
            missing.append(sys_id)
        elif entry.probed_search_pattern:
            search_patterns.add(entry.probed_search_pattern)
    if missing and not req.force:
        log.warning("[launch] /launch BLOCKED — missing mission: %s", missing)
        raise HTTPException(
            status_code=409,
            detail=f"Vehicles missing mission: {missing}",
        )
    if len(search_patterns) > 1:
        log.warning("[launch] /launch BLOCKED — search pattern mismatch: %s", search_patterns)
        raise HTTPException(
            status_code=409,
            detail=f"Search pattern mismatch across vehicles: {search_patterns}",
        )

    if settings.launch.launch_type == "container":
        # A container launch is always the whole fleet, so a new mission. A bungee
        # launch here is one UAV's launch or relaunch; its fleet start goes through restart.
        result = await _container_launch(req, settings)
        await _broadcast_new_mission(ws_manager, req.sys_ids)
        return result
    return await _bungee_launch(req, settings)

#: AUTO requests per vehicle before the restart reports failure.
AUTO_CONFIRM_TRIES = 3
#: Wait per request for a newer heartbeat showing AUTO. ArduPlane sends one
#: per second, so this covers at least two.
AUTO_CONFIRM_WAIT_S = 2.5
AUTO_CONFIRM_POLL_S = 0.1


class _ModeConfirmVehicle(Protocol):
    heartbeat_count: int
    get_mode: FlightMode | None

    def set_mode(self, flight_mode: FlightMode) -> bool: ...


async def _set_auto_confirmed(vehicle: _ModeConfirmVehicle) -> tuple[bool, FlightMode | None]:
    """Request AUTO until a heartbeat newer than the request shows it.

    Returns whether it was confirmed and the last fresh mode seen (None when
    no newer heartbeat came). A request sent but not taken (seen as AUTOTUNE,
    2026-11-05) is then reported, not counted as a restart.
    """
    seen: FlightMode | None = None
    for _ in range(AUTO_CONFIRM_TRIES):
        before = vehicle.heartbeat_count
        vehicle.set_mode(FlightMode.AUTO)
        for _ in range(max(1, round(AUTO_CONFIRM_WAIT_S / AUTO_CONFIRM_POLL_S))):
            await asyncio.sleep(AUTO_CONFIRM_POLL_S)
            if vehicle.heartbeat_count <= before:
                continue
            seen = vehicle.get_mode
            if seen == FlightMode.AUTO:
                return True, seen
    return False, seen


async def restart_mission(
    req: LaunchRequest,
    *,
    vehicle_mgr: VehicleManager,
    settings_store: SettingsStore,
    _maybe_auto_preflight_cal: Callable[..., Awaitable[list[int]]],
    _check_launch_readiness: Callable[..., list[str]],
    ws_manager: ConnectionManager,
) -> dict:
    """Restart mission: reset to WP 0 and set AUTO mode for each vehicle."""
    # Bungee START MISSION routes here; run optional auto preflight cal on
    # disarmed vehicles before the readiness gate/arming.
    await _maybe_auto_preflight_cal(req.sys_ids, settings_store.get(), req.pitot_covered)
    if not req.force:
        issues = _check_launch_readiness(req.sys_ids)
        if issues:
            raise HTTPException(status_code=409, detail="; ".join(issues))

    # /restart's fixed default gates, carried to the re-check before each arm
    # and before AUTO (bungee START MISSION releases through AUTO).
    ekf_recheck = ekf_recheck_for_session(None, force=req.force, vehicle_mgr=vehicle_mgr)
    results = {}

    def ekf_blocks(sys_id: int, step: str) -> bool:
        issues = ekf_recheck(sys_id)
        if not issues:
            return False
        log.error("Restart %s BLOCKED for vehicle %d by EKF re-check: %s",
                  step, sys_id, issues)
        results[sys_id] = f"blocked: {'; '.join(issues)}"
        return True

    for sys_id in req.sys_ids:
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[sys_id] = "not_connected"
            continue
        if not entry.mission_uploaded and not req.force:
            results[sys_id] = "no_verified_upload"
            continue
        try:
            entry.vehicle.restart_mission(0)
            await asyncio.sleep(0.3)

            # Re-arm if disarmed (e.g. after one-shot termination), unless
            # the EKF went bad during the waits (earlier vehicles, the settle).
            if not entry.vehicle.is_armed:
                if ekf_blocks(sys_id, "arm"):
                    continue
                entry.vehicle.send_command_long(
                    MAV_CMD_COMPONENT_ARM_DISARM, p1=1,
                    **({"p2": 2989} if req.force else {}),
                )
                for _ in range(50):
                    if entry.vehicle.is_armed:
                        break
                    await asyncio.sleep(0.1)

            # Checked again right before AUTO, after the settle or arm wait.
            if ekf_blocks(sys_id, "AUTO"):
                continue
            confirmed, seen = await _set_auto_confirmed(entry.vehicle)
            if not confirmed:
                detail = f"mode {seen.name}" if seen is not None else "no new heartbeat"
                results[sys_id] = f"error: AUTO not confirmed ({detail})"
                log.error("Restart for vehicle %d: AUTO not confirmed (%s)", sys_id, detail)
                continue
            results[sys_id] = "restarted"
            log.info("Vehicle %d mission restarted", sys_id)
        except Exception as e:
            results[sys_id] = f"error: {e}"
            log.error("Restart failed for vehicle %d: %s", sys_id, e)

    # Mission restart ends any in-flight navigation task; tell all clients
    # (cross-client convergence, not just the initiating client). It starts a
    # new mission only if a UAV did restart: otherwise the map keeps its history.
    if "restarted" in results.values():
        await _broadcast_new_mission(ws_manager, req.sys_ids)
    else:
        await ws_manager.broadcast({"type": "task_confirm_reset", "sys_ids": list(req.sys_ids)})

    return {"status": "restart_complete", "results": results}
