"""Bungee actions and trigger-controller preparation."""
from __future__ import annotations

import asyncio
import logging
from fastapi import HTTPException
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM
from navpy.modules.vehicle.flight_mode import FlightMode
from gcs.backend.vehicle_manager import VehicleManager
from gcs.backend.broadcast import ConnectionManager
from gcs.backend.settings_model import GcsSettings
from gcs.backend.models import LaunchRequest
from gcs.backend.launch_controller import LaunchController, UAVS_PER_CONTAINER

log = logging.getLogger("gcs.backend.routes.control")

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from navpy.tools.esp32_simulator import Esp32Simulator
from gcs.backend.routes.control_readiness import _resolve_channel_map

async def _bungee_launch(
    req: LaunchRequest,
    settings: GcsSettings,
    *,
    vehicle_mgr: VehicleManager,
    ws_manager: ConnectionManager,
) -> dict:
    """Bungee launch: AUTO then ARM per vehicle (no ESP32)."""
    ls = settings.launch
    settle_s = ls.bungee_settle_s
    arm_poll_iters = max(1, round(ls.bungee_arm_timeout_s / 0.1))
    stagger_s = ls.bungee_stagger_s
    results = {}

    launched_any = False
    for sys_id in req.sys_ids:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue

        # Inter-vehicle stagger: space out consecutive launches (skip before
        # the first one). Default 0 = back-to-back, as before.
        if launched_any and stagger_s > 0:
            await asyncio.sleep(stagger_s)

        await ws_manager.broadcast({
            "type": "launch_progress",
            "sys_id": sys_id,
            "state": "arming",
        })

        try:
            # Step 1: Set AUTO mode (fire-and-forget)
            entry.vehicle.set_mode(FlightMode.AUTO)

            # Step 1b: optional AUTO->ARM settle pause (default 0 = none)
            if settle_s > 0:
                await asyncio.sleep(settle_s)

            # Step 2: Arm (vehicle is ready for physical bungee launch)
            if not entry.vehicle.is_armed:
                entry.vehicle.send_command_long(
                    MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1,  # 1 = arm
                    **({"p2": 2989} if req.force else {}),
                )

                # Poll until armed (100ms intervals, bungee_arm_timeout_s)
                for _ in range(arm_poll_iters):
                    if entry.vehicle.is_armed:
                        break
                    await asyncio.sleep(0.1)
                else:
                    log.warning("Vehicle %d arm timeout, proceeding anyway", sys_id)

            await ws_manager.broadcast({
                "type": "launch_progress",
                "sys_id": sys_id,
                "state": "launched",
            })
            results[key] = "launched"
            launched_any = True
            log.info("Vehicle %d bungee launched", sys_id)

        except Exception as e:
            results[key] = f"error: {e}"
            log.error("Launch failed for vehicle %d: %s", sys_id, e)
            await ws_manager.broadcast({
                "type": "launch_progress",
                "sys_id": sys_id,
                "state": "error",
                "error": str(e),
            })

    await ws_manager.broadcast({"type": "launch_complete"})
    return {"status": "launch_complete", "results": results}

async def _container_launch(
    req: LaunchRequest,
    settings: GcsSettings,
    *,
    vehicle_mgr: VehicleManager,
    launch_controller: LaunchController,
    _esp32_sim: Esp32Simulator | None,
) -> dict:
    """Prepare ESP32 container session via LaunchController (no auto-trigger)."""
    if launch_controller.is_running:
        raise HTTPException(409, "Launch already in progress")
    if launch_controller.is_prepared:
        launch_controller.cleanup()

    ls = settings.launch
    channel_map = _resolve_channel_map(ls.default_channel_map, req.sys_ids)
    log.info("[launch] resolved ESP32 channel map: %s", channel_map)

    # When the local simulator is running, target localhost on its port
    if _esp32_sim is not None:
        esp32_host = "127.0.0.1"
        esp32_port = _esp32_sim.port
    else:
        esp32_host = ls.esp32_host
        esp32_port = ls.esp32_port

    def arm_func(sys_id: int) -> None:
        entry = vehicle_mgr.get_vehicle(sys_id)
        if entry:
            entry.vehicle.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, p1=1)

    def auto_func(sys_id: int) -> None:
        entry = vehicle_mgr.get_vehicle(sys_id)
        if entry:
            entry.vehicle.set_mode(FlightMode.AUTO)

    await launch_controller.prepare(
        sys_ids=req.sys_ids,
        channel_map=channel_map,
        esp32_host=esp32_host,
        esp32_port=esp32_port,
        altitude_threshold=ls.altitude_threshold_m,
        arm_timeout=ls.arm_timeout_s,
        altitude_timeout=ls.altitude_timeout_s,
        arm_func=arm_func,
        auto_func=auto_func,
        settle_s=ls.container_settle_s,
        stagger_s=ls.container_stagger_s,
        container_gap_s=ls.container_gap_s,
        uavs_per_container=UAVS_PER_CONTAINER,
        require_armed=ls.airborne_require_armed,
        require_throttle=ls.airborne_require_throttle,
        min_throttle_pct=ls.airborne_min_throttle_pct,
        min_climb_rate_ms=ls.min_climb_rate_ms,
        climb_confirm_s=ls.climb_confirm_s,
    )
    if not launch_controller.is_prepared:
        raise HTTPException(503, "ESP32 not reachable")
    return {"status": "container_prepared", "sys_ids": list(req.sys_ids)}
