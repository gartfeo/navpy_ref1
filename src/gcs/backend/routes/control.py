"""Stable control API routes; dispatch to focused command and launch services."""
from __future__ import annotations

import asyncio
import logging
from fastapi import APIRouter, HTTPException
from gcs.backend.models import CommandRequest, LaunchRequest, TriggerRequest
from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.launch_controller import launch_controller, UAVS_PER_CONTAINER
from gcs.backend.settings_store import settings_store
from gcs.backend.routes.control_readiness import (
    ReadinessGates, _resolve_channel_map, _readiness_gates_from_settings,
)
from gcs.backend.routes.control_preflight import _send_gyro, _send_baro, _pitot_baro_ok
from gcs.backend.routes.control_rc_calibration import _coerce_pwm, _write_rc_cal
from gcs.backend.routes import (
    control_vehicle_commands,
    control_rc_calibration,
    control_compass_calibration,
    control_accel_calibration,
    control_preflight,
    control_readiness,
    control_launch_workflow,
    control_launch_actions,
    control_trigger,
    control_esp32,
)

log = logging.getLogger(__name__)
router = APIRouter()
_esp32_sim = None




def _handle_estop(targets, params):
    return control_vehicle_commands._handle_estop(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_set_mode(targets, params):
    return control_vehicle_commands._handle_set_mode(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_arm(targets, params):
    return control_vehicle_commands._handle_arm(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_force_arm(targets, params):
    return control_vehicle_commands._handle_force_arm(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_force_disarm(targets, params):
    return control_vehicle_commands._handle_force_disarm(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_disarm(targets, params):
    return control_vehicle_commands._handle_disarm(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_preflight_cal(targets, params):
    return control_preflight._handle_preflight_cal(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_rc_cal_save(targets, params):
    return control_rc_calibration._handle_rc_cal_save(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_compass_cal_start(targets, params):
    return control_compass_calibration._handle_compass_cal_start(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_compass_cal_cancel(targets, params):
    return control_compass_calibration._handle_compass_cal_cancel(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_compass_cal_accept(targets, params):
    return control_compass_calibration._handle_compass_cal_accept(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_compass_cal_reboot(targets, params):
    return control_compass_calibration._handle_compass_cal_reboot(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_accel_level(targets, params):
    return control_accel_calibration._handle_accel_level(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_accel_cal_start(targets, params):
    return control_accel_calibration._handle_accel_cal_start(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_accel_cal_pos(targets, params):
    return control_accel_calibration._handle_accel_cal_pos(targets, params, vehicle_mgr=vehicle_mgr)


def _handle_reboot(targets, params):
    return control_vehicle_commands._handle_reboot(targets, params, vehicle_mgr=vehicle_mgr)


_COMMAND_HANDLERS = {
    "estop": _handle_estop,
    "set_mode": _handle_set_mode,
    "arm": _handle_arm,
    "force_arm": _handle_force_arm,
    "force_disarm": _handle_force_disarm,
    "disarm": _handle_disarm,
    "preflight_cal": _handle_preflight_cal,
    "reboot": _handle_reboot,
    "rc_cal_save": _handle_rc_cal_save,
    "compass_cal_start": _handle_compass_cal_start,
    "compass_cal_cancel": _handle_compass_cal_cancel,
    "compass_cal_accept": _handle_compass_cal_accept,
    "compass_cal_reboot": _handle_compass_cal_reboot,
    "accel_level": _handle_accel_level,
    "accel_cal_start": _handle_accel_cal_start,
    "accel_cal_pos": _handle_accel_cal_pos,
}


async def _maybe_auto_preflight_cal(sys_ids, settings, pitot_covered=False) -> list[int]:
    return await control_preflight._maybe_auto_preflight_cal(sys_ids, settings, pitot_covered, vehicle_mgr=vehicle_mgr)


@router.post("/command")
async def send_command(req: CommandRequest):
    """Send command to vehicles."""
    targets = req.sys_ids or list(vehicle_mgr.vehicles.keys())

    handler = _COMMAND_HANDLERS.get(req.command)
    if handler is None:
        raise HTTPException(400, f"Unknown command: {req.command}")

    results = handler(targets, req.params)

    if req.command == "estop":
        # E-STOP force-disarms and ends all active navigation tasks (whether
        # targeting the whole swarm or a single per-UAV E-STOP), so tell every
        # client to clear those UAVs' confirmation cards.
        await ws_manager.broadcast({"type": "estop", "results": results})
        await ws_manager.broadcast({"type": "task_confirm_reset", "sys_ids": list(targets)})
        return {"status": "estop_sent", "results": results}

    return {"status": f"{req.command}_done", "results": results}


def _check_launch_readiness(
    sys_ids: list[int], gates: ReadinessGates | None = None,
) -> list[str]:
    return control_readiness._check_launch_readiness(sys_ids, gates, vehicle_mgr=vehicle_mgr)


@router.post("/launch")
async def launch_sequence(req: LaunchRequest):
    return await control_launch_workflow.launch_sequence(
        req,
        vehicle_mgr=vehicle_mgr,
        settings_store=settings_store,
        _maybe_auto_preflight_cal=_maybe_auto_preflight_cal,
        _check_launch_readiness=_check_launch_readiness,
        _container_launch=_container_launch,
        _bungee_launch=_bungee_launch,
    )


async def _bungee_launch(req: LaunchRequest, settings):
    return await control_launch_actions._bungee_launch(req, settings, vehicle_mgr=vehicle_mgr, ws_manager=ws_manager)


async def _container_launch(req: LaunchRequest, settings):
    return await control_launch_actions._container_launch(
        req,
        settings,
        vehicle_mgr=vehicle_mgr,
        launch_controller=launch_controller,
        _esp32_sim=_esp32_sim,
    )


@router.post("/launch/trigger")
async def trigger_vehicle(req: TriggerRequest):
    return await control_trigger.trigger_vehicle(req, launch_controller=launch_controller)


@router.post("/launch/abort")
async def abort_launch():
    return await control_trigger.abort_launch(launch_controller=launch_controller)


@router.get("/launch/esp32-status")
async def esp32_status():
    return await control_esp32.esp32_status(_esp32_sim=_esp32_sim, settings_store=settings_store)


@router.post("/launch/esp32-sim/start")
async def start_esp32_sim(port: int | None = None):
    """Start the ESP32 simulator (sim mode only).

    Accepts an optional ``port`` query param so the frontend can pass
    the draft port value before settings are saved.
    """
    global _esp32_sim
    settings = settings_store.get()
    if not settings.simulation.sim_mode:
        raise HTTPException(400, "Simulator only available in sim mode")
    if _esp32_sim is not None:
        raise HTTPException(409, "Simulator already running")

    from navpy.tools.esp32_simulator import Esp32Simulator
    sim_port = port or settings.launch.esp32_port
    _esp32_sim = Esp32Simulator(port=sim_port)
    _esp32_sim.start()
    log.info("ESP32 simulator started on port %d", sim_port)
    return {"status": "started", "port": sim_port}


@router.post("/launch/esp32-sim/stop")
async def stop_esp32_sim():
    """Stop the ESP32 simulator."""
    global _esp32_sim
    if _esp32_sim is None:
        raise HTTPException(409, "Simulator not running")
    _esp32_sim.stop()
    _esp32_sim = None
    log.info("ESP32 simulator stopped")
    return {"status": "stopped"}


@router.get("/launch/esp32-sim/status")
async def esp32_sim_status():
    """Check if the ESP32 simulator is running."""
    return {"running": _esp32_sim is not None}


def stop_esp32_sim_if_running():
    """Shutdown helper called from lifespan."""
    global _esp32_sim
    if _esp32_sim is not None:
        _esp32_sim.stop()
        _esp32_sim = None


def auto_start_esp32_sim() -> bool:
    """Start the ESP32 simulator if sim_mode + container launch type.

    Called automatically after vehicle connect. Silently skips if
    conditions aren't met or simulator is already running. Returns True
    if started.
    """
    global _esp32_sim
    settings = settings_store.get()
    if not settings.simulation.sim_mode:
        return False
    if settings.launch.launch_type != "container":
        return False
    if _esp32_sim is not None:
        return False

    from navpy.tools.esp32_simulator import Esp32Simulator
    sim_port = settings.launch.esp32_port
    _esp32_sim = Esp32Simulator(port=sim_port)
    _esp32_sim.start()
    log.info("Auto-started ESP32 simulator on port %d", sim_port)
    return True


@router.post("/restart")
async def restart_mission(req: LaunchRequest):
    return await control_launch_workflow.restart_mission(
        req,
        vehicle_mgr=vehicle_mgr,
        settings_store=settings_store,
        _maybe_auto_preflight_cal=_maybe_auto_preflight_cal,
        _check_launch_readiness=_check_launch_readiness,
        ws_manager=ws_manager,
    )
