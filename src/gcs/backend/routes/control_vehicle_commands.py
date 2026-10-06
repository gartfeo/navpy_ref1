"""Basic flight-mode, arming and reboot command dispatch."""
from __future__ import annotations

import logging
from fastapi import HTTPException
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
from navpy.modules.vehicle.flight_mode import FlightMode
from gcs.backend.vehicle_manager import VehicleManager

log = logging.getLogger("gcs.backend.routes.control")

def _handle_estop(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Disarm all targeted vehicles (emergency stop)."""
    results = {}
    for sys_id in targets:
        entry = vehicle_mgr.get_vehicle(sys_id)
        key = str(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        try:
            entry.vehicle.set_mode(FlightMode.MANUAL)
            entry.vehicle.send_command_long(
                MAV_CMD_COMPONENT_ARM_DISARM,
                p1=0,       # 0 = disarm
                p2=21196,   # force disarm magic number
            )
            results[key] = "disarmed"
            log.warning("E-STOP: vehicle %d disarmed", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("E-STOP failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_set_mode(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Set flight mode on targeted vehicles."""
    mode_name = params.get("mode") if params else None
    if not mode_name:
        raise HTTPException(400, "Missing 'mode' in params")
    try:
        mode = FlightMode[mode_name.upper()]
    except KeyError:
        raise HTTPException(400, f"Unknown mode: {mode_name}")

    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        entry.vehicle.set_mode(mode)
        results[key] = f"mode_set_{mode.name}"
    return results

def _handle_arm(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Arm targeted vehicles (normal arm, no force flag)."""
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        try:
            entry.vehicle.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
            results[key] = "armed"
            log.info("ARM: vehicle %d armed", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("ARM failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_force_arm(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Force-arm targeted vehicles, bypassing prearm checks (p2=2989)."""
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        try:
            entry.vehicle.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=2989)
            results[key] = "force_armed"
            log.info("FORCE ARM: vehicle %d force-armed", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("FORCE ARM failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_force_disarm(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Force-disarm targeted vehicles, bypassing checks (p2=21196)."""
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        try:
            entry.vehicle.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, p1=0, p2=21196)
            results[key] = "force_disarmed"
            log.info("FORCE DISARM: vehicle %d force-disarmed", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("FORCE DISARM failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_disarm(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Disarm targeted vehicles (normal disarm, no force flag)."""
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        try:
            entry.vehicle.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, p1=0)
            results[key] = "disarmed"
            log.info("DISARM: vehicle %d disarmed", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("DISARM failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_reboot(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Reboot the autopilot (flight controller) on targeted vehicles.

    Sends MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with p1=1 (reboot autopilot).
    Refuses any armed vehicle as a safety gate — rebooting in flight would
    drop the vehicle. p1=3 (reboot to bootloader) is intentionally not
    exposed here; this is a normal autopilot reboot only.
    """
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        if entry.vehicle.is_armed:
            results[key] = "refused_armed"
            log.warning("REBOOT refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.vehicle.send_command_long(
                MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                p1=1,  # 1 = reboot autopilot
            )
            results[key] = "reboot_sent"
            log.warning("REBOOT: vehicle %d autopilot reboot command sent", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("REBOOT failed for vehicle %d: %s", sys_id, e)
    return results
