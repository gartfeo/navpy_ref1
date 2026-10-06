"""Accelerometer calibration command handlers."""
from __future__ import annotations

import logging
from fastapi import HTTPException
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACCELCAL_VEHICLE_POS
from gcs.backend.vehicle_manager import VehicleManager

log = logging.getLogger("gcs.backend.routes.control")

_ACCEL_CAL_PARAM_LEVEL = 2
_ACCEL_CAL_PARAM_FULL = 1
# MAV_CMD_ACCELCAL_VEHICLE_POS param1 range (1=level .. 6=back), matching
# ArduPilot's AccelCalibrator vehicle-position enum.
_ACCEL_CAL_POS_MIN = 1
_ACCEL_CAL_POS_MAX = 6


def _handle_accel_level(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Quick accelerometer level/trim calibration (PREFLIGHT_CALIBRATION p5=2).

    Sets the board-level trim from the vehicle's current (resting-level)
    attitude. Single-shot: the autopilot replies with a COMMAND_ACK rather than
    interactive prompts. Refuses an armed vehicle (T-1-07) — the 466-behind
    source branch lacked this gate; added here to match _handle_preflight_cal.
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
            log.warning("ACCEL LEVEL refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.mark_accel_cal_active()
            entry.vehicle.send_command_long(
                MAV_CMD_PREFLIGHT_CALIBRATION, p5=_ACCEL_CAL_PARAM_LEVEL,
            )
            results[key] = "level_cal_sent"
            log.info("ACCEL LEVEL: vehicle %d level calibration sent", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("ACCEL LEVEL failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_accel_cal_start(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Start the full 6-position accel calibration (PREFLIGHT_CALIBRATION p5=1).

    The autopilot then emits STATUSTEXT position prompts; the operator advances
    each step with the ``accel_cal_pos`` command. Refuses an armed vehicle
    (T-1-07).
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
            log.warning("ACCEL CAL start refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.mark_accel_cal_active()
            entry.vehicle.send_command_long(
                MAV_CMD_PREFLIGHT_CALIBRATION, p5=_ACCEL_CAL_PARAM_FULL,
            )
            results[key] = "accel_cal_started"
            log.info("ACCEL CAL: vehicle %d full accel calibration started", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("ACCEL CAL start failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_accel_cal_pos(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Advance the full accel cal to a position (ACCELCAL_VEHICLE_POS p1=pos).

    ``params['pos']`` is the 1..6 position code matching the current prompt
    (1=level, 2=left, 3=right, 4=nose down, 5=nose up, 6=back). Refuses an
    armed vehicle (T-1-07) — a mid-flight position command would otherwise be
    sent to the flight controller.
    """
    pos_raw = params.get("pos") if params else None
    if pos_raw is None:
        raise HTTPException(400, "Missing 'pos' in params")
    try:
        pos = int(pos_raw)
    except (TypeError, ValueError):
        raise HTTPException(400, f"Invalid 'pos': {pos_raw!r}")
    if not (_ACCEL_CAL_POS_MIN <= pos <= _ACCEL_CAL_POS_MAX):
        raise HTTPException(
            400,
            f"'pos' out of range [{_ACCEL_CAL_POS_MIN}..{_ACCEL_CAL_POS_MAX}]: {pos}",
        )
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        if entry.vehicle.is_armed:
            results[key] = "refused_armed"
            log.warning("ACCEL CAL pos refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.mark_accel_cal_active()
            entry.vehicle.send_command_long(MAV_CMD_ACCELCAL_VEHICLE_POS, p1=pos)
            results[key] = f"pos_{pos}_sent"
            log.info("ACCEL CAL: vehicle %d position %d confirmed", sys_id, pos)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("ACCEL CAL pos failed for vehicle %d: %s", sys_id, e)
    return results
