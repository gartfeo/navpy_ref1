"""Onboard compass-calibration command handlers."""
from __future__ import annotations

import logging
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_CMD_DO_START_MAG_CAL, MAV_CMD_DO_CANCEL_MAG_CAL, MAV_CMD_DO_ACCEPT_MAG_CAL
from gcs.backend.vehicle_manager import VehicleManager

log = logging.getLogger("gcs.backend.routes.control")

def _handle_compass_cal_start(
    targets: list[int],
    params: dict | None,
    *,
    vehicle_mgr: VehicleManager,
) -> dict:
    """Start onboard magnetometer calibration on targeted vehicles.

    p1=mag_mask (0 = all compasses), p2=retry, p3=autosave (1 = save on
    success), p4=delay, p5=autoreboot (0 = do NOT auto-reboot; the operator
    triggers the required reboot explicitly after accepting). The mag_mask is
    operator-overridable via ``params['mag_mask']`` to recalibrate a single
    compass; everything else stays fixed.
    """
    mag_mask = int((params or {}).get("mag_mask", 0))
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        if entry.vehicle.is_armed:
            results[key] = "refused_armed"
            log.warning("COMPASS CAL start refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.vehicle.send_command_long(
                MAV_CMD_DO_START_MAG_CAL,
                p1=mag_mask,  # mag_mask: 0 = all compasses
                p2=0,         # retry
                p3=1,         # autosave on success
                p4=0,         # delay (s)
                p5=0,         # autoreboot: 0 = operator reboots explicitly
            )
            results[key] = "cal_started"
            log.info("COMPASS CAL: vehicle %d calibration started (mask=%d)",
                     sys_id, mag_mask)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("COMPASS CAL start failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_compass_cal_cancel(
    targets: list[int],
    params: dict | None,
    *,
    vehicle_mgr: VehicleManager,
) -> dict:
    """Cancel an in-progress magnetometer calibration on targeted vehicles."""
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        if entry.vehicle.is_armed:
            results[key] = "refused_armed"
            log.warning("COMPASS CAL cancel refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.vehicle.send_command_long(MAV_CMD_DO_CANCEL_MAG_CAL)
            results[key] = "cal_cancelled"
            log.info("COMPASS CAL: vehicle %d calibration cancelled", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("COMPASS CAL cancel failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_compass_cal_accept(
    targets: list[int],
    params: dict | None,
    *,
    vehicle_mgr: VehicleManager,
) -> dict:
    """Accept and save a completed magnetometer calibration on targeted vehicles."""
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        if entry.vehicle.is_armed:
            results[key] = "refused_armed"
            log.warning("COMPASS CAL accept refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.vehicle.send_command_long(MAV_CMD_DO_ACCEPT_MAG_CAL)
            results[key] = "cal_accepted"
            log.info("COMPASS CAL: vehicle %d calibration accepted", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("COMPASS CAL accept failed for vehicle %d: %s", sys_id, e)
    return results

def _handle_compass_cal_reboot(
    targets: list[int],
    params: dict | None,
    *,
    vehicle_mgr: VehicleManager,
) -> dict:
    """Reboot the autopilot so saved compass offsets take effect.

    Scoped to the compass-cal flow (named distinctly from the generic
    ``reboot`` command) so it stays additive and isolated. p1=1 reboots the
    autopilot; all other PREFLIGHT_REBOOT_SHUTDOWN params stay 0 (no
    companion/component action). Refuses an armed vehicle like the generic
    reboot handler (T-1-08's primary boundary is the operator ConfirmModal;
    this is defence-in-depth).
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
            log.warning("COMPASS CAL reboot refused: vehicle %d is armed", sys_id)
            continue
        try:
            entry.vehicle.send_command_long(
                MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, p1=1,
            )
            results[key] = "reboot_sent"
            log.warning("COMPASS CAL: vehicle %d autopilot reboot requested", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("COMPASS CAL reboot failed for vehicle %d: %s", sys_id, e)
    return results
