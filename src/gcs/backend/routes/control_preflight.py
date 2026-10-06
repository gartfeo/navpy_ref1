"""Shared pitot-safe manual and automatic preflight calibration."""
from __future__ import annotations

import asyncio
import logging
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_PREFLIGHT_CALIBRATION
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from gcs.backend.vehicle_manager import VehicleManager
from gcs.backend.settings_model import GcsSettings

log = logging.getLogger("gcs.backend.routes.control")

def _send_gyro(vehicle: VehicleMav) -> None:
    """Gyro offset calibration (p1=1). Always safe to (re-)run; never touches
    the airspeed sensor."""
    vehicle.send_command_long(MAV_CMD_PREFLIGHT_CALIBRATION, p1=1)

def _send_baro(vehicle: VehicleMav) -> None:
    """Baro / ground-pressure calibration (p3=1).

    Sent as its own command (not combined with gyro): ArduPilot's
    MAV_CMD_PREFLIGHT_CALIBRATION handler returns after the first matching
    param, so gyro (p1=1) and baro (p3=1) in one command would only run gyro.

    On ArduPilot the p3=1 handler ALSO re-zeros the airspeed sensor when a pitot
    is fitted, and a valid airspeed zero needs the pitot covered / calm air —
    so callers MUST gate this on :func:`_pitot_baro_ok`.
    """
    vehicle.send_command_long(MAV_CMD_PREFLIGHT_CALIBRATION, p3=1)  # baro (+ airspeed zero if pitot fitted)

def _pitot_baro_ok(vehicle: VehicleMav, pitot_covered: bool) -> bool:
    """Whether baro (p3=1) may be sent without risking a bad airspeed zero.

    Single source of truth shared by the manual and auto-START cal paths so they
    can never diverge. Safe only when the vehicle *definitely* has no pitot
    (``airspeed_present is False``), or the operator acknowledged the pitot is
    covered / air is calm. Unknown presence (``None`` — no SYS_STATUS yet) is
    treated conservatively as "might have a pitot" → not ok unless acknowledged.
    """
    return vehicle.airspeed_present is False or pitot_covered

def _handle_preflight_cal(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Manual preflight calibration, disarmed vehicles only.

    Always sends gyro (p1=1). Sends baro (p3=1) only when :func:`_pitot_baro_ok`
    holds — i.e. the vehicle has no pitot, or the operator acknowledged the pitot
    is covered (``params['pitot_covered']``, set by the frontend confirm). This
    is defence-in-depth: the UI already gates the confirm, but the airspeed-zero
    safety invariant must not depend solely on the client.

    Send-and-done. An armed vehicle is refused rather than sent a calibration
    command mid-flight (the autopilot would reject it anyway; we fail fast and
    report a clear result).
    """
    pitot_covered = bool((params or {}).get("pitot_covered", False))
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = "not_connected"
            continue
        if entry.vehicle.is_armed:
            results[key] = "refused_armed"
            log.warning("preflight cal refused for vehicle %d: armed", sys_id)
            continue
        try:
            _send_gyro(entry.vehicle)
            if _pitot_baro_ok(entry.vehicle, pitot_covered):
                _send_baro(entry.vehicle)
                results[key] = "preflight_cal_sent"
                log.info("preflight cal (gyro+baro) sent to vehicle %d", sys_id)
            else:
                results[key] = "preflight_cal_sent_gyro_only"
                log.info("preflight cal: gyro only on vehicle %d — baro/airspeed "
                         "skipped (pitot present/unknown, not acknowledged covered)", sys_id)
        except Exception as e:
            results[key] = f"error: {e}"
            log.error("preflight cal failed for vehicle %d: %s", sys_id, e)
    return results

async def _maybe_auto_preflight_cal(
    sys_ids: list[int],
    settings: GcsSettings,
    pitot_covered: bool = False,
    *,
    vehicle_mgr: VehicleManager,
) -> list[int]:
    """When enabled, run gyro (+ baro/airspeed) preflight cal on disarmed targets.

    Opt-in via ``LaunchSettings.auto_preflight_cal`` (default off), so no run is
    affected unless the operator turns it on. Only disarmed vehicles are touched.

    Gyro (p1=1) is always sent — safe to re-run. Baro (p3=1) ALSO re-zeros the
    airspeed sensor on ArduPilot when a pitot is fitted, and that zero is only
    valid with the pitot covered / calm air — which START cannot verify. So:

    - vehicle with **no pitot** (airspeed sensor not present): gyro + baro (pure
      baro, always safe);
    - vehicle **with a pitot** (or presence not yet known): gyro only, UNLESS the
      operator acknowledged *pitot_covered*, in which case gyro + baro/airspeed.

    After sending, wait ``preflight_cal_settle_s`` so the gyro cal completes
    before the caller arms (arming mid-calibration is rejected by the autopilot).
    Returns the list of calibrated sys_ids (for logging/tests).
    """
    ls = settings.launch
    if not getattr(ls, "auto_preflight_cal", False):
        return []
    caled = []
    skipped_airspeed = []
    for sys_id in sys_ids:
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry or entry.vehicle.is_armed:
            continue
        try:
            _send_gyro(entry.vehicle)  # always safe
            # Baro (+ airspeed zero) only when we KNOW there's no pitot, or the
            # operator acknowledged it's covered — same gate as the manual path.
            if _pitot_baro_ok(entry.vehicle, pitot_covered):
                _send_baro(entry.vehicle)
            else:
                skipped_airspeed.append(sys_id)
            caled.append(sys_id)
        except Exception as e:
            log.error("[launch] auto preflight cal failed for V%d: %s", sys_id, e)
    if skipped_airspeed:
        log.info("[launch] auto cal: gyro only on %s — baro/airspeed skipped "
                 "(pitot present, not acknowledged covered)", skipped_airspeed)
    if caled:
        settle = max(0.0, float(getattr(ls, "preflight_cal_settle_s", 2.0)))
        log.info("[launch] auto preflight cal sent to %s; settling %.1fs", caled, settle)
        if settle > 0:
            await asyncio.sleep(settle)
    return caled
