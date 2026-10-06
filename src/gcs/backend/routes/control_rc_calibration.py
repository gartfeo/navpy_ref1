"""Validate and persist radio calibration values."""
from __future__ import annotations

import logging
from typing import Any
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from gcs.backend.vehicle_manager import VehicleManager

log = logging.getLogger("gcs.backend.routes.control")

_RC_CAL_MAX_CHANNELS = 16   # ArduPilot exposes writable RC1..RC16 params
_RC_CAL_PWM_MIN = 800       # plausible RC PWM lower bound (µs)
_RC_CAL_PWM_MAX = 2200      # plausible RC PWM upper bound (µs)


def _coerce_pwm(value: Any) -> int | None:
    """Round *value* to an int PWM within the plausible range, or None."""
    try:
        pwm = int(round(float(value)))
    except (TypeError, ValueError):
        return None
    if pwm < _RC_CAL_PWM_MIN or pwm > _RC_CAL_PWM_MAX:
        return None
    return pwm

def _write_rc_cal(vehicle: VehicleMav, sys_id: int, channels: dict) -> dict:
    """Write RCn_MIN/MAX/TRIM/REVERSED for each calibrated channel.

    *channels* maps a channel number (1..16) to ``{min, max, trim, reversed}``.
    Invalid channels are skipped and reported; trim defaults to the channel
    midpoint and is always clamped into ``[min, max]``. Returns per-parameter
    write success (True only when the autopilot echoed a matching value).
    """
    param_results: dict[str, bool] = {}
    channel_errors: dict[str, str] = {}
    for ch_key, cal in channels.items():
        try:
            ch = int(ch_key)
        except (TypeError, ValueError):
            channel_errors[str(ch_key)] = "invalid_channel"
            continue
        if ch < 1 or ch > _RC_CAL_MAX_CHANNELS:
            channel_errors[str(ch_key)] = "channel_out_of_range"
            continue

        cal = cal or {}
        cmin = _coerce_pwm(cal.get("min"))
        cmax = _coerce_pwm(cal.get("max"))
        if cmin is None or cmax is None or cmin >= cmax:
            channel_errors[str(ch)] = "invalid_min_max"
            continue

        ctrim = _coerce_pwm(cal.get("trim"))
        if ctrim is None:
            ctrim = (cmin + cmax) // 2
        ctrim = max(cmin, min(cmax, ctrim))
        reversed_flag = 1 if cal.get("reversed") else 0

        writes = {
            f"RC{ch}_MIN": cmin,
            f"RC{ch}_MAX": cmax,
            f"RC{ch}_TRIM": ctrim,
            f"RC{ch}_REVERSED": reversed_flag,
        }
        for name, value in writes.items():
            try:
                ok = vehicle.set_parameter(name, value, timeout=2.0)
            except Exception as e:  # one bad write must not abort the rest
                ok = False
                log.error("RC cal: %s write error on V%d: %s", name, sys_id, e)
            param_results[name] = bool(ok)

    if not param_results:
        status = "no_valid_channels"
    elif not all(param_results.values()):
        status = "partial"
    else:
        status = "saved"
    result = {"status": status, "params": param_results}
    if channel_errors:
        result["channel_errors"] = channel_errors
    return result

def _handle_rc_cal_save(targets: list[int], params: dict | None, *, vehicle_mgr: VehicleManager) -> dict:
    """Persist RC transmitter calibration to the targeted vehicles.

    *params* shape: ``{"channels": {"<n>": {min, max, trim, reversed}}}``.
    Refuses to write while a vehicle is armed (radio calibration must happen on
    a disarmed vehicle). Returns per-vehicle, per-parameter write results.

    ``channels`` is client-supplied JSON — a malformed shape (e.g. a list
    instead of a dict) must not crash the whole /api/control/command request
    (the dispatcher invokes handlers uncaught); any per-vehicle write failure
    is caught and reported the same way every other cal handler's exceptions
    are, matching the standard not_connected/refused_armed/error results shape.
    """
    channels = (params or {}).get("channels") or {}
    results = {}
    for sys_id in targets:
        key = str(sys_id)
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            results[key] = {"status": "not_connected"}
            continue
        if entry.vehicle.is_armed:
            log.warning("RC cal: vehicle %d is armed — refusing to write", sys_id)
            results[key] = {"status": "refused_armed"}
            continue
        try:
            results[key] = _write_rc_cal(entry.vehicle, sys_id, channels)
        except Exception as e:
            results[key] = {"status": f"error: {e}"}
            log.error("RC cal: vehicle %d save failed: %s", sys_id, e)
            continue
        log.info("RC cal: vehicle %d save result=%s", sys_id, results[key]["status"])
    return results
