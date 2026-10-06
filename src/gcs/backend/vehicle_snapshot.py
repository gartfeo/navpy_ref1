"""Assemble operator telemetry from one live vehicle entry."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gcs.backend.vehicle_entry import VehicleEntry

log = logging.getLogger("gcs.backend.vehicle_manager")

def snapshot(entry: VehicleEntry) -> dict:
    """Build a telemetry snapshot dict for WebSocket broadcast."""
    v = entry.vehicle
    # Maintain stale PreArm-flag state every cycle (see _refresh_prearm_flags):
    # the launch gate must not rely on having been polled during an armable
    # transition.
    entry._refresh_prearm_flags()
    entry._refresh_confirm_blocked()
    loc = v.location(is_relative=False)
    loc_rel = v.location(is_relative=True)
    mode = v.get_mode
    att = v.attitude
    armed = v.is_armed
    # Drain pending STATUSTEXT messages
    texts = []
    while entry._status_texts:
        try:
            texts.append(entry._status_texts.popleft())
        except IndexError:
            break
    # Drain pending accel-cal events (relayed as accel_cal_step by the loop)
    cal_events = []
    while entry._accel_cal_events:
        try:
            cal_events.append(entry._accel_cal_events.popleft())
        except IndexError:
            break
    snap = {
        "sys_id": entry.sys_id,
        "name": entry.name,
        "battery": v.battery_level,
        "voltage": v.battery_voltage,
        "current": v.battery_current,
        "mode": mode.name if mode else None,
        "armed": armed,
        "lat": loc.lat if loc else None,
        "lon": loc.lng if loc else None,
        "alt": loc.alt if loc else None,
        "alt_rel": loc_rel.alt if loc_rel else None,
        "heading": v.heading,
        "air_speed": v.air_speed,
        "ground_speed": v.ground_speed,
        "climb": getattr(v, "climb_rate", None),
        "throttle": getattr(v, "throttle_pct", None),
        "gps_fix": v.gps_fix_type,
        "gps_sats": v.gps_satellites,
        "gps_hacc": v.gps_hacc,
        "link_ok": v.link_ok,
        "link_quality": v.link_quality,
        "mission_progress": v.mission_items_next,
        "mission_total": v.mission_items_count,
        "mission_uploaded": entry.mission_uploaded,
        "is_probing": entry.is_probing,
        "mission_download_progress": entry.mission_download_progress,
        "roll": att.roll if att else None,
        "pitch": att.pitch if att else None,
        "yaw": att.yaw if att else None,
        "prearm_ok": v.prearm_ok,
        "prearm_check_state": v.prearm_check_state,
        "ekf": v.ekf_status,
        "sensors": v.sensor_health,
        "airspeed_present": v.airspeed_present,
        "airspeed_ok": v.airspeed_healthy,
        "rc3": v.rc3_raw,
        "gimbals": entry._gimbal_snapshot(),
        "confirm_blocked": entry._confirm_blocked,
    }
    rc_channels = entry._rc_channels_snapshot()
    if rc_channels is not None:
        snap["rc_channels"] = rc_channels
    if texts:
        snap["status_texts"] = texts
    if cal_events:
        snap["accel_cal_events"] = cal_events
    return snap
