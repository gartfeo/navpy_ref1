"""Read-only launch readiness policy and channel mapping."""
from __future__ import annotations

import logging
from gcs.backend.vehicle_manager import VehicleManager
from gcs.backend.settings_model import LaunchSettings

log = logging.getLogger("gcs.backend.routes.control")

from dataclasses import dataclass
from navpy.modules.vehicle.vehicle_mav import (
    PREARM_STATE_NO_SYS_STATUS, PREARM_STATE_NOT_REPORTED, PREARM_STATE_CHECKS_DISABLED,
    PREARM_STATE_FAILED, PREARM_STATE_OK,
)

_GPS_FIX_3D = 3  # MAVLink GPS_FIX_TYPE_3D_FIX — protocol constant, not a policy knob
_ESP32_NUM_CHANNELS = 6  # ESP32 launch controller relay channels (1..6)


def _resolve_channel_map(default_map: dict, sys_ids: list[int]) -> dict[int, int]:
    """Resolve the sys_id -> ESP32 channel map for a launch.

    Explicit mappings from settings win. Any vehicle without one is
    auto-assigned the next free channel (1..N) in ascending sys_id order, so a
    sys_id outside 1..N is never sent to the ESP32 as an invalid channel. With
    more vehicles than channels, the surplus are left unmapped (logged).
    """
    channel_map = {int(k): int(v) for k, v in default_map.items()}
    used = set(channel_map.values())
    free = (c for c in range(1, _ESP32_NUM_CHANNELS + 1) if c not in used)
    for sid in sorted(sys_ids):
        if sid in channel_map:
            continue
        ch = next(free, None)
        if ch is None:
            log.warning("[launch] V%d: no free ESP32 channel (>%d vehicles) — "
                        "leaving unmapped", sid, _ESP32_NUM_CHANNELS)
            continue
        channel_map[sid] = ch
    return channel_map


@dataclass(frozen=True)
class ReadinessGates:
    """Tunable launch-readiness gates.

    Defaults reproduce the original hardcoded /launch behavior exactly and are
    what /restart always uses (restart gating is not operator-tunable). The
    GPS 3D-fix threshold itself stays fixed (protocol constant); only whether
    the GPS check runs is toggleable.
    """
    check_prearm: bool = True
    check_gps: bool = True
    check_gps_acc: bool = False
    max_gps_hacc_m: float = 1.0
    check_throttle: bool = True
    max_throttle_rc3: int = 1050
    check_battery: bool = True
    min_battery_pct: float = 15.0
    block_on_unknown_battery: bool = False


def _readiness_gates_from_settings(ls: LaunchSettings) -> ReadinessGates:
    """Build readiness gates from LaunchSettings (used by /launch only)."""
    return ReadinessGates(
        check_prearm=ls.check_prearm_enabled,
        check_gps=ls.check_gps_enabled,
        check_gps_acc=ls.check_gps_acc_enabled,
        max_gps_hacc_m=ls.max_gps_hacc_m,
        check_throttle=ls.check_throttle_enabled,
        max_throttle_rc3=ls.max_throttle_rc3,
        check_battery=ls.check_battery_enabled,
        min_battery_pct=ls.min_battery_pct,
        block_on_unknown_battery=ls.block_on_unknown_battery,
    )


def _check_launch_readiness(
    sys_ids: list[int],
    gates: ReadinessGates | None = None,
    *,
    vehicle_mgr: VehicleManager,
) -> list[str]:
    """Return list of issues blocking launch across all requested vehicles.

    *gates* tunes prearm/telemetry checks and their thresholds. ``None`` selects
    their fixed defaults for /restart. Companion liveness is always checked;
    callers retain the existing force override by bypassing readiness entirely.
    """
    if gates is None:
        gates = ReadinessGates()
    issues = []
    for sys_id in sys_ids:
        entry = vehicle_mgr.get_vehicle(sys_id)
        if not entry:
            issues.append(f"Vehicle {sys_id}: not connected")
            continue
        v = entry.vehicle
        prefix = f"Vehicle {sys_id}"
        # Match current tri-state liveness; transient heartbeat loss is allowed.
        # The legacy binary is_companion_active check would reject "checking".
        try:
            companion_status = vehicle_mgr.companion_status(sys_id)
        except Exception:
            log.exception("[launch] companion status unavailable for V%d", sys_id)
            companion_status = None
        if companion_status == "down":
            issues.append(f"{prefix}: companion computer not connected")
        elif companion_status not in ("ok", "checking"):
            issues.append(f"{prefix}: companion status unavailable")
        state = v.prearm_check_state
        # Pure read (stale-flag maintenance lives in VehicleEntry.snapshot via
        # _refresh_prearm_flags); kept in a local for the log + branch below.
        mode_only = entry.prearm_mode_not_armable_only
        log.debug("[launch] readiness V%d: prearm_state=%s, gps_fix=%s, rc3=%s, "
                  "battery=%s, mission_uploaded=%s, mission_count=%s, "
                  "prearm_mode_only=%s",
                  sys_id, state, v.gps_fix_type, v.rc3_raw,
                  v.battery_level, entry.mission_uploaded,
                  v.mission_items_count, mode_only)
        if gates.check_prearm:
            if state == PREARM_STATE_NO_SYS_STATUS:
                issues.append(f"{prefix}: waiting for pre-arm status")
            elif state == PREARM_STATE_NOT_REPORTED:
                issues.append(f"{prefix}: pre-arm status not reported")
            elif state == PREARM_STATE_FAILED:
                # "Mode not armable" alone is not a real failure (vehicle armable
                # once mode changes); any other failure blocks launch.
                if not mode_only:
                    issues.append(f"{prefix}: pre-arm check failed")
            elif state in (PREARM_STATE_CHECKS_DISABLED, PREARM_STATE_OK):
                pass  # armable — arming checks disabled or passing
            else:
                # Fail closed: an unknown/unexpected state must never allow launch.
                issues.append(f"{prefix}: unknown pre-arm status ({state})")
        if gates.check_gps and (v.gps_fix_type is None or v.gps_fix_type < _GPS_FIX_3D):
            issues.append(f"{prefix}: no GPS 3D fix")
        if gates.check_gps_acc and v.gps_hacc is not None and v.gps_hacc > gates.max_gps_hacc_m:
            issues.append(f"{prefix}: GPS accuracy {v.gps_hacc:.1f}m > {gates.max_gps_hacc_m:.1f}m")
        if gates.check_throttle and v.rc3_raw is not None and v.rc3_raw > gates.max_throttle_rc3:
            issues.append(f"{prefix}: throttle not zero")
        if gates.check_battery:
            if v.battery_level is not None and v.battery_level < gates.min_battery_pct:
                issues.append(f"{prefix}: battery low ({v.battery_level:.0f}%)")
            elif v.battery_level is None and gates.block_on_unknown_battery:
                # Closes the gap where an unconfigured/unreporting battery monitor
                # silently passes both prearm and this gate.
                issues.append(f"{prefix}: battery telemetry missing")
        if not entry.mission_uploaded and not (v.mission_items_count or 0) > 0:
            issues.append(f"{prefix}: mission not uploaded")
    return issues
