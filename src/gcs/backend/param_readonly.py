"""ArduPilot parameters that must not be edited from the GCS.

The parameter blob (`param.pck`) carries no read-only flag, so this is a
curated set grounded in ArduPilot's firmware source: parameters tagged
`// @ReadOnly: True` (statistics, item counts, hardware-detected device IDs,
firmware-calibrated values). Writing them does nothing useful — the firmware
ignores or overwrites the value — so the editor disables them and the write
path rejects them.

The families below were derived by grepping `@ReadOnly: True` across the
ArduPilot tree (AP_Stats, AP_Mission, AP_Compass, AP_Baro, AP_Airspeed,
AP_InertialSensor, AP_GPS, AP_VideoTX, AP_TempCalibration, ...). Patterns are
deliberately specific: a blanket ``*_ID`` rule would wrongly lock user-settable
IDs such as ``COMPASS_PRIO*_ID``, ``FRSKY_*_ID`` and ``ADSB_ICAO_ID``.
"""
from __future__ import annotations

import re

# Exact names the firmware maintains / sets itself.
_READONLY_EXACT = frozenset({
    "MIS_TOTAL",       # mission item count — set by mission upload
    "FENCE_TOTAL",     # fence point count — set by fence upload
    "RALLY_TOTAL",     # rally point count — set by rally upload
    "SYS_NUM_RESETS",  # board boot / reset counter
    "VTX_FREQ",        # video-TX frequency — auto-derived from band/channel
})

# Whole families identified by a name prefix.
_READONLY_PREFIXES = ("STAT_",)  # STAT_BOOTCNT, STAT_FLTTIME, STAT_RUNTIME, STAT_RESET

# Hardware-detected sensor device IDs, caught by these substrings:
#   COMPASS_DEV_ID(2..8), BARO*_DEVID, ARSPD*_DEVID, SIM_MAG*_DEVID.
_READONLY_ID_MARKERS = ("DEVID", "DEV_ID")

# Read-only families the substring markers miss (their names use *_ID /
# *_NODEID / *_GND_PRESS). Anchored + specific to avoid locking writable IDs.
_READONLY_PATTERNS = tuple(re.compile(p) for p in (
    r"^INS\d*_(ACC|GYR)\d*_ID$",                # IMU accel/gyro device IDs
    r"^GPS\d*_CAN_NODEID\d*$",                  # auto-discovered DroneCAN GPS node IDs
    r"^BARO\d*_GND_PRESS$",                     # firmware-calibrated baro ground pressure (primary = no digit)
    r"^TCAL\d*_(TEMP_MIN|TEMP_MAX|BARO_EXP)$",  # temperature-calibration learned values
))


def is_readonly(name: str) -> bool:
    """True if `name` is a firmware-maintained (read-only) parameter."""
    n = (name or "").upper()
    if n in _READONLY_EXACT:
        return True
    if n.startswith(_READONLY_PREFIXES):
        return True
    if any(marker in n for marker in _READONLY_ID_MARKERS):
        return True
    return any(p.match(n) for p in _READONLY_PATTERNS)
