"""Eligible airspeed values, revoke reasons and the simulated pitot profile (hitl_guidance.md section 9, step 20).

The profile is simulator-only: ``profile_for_mode`` knows only pc-sitl, so any flight or Qualification mode refuses
it. The approach runner pins the same profile and the firmware mapping on the board before arming
(tools/tuning/approach_airspeed.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import pymavlink.dialects.v20.all as all_dialect

AIRSPEED_MSG_ID = all_dialect.MAVLINK_MSG_ID_AIRSPEED                       # 295
SET_MESSAGE_INTERVAL = all_dialect.MAV_CMD_SET_MESSAGE_INTERVAL             # 511
DIFFERENTIAL_PRESSURE = all_dialect.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
FLAG_UNHEALTHY = all_dialect.AIRSPEED_SENSOR_UNHEALTHY
FLAG_USING = all_dialect.AIRSPEED_SENSOR_USING
AUTOPILOT_COMPONENT = all_dialect.MAV_COMP_ID_AUTOPILOT1
MAV_RESULT_ACCEPTED = all_dialect.MAV_RESULT_ACCEPTED

# The plan's values (step 20).
AIRSPEED_INTERVAL_US = 50_000           # 20 Hz claim
ADMIT_MIN_RATE_HZ = 18.0                # over the healthy window
ADMIT_WINDOW_S = 60.0                   # healthy seconds before admission
# My values (listed in the step's Built record).
AIRSPEED_MAX_AGE_S = 0.2                # 4 claimed periods; step 11's source age limit
STATUS_MAX_AGE_S = 0.5                  # 5 periods of the runner's 10 Hz streams (SYS_STATUS is not claimed)
CLAIM_ACK_TIMEOUT_S = 3.0               # wait for the claim's COMMAND_ACK
PARAM_READ_TIMEOUT_S = 2.0              # per pinned parameter read

PENDING = "pending"
ADMITTED = "admitted"
REVOKED = "revoked"

# Revoke reasons, named in the log.
CLAIM_REFUSED = "claim_refused"
CLAIM_UNACKNOWLEDGED = "claim_unacknowledged"
PIN_MISMATCH = "pin_mismatch"
PIN_UNREADABLE = "pin_unreadable"
PIN_CHANGED = "pin_changed"
SENSOR_UNHEALTHY = "sensor_unhealthy"
SENSOR_NOT_USED = "sensor_not_used"
SENSOR_VALUE_INVALID = "sensor_value_invalid"
SENSOR_STALE = "sensor_stale"
STATUS_BAD = "status_bad"
STATUS_STALE = "status_stale"
SOURCE_CHANGED = "source_changed"


@dataclass(frozen=True)
class AirspeedProfile:
    """The simulated pitot source a HITL run may admit: the board's effective parameters."""

    mode: str
    arspd_type: int             # the board's sensor driver (ARSPD_TYPE)
    arspd_use: int              # 1: the AHRS uses the sensor
    sensor: int                 # ARSPD_PRIMARY: the selected sensor's index, AIRSPEED.id

    def pins(self) -> dict[str, int]:
        return {"ARSPD_TYPE": self.arspd_type, "ARSPD_USE": self.arspd_use, "ARSPD_PRIMARY": self.sensor}


# Simulator only: there is no flight profile, so a flight or Qualification mode refuses (profile_for_mode).
# pc-sitl: SITL stands in for the board (approach_chain.backing), so no hitl_board.parm is loaded and ARSPD_TYPE is
# SITL ArduPlane's default 2, the analog driver reading SITL's simulated pitot pin (AP_HAL_SITL sitl_airspeed.cpp),
# which SIM_HIL feeds from the bridge's ind_airspeed (JSBSim's calibrated airspeed); ARSPD_USE 1 comes from
# Tools/autotest/models/plane.parm. A real analog pitot has the same type: the runner's firmware pin (a SITL build,
# tools/tuning/approach_airspeed.py) is what makes it simulated.
SIMULATED_PROFILES = {"pc-sitl": AirspeedProfile("pc-sitl", arspd_type=2, arspd_use=1, sensor=0)}


def profile_for_mode(mode: str) -> AirspeedProfile:
    profile = SIMULATED_PROFILES.get(mode)
    if profile is None:
        raise ValueError(f"no eligible airspeed source for mode {mode!r}: the simulated pitot profile is "
                         f"for {sorted(SIMULATED_PROFILES)} only")
    return profile


def pin_matches(value: object, expected: int) -> bool:
    """A parameter read (a float on the wire) holds exactly the pinned integer."""
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            and abs(value - expected) < 1e-6)


__all__ = [
    "ADMITTED", "ADMIT_MIN_RATE_HZ", "ADMIT_WINDOW_S", "AIRSPEED_INTERVAL_US", "AIRSPEED_MAX_AGE_S",
    "AirspeedProfile", "PENDING", "REVOKED", "SIMULATED_PROFILES", "STATUS_MAX_AGE_S", "pin_matches",
    "profile_for_mode",
]
