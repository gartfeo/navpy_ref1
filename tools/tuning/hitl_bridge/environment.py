"""The wind the bridge adds to the JSBSim model: constant and horizontal, on the board clock.

Given as the speed and the direction it blows FROM (degrees true, the meaning SIM_WIND_DIR
documents, fork libraries/SITL/SITL.cpp:89-94); the default is calm. As in ArduPilot SITL,
the air is still for the first WIND_START_BOARD_TIME_S after the board booted, so the
airspeed sensor calibrates its zero at zero airspeed (SITL_State.cpp:327-331: "give 5 seconds
to calibrate airspeed sensor at 0 wind speed"). Board time is micros64 since boot, so this
also holds after a board reboot mid-run. Then the wind is on at full strength: SITL's
SIM_WIND_TC slew is not modelled (the aircraft is on its launcher or the ground and disarmed then).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

US_PER_S = 1_000_000
WIND_START_BOARD_TIME_S = 5.0           # SITL_State.cpp:331, the airspeed calibration window after boot


@dataclass(frozen=True)
class Wind:
    """A constant horizontal wind: speed, and the direction it blows from (degrees true)."""
    speed_m_s: float = 0.0
    from_deg: float = 0.0

    def air_velocity_ne_m_s(self) -> tuple[float, float]:
        """Velocity of the air mass (north, east): it moves towards from_deg + 180."""
        rad = math.radians(self.from_deg)
        # 0.0 - x rather than -x: a calm wind is +0.0, not -0.0, in the commands sent
        return 0.0 - self.speed_m_s * math.cos(rad), 0.0 - self.speed_m_s * math.sin(rad)


CALM = Wind()


def wind_at(wind: Wind, board_time_us: float) -> Wind:
    """The wind at this board time: calm during the airspeed calibration window after boot."""
    return wind if board_time_us >= WIND_START_BOARD_TIME_S * US_PER_S else CALM
