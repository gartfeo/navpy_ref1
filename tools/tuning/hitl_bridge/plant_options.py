"""What the bridge adds to the JSBSim aircraft model's servos: SITL's servo (actuator) lag.

Servo lag, the same as ArduPilot SITL's plane models. SITL filters every servo channel
(Aircraft::filtered_servo_angle / filtered_servo_range, fork libraries/SITL/SIM_Aircraft.cpp:
991-1003) with ServoModel. With SIM_SERVO_DELAY and SIM_SERVO_FILTER at their default 0 that
is the simple filter (libraries/SITL/ServoModel.cpp:53-62, chosen at :74-78): a 1-pole low pass
with cutoff 1/(2*pi*SIM_SERVO_SPEED), SIM_SERVO_SPEED default 0.14 s (ServoModel.cpp:32),
updated once per physics frame with dt = the frame time (SIM_Aircraft.cpp:993, 1002), by
LowPassFilter::apply (libraries/Filter/LowPassFilter.cpp:24-30, 119-122) and
calc_lowpass_alpha_dt (libraries/AP_Math/AP_Math.cpp:400-414):
    alpha = dt / (dt + 1/(2*pi*cutoff)) = dt / (dt + SIM_SERVO_SPEED)
    output += (sample - output) * alpha
SITL steps its physics at SIM_RATE_HZ, default 1200 Hz (fork libraries/SITL/SITL.cpp:48), the
bridge's default integration step, and the bridge updates the lag once per integration step
too. The filter is linear, so lagging the unmixed aileron/elevator commands equals SITL's
lagging of the two elevon channels before its unmix. Two negligible differences: SITL's
filter starts at its first sample, the bridge's at neutral (a board at rest outputs neutral);
SITL's frame time is whole microseconds (833 us at 1200 Hz), the bridge's is the exact step.
A servo speed of 0 disables the lag, as SIM_SERVO_SPEED 0 does (ServoModel.cpp:56-58).
The wind is in environment.py (it follows the board clock, like the catapult).

Start pose: where the physics starts, at the origin's heading. On the ground it is the aircraft at
rest on its skids (the launch profile's ground_rest, launch_profile.py; GROUND_LEVEL is only a neutral
placeholder); the simple launcher (catapult.py) holds it at rest at its own pitch and height
until the stroke; the catapult rail (rail.py, the bridge's default) starts it at the rail's end,
moving with the release velocity, because the rail itself is flown without physics.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from tools.tuning.hitl_bridge.physics import NEUTRAL_COMMAND, SurfaceCommand

DEFAULT_SERVO_SPEED_S = 0.14            # SIM_SERVO_SPEED default, fork libraries/SITL/ServoModel.cpp:32
NO_LAG_ALPHA = 1.0                      # the filter output equals its input
FT_PER_M = 1 / 0.3048
# WGS84, the ellipsoid of JSBSim's geodetic latitude (FGInertial) and of GPS positions
WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class StartPose:
    """The aircraft where the physics starts: the height of the model's reference point above the
    ground, the pitch, the offset from the origin (north_m, east_m) and the ground velocity (NED,
    m/s). on_ground: carried by its skid contacts (else held by a launcher or released by the rail)."""
    height_agl_ft: float
    pitch_deg: float
    on_ground: bool
    north_m: float = 0.0
    east_m: float = 0.0
    velocity_ned_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


def offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """The point north_m and east_m from (lat_deg, lon_deg) on the WGS84 ellipsoid, to first order
    (meridian and prime-vertical radii at the start): off by far less than a millimetre over a rail."""
    sin_lat = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    meridian_m = WGS84_A_M * (1.0 - WGS84_E2) / w ** 3
    prime_vertical_m = WGS84_A_M / w
    return (lat_deg + math.degrees(north_m / meridian_m),
            lon_deg + math.degrees(east_m / (prime_vertical_m * math.cos(math.radians(lat_deg)))))


# A neutral start on the ground: the model's reference point at ground level, wings and nose level. The
# aircraft's own rest pose on its skids (what JSBSim's ground trim finds for its model) is private data, read
# from the launch profile for --launch ground (launch_profile.ground_rest).
GROUND_LEVEL = StartPose(height_agl_ft=0.0, pitch_deg=0.0, on_ground=True)


def launcher_pose(pitch_deg: float, height_m: float) -> StartPose:
    """The simple launcher's pose: its pitch, the aircraft's height on it, at rest."""
    return StartPose(height_agl_ft=height_m * FT_PER_M, pitch_deg=pitch_deg, on_ground=False)


def servo_lag_alpha(dt_s: float, servo_speed_s: float) -> float:
    """ArduPilot's calc_lowpass_alpha_dt(dt, 1/(2*pi*servo_speed)); NO_LAG_ALPHA when the speed is 0."""
    if servo_speed_s <= 0.0:
        return NO_LAG_ALPHA
    return dt_s / (dt_s + servo_speed_s)


class ServoLag:
    """The servo lag on the three surface commands, one update per integration step."""

    def __init__(self, alpha: float) -> None:
        self.alpha = alpha
        self.output = NEUTRAL_COMMAND

    def reset(self) -> None:
        self.output = NEUTRAL_COMMAND

    def apply(self, command: SurfaceCommand) -> SurfaceCommand:
        """Advance one step towards command and return the lagged command."""
        if self.alpha >= NO_LAG_ALPHA:
            self.output = command
            return command
        out, a = self.output, self.alpha
        self.output = SurfaceCommand(aileron=out.aileron + (command.aileron - out.aileron) * a,
                                     elevator=out.elevator + (command.elevator - out.elevator) * a,
                                     throttle=out.throttle + (command.throttle - out.throttle) * a)
        return self.output


@dataclass(frozen=True)
class PlantOptions:
    """Plant settings beyond the model: the servo lag's time constant (0 = none), the start pose."""
    servo_speed_s: float = DEFAULT_SERVO_SPEED_S
    start: StartPose = GROUND_LEVEL
