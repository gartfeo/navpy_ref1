"""Pure logic of the HITL JSBSim bridge: no I/O, no JSBSim, no MAVLink.

The link contract (HIL link contract v2 of the fork's SIM_HIL backend,
libraries/SITL/examples/HIL/readme.md) has the flight controller send HIL_ACTUATOR_CONTROLS
once per main loop and the bridge answer each one with HIL_STATE_QUATERNION at the same
time_usec. This module holds what the bridge computes in between, so it can be tested
without a board or a simulator:

- unmix_elevon: servo PWM to the JSBSim surface commands, the same arithmetic as
  ArduPilot's SIM_JSBSim elevon frame;
- Launch: how the aircraft leaves its start pose, armed by the first armed bit: the catapult
  rail (rail.py, the default) or the simple launcher and the skids (catapult.py). It gives the
  force on the plant by board time, holds the plant unstepped while the aircraft is on the
  launcher, and gives the state the board reads meanwhile;
- Plant: the narrow interface to the physics (below).
board_clock.BoardClockStepper steps a Plant on the board clock; state_fields turns a PlantState
into the HIL_STATE_QUATERNION fields.

The Plant protocol is the narrow interface to the physics: jsbsim_process.JSBSimProcessPlant
(the JSBSim program, the default) or jsbsim_plant.JSBSimPlant (the jsbsim Python package).
Both model the servos (plant_options: SITL's servo lag) and the wind.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

from tools.tuning.hitl_bridge.environment import Wind

# --- PWM to surfaces --------------------------------------------------------------
# ArduPilot normalises servo PWM with ServoModel::filter_angle / filter_range
# (libraries/SITL/ServoModel.cpp:115-125) over the default range 1000..2000 us
# (ServoModel.h:47-48), which SIM_JSBSim never changes.
PWM_MIN_US = 1000.0
PWM_MAX_US = 2000.0
PWM_CENTRE_US = 0.5 * (PWM_MAX_US + PWM_MIN_US)
PWM_HALF_RANGE_US = 0.5 * (PWM_MAX_US - PWM_MIN_US)
# Contract: a control value of 0 means "no output on this channel". A servo without a
# signal does not move to an end stop, so it is held at neutral (angle 0, throttle 0)
# instead of feeding 0 us through the formula (which would give -3, three times full).
NO_OUTPUT_PWM_US = 0.0
# Channel indices in HIL_ACTUATOR_CONTROLS.controls (channels 1..3 of the elevon frame)
LEFT_ELEVON_INDEX = 0
RIGHT_ELEVON_INDEX = 1
THROTTLE_INDEX = 2


def angle_from_pwm(pwm_us: float) -> float:
    """-1..1 deflection from PWM, as ServoModel::filter_angle (ServoModel.cpp:121-125)."""
    if pwm_us == NO_OUTPUT_PWM_US:
        return 0.0
    return (pwm_us - PWM_CENTRE_US) / PWM_HALF_RANGE_US


def range_from_pwm(pwm_us: float) -> float:
    """0..1 throttle from PWM, as ServoModel::filter_range (ServoModel.cpp:115-119)."""
    if pwm_us == NO_OUTPUT_PWM_US:
        return 0.0
    return (pwm_us - PWM_MIN_US) / (PWM_MAX_US - PWM_MIN_US)


@dataclass(frozen=True)
class SurfaceCommand:
    """JSBSim command inputs fcs/aileron-cmd-norm, fcs/elevator-cmd-norm, fcs/throttle-cmd-norm."""
    aileron: float
    elevator: float
    throttle: float


NEUTRAL_COMMAND = SurfaceCommand(aileron=0.0, elevator=0.0, throttle=0.0)


def unmix_elevon(controls: Sequence[float]) -> SurfaceCommand:
    """Servo PWM (HIL_ACTUATOR_CONTROLS.controls) to JSBSim surface commands.

    Copied from the fork's libraries/SITL/SIM_JSBSim.cpp:342-353 (JSBSim::send_servos,
    FRAME_ELEVON branch):
        float aileron  = filtered_servo_angle(input, 0);
        float elevator = filtered_servo_angle(input, 1);
        float throttle = filtered_servo_range(input, 2);
        ...
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;
    The servo filter (ServoModel.cpp:53-62, a 1-pole lag of SIM_SERVO_SPEED) is not part of
    the unmix: the plant applies it per integration step (plant_options.ServoLag).
    """
    ch1 = angle_from_pwm(controls[LEFT_ELEVON_INDEX])
    ch2 = angle_from_pwm(controls[RIGHT_ELEVON_INDEX])
    return SurfaceCommand(aileron=(ch2 - ch1) / 2.0,
                          elevator=-(ch2 + ch1) / 2.0,
                          throttle=range_from_pwm(controls[THROTTLE_INDEX]))


# --- Plant ------------------------------------------------------------------------
@dataclass(frozen=True)
class Origin:
    """Where the aircraft starts: the ground under its start pose (= the board's SIM_OPOS_*)."""
    lat_deg: float
    lon_deg: float
    ground_alt_msl_m: float
    heading_deg: float


@dataclass(frozen=True)
class PlantState:
    """Aircraft state in SI units. Body axes are x forward, y right, z down."""
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    p_rad_s: float              # body rates
    q_rad_s: float
    r_rad_s: float
    vn_m_s: float               # NED ground velocity
    ve_m_s: float
    vd_m_s: float
    ias_m_s: float              # calibrated (indicated) airspeed
    tas_m_s: float              # true airspeed
    fx_m_s2: float              # body specific force at the CG: what an accelerometer
    fy_m_s2: float              # measures, gravity reaction included (level at rest:
    fz_m_s2: float              # (0, 0, -g))


class Plant(Protocol):
    """The physics the bridge steps. One step() advances exactly dt_s.

    A plant may defer the work of step() until state() is asked for (the JSBSim program
    plant sends one 'iterate N' for N equal steps), so state() is where it must be done.
    """
    dt_s: float

    def reset(self) -> None:
        """Put the aircraft at the origin in its start pose (plant_options.StartPose), at rest, commands neutral."""

    def step(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind) -> None:
        """Advance one integration step with these inputs held (command: before the servo lag)."""

    def state(self) -> PlantState:
        """The state after the last step (or reset)."""

    def close(self) -> None:
        """Release the physics (stop a simulator process); called once on every exit path."""


# --- Launch -----------------------------------------------------------------------
class Launch(Protocol):
    """How the aircraft leaves its start pose (catapult.Catapult, rail.RailLaunch). It fires once
    per plant initialisation, a set delay after the armed bit first appears (note); reset()
    re-arms it (board reboot). holds: it holds the aircraft on a launcher at first, and the plant
    is not stepped while holding() is true."""
    holds: bool

    def note(self, board_time_us: int, armed: bool) -> None:
        """Remember the first armed frame."""

    def reset(self) -> None:
        """Forget the arming: the next armed frame fires the launch again."""

    def force_at(self, board_time_us: float) -> float:
        """The force (N) on the plant's catapult reaction at this board time."""

    def holding(self, board_time_us: float) -> bool:
        """True while the launcher holds the aircraft: the plant must not be stepped."""

    def held_state(self, plant_state: PlantState, board_time_us: float, wind: Wind) -> PlantState:
        """The state the board reads while the aircraft is on the launcher (the plant not yet stepped)."""
