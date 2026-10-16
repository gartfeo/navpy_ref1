"""A plant state (physics.PlantState, SI units) to the HIL_STATE_QUATERNION fields (HIL link contract v2),
NED vectors in body axes, and the state of an aircraft held at rest by the simple launcher (on_launcher)."""
from __future__ import annotations

import math
from dataclasses import replace

from tools.tuning.hitl_bridge.physics import PlantState

# mG are thousandths of standard gravity; the board turns them back into m/s^2 with
# ArduPilot's GRAVITY_MSS (libraries/AP_Math/definitions.h:48), the same 9.80665.
STANDARD_GRAVITY_M_S2 = 9.80665
MILLI_G_PER_G = 1000.0
CM_PER_M = 100.0
MM_PER_M = 1000.0
DEG_E7 = 1e7
# MAVLink field ranges (HIL_STATE_QUATERNION: vx/vy/vz, xacc/yacc/zacc int16_t,
# ind_airspeed/true_airspeed uint16_t): values outside are clamped, not wrapped.
INT16_MIN = -32768
INT16_MAX = 32767
UINT16_MAX = 65535


def _clamp_int(value: float, lo: int, hi: int) -> int:
    return int(min(hi, max(lo, round(value))))


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Body-to-NED quaternion (w, x, y, z) from 3-2-1 Euler angles, as ArduPilot's
    QuaternionT::from_euler (libraries/AP_Math/quaternion.cpp:421-434)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def ned_to_body(vector_ned: tuple[float, float, float], roll: float, pitch: float,
                yaw: float) -> tuple[float, float, float]:
    """A NED vector in body axes for 3-2-1 Euler angles (the inverse of the quaternion's rotation)."""
    n, e, d = vector_ned
    sr, cr, sp, cp, sy, cy = (math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch),
                              math.sin(yaw), math.cos(yaw))
    x1, y1 = cy * n + sy * e, -sy * n + cy * e          # about z by the yaw
    x2, z2 = cp * x1 - sp * d, sp * x1 + cp * d          # about y by the pitch
    return x2, cr * y1 + sr * z2, -sr * y1 + cr * z2     # about x by the roll


def at_rest(state: PlantState) -> PlantState:
    """The aircraft at rest where the state is, in its attitude: no motion, no airspeed, and the accelerometer
    reading the support against gravity in body axes (see on_launcher). After a crash the board reads this."""
    return on_launcher(state)


def on_launcher(state: PlantState) -> PlantState:
    """The state of an aircraft held at rest by the simple launcher: no motion, and the accelerometer
    reads the launcher's support against gravity in body axes, (g sin(pitch), -g sin(roll)
    cos(pitch), -g cos(roll) cos(pitch)): (0, 0, -g) when level. The plant is not stepped while
    held (catapult.Catapult.holds), so its own specific force (free fall above the ground) is replaced.
    The airspeed reads 0 there even in a set wind (--wind-speed-m-s; calm by default)."""
    sin_r, cos_r = math.sin(state.roll_rad), math.cos(state.roll_rad)
    sin_p, cos_p = math.sin(state.pitch_rad), math.cos(state.pitch_rad)
    g = STANDARD_GRAVITY_M_S2
    return replace(state, p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0, vn_m_s=0.0, ve_m_s=0.0, vd_m_s=0.0,
                   ias_m_s=0.0, tas_m_s=0.0, fx_m_s2=g * sin_p, fy_m_s2=-g * sin_r * cos_p,
                   fz_m_s2=-g * cos_r * cos_p)


def milli_g(specific_force_m_s2: float) -> int:
    return _clamp_int(specific_force_m_s2 / STANDARD_GRAVITY_M_S2 * MILLI_G_PER_G, INT16_MIN, INT16_MAX)


def hil_state_fields(state: PlantState) -> dict:
    """Keyword arguments of MAVLink.hil_state_quaternion_encode, without time_usec."""
    return {
        "attitude_quaternion": list(euler_to_quaternion(state.roll_rad, state.pitch_rad, state.yaw_rad)),
        "rollspeed": state.p_rad_s,
        "pitchspeed": state.q_rad_s,
        "yawspeed": state.r_rad_s,
        "lat": int(round(state.lat_deg * DEG_E7)),
        "lon": int(round(state.lon_deg * DEG_E7)),
        "alt": int(round(state.alt_msl_m * MM_PER_M)),
        "vx": _clamp_int(state.vn_m_s * CM_PER_M, INT16_MIN, INT16_MAX),
        "vy": _clamp_int(state.ve_m_s * CM_PER_M, INT16_MIN, INT16_MAX),
        "vz": _clamp_int(state.vd_m_s * CM_PER_M, INT16_MIN, INT16_MAX),
        "ind_airspeed": _clamp_int(state.ias_m_s * CM_PER_M, 0, UINT16_MAX),
        "true_airspeed": _clamp_int(state.tas_m_s * CM_PER_M, 0, UINT16_MAX),
        "xacc": milli_g(state.fx_m_s2),
        "yacc": milli_g(state.fy_m_s2),
        "zacc": milli_g(state.fz_m_s2),
    }
