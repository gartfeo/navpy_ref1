"""TRUTH-FED closing speed: a DIAGNOSTIC CEILING, never a shipping law.

Every number this module reads is barred from the command path by AGENTS.md --
ground velocity, and the wind vector the bench itself commanded. It exists to
answer the one question no legal estimate can answer first: HOW MUCH IS THERE
TO WIN. If handing the law a perfect ground-frame closing speed does not move
the miss, then no estimate of that quantity will either, and the effort belongs
somewhere else. Measuring the ceiling before building the estimator is the
cheap order to do this in.

WHY THERE IS NO COSINE HERE, which is the whole reason this form is safe.

A bank accelerates perpendicular to the NOSE, so only the fraction cos(sigma)
of it reaches the line of sight, sigma being the angle between the two. True PN
asks for `N * Vc_ground * lambda_dot` perpendicular to the LOS. Writing S for
whatever speed the law multiplies by, the law delivers

    N * S * cos(sigma) * lambda_dot        perpendicular to the LOS

and `V_air_true * cos(sigma)` IS the closing speed against the AIR MASS. So

    S = V_air_true * Vc_ground / Vc_air    =>    delivered = N * Vc_ground * ld

The cosine CANCELS rather than being divided out. Writing `/ cos(sigma)`
instead carries a singularity at 90 degrees and a SIGN REVERSAL beyond it,
which turns the certified 118-degree rear acquisition in
`tests/guards/test_vision_nav_point_mass_certification.py:174` into a
fly-away. This form has neither, because Vc_air already contains the cosine.

Applied to the frame's AIRSPEED rather than to the law, because
`frame.air_speed_mps` is read at exactly one point in the command path
(`law.py:164`). Substituting the value therefore moves the gain and nothing
else: no law variant, no second source tree, no new frame field, and no breach
of the frozen inventory that keeps range and ground velocity out of
`FinalApproachVisionFrame`.

Selected PER AIRCRAFT by a cell field, so treated and untreated fly the same
code inside one launch. This bench has measured 5.5x batch-to-batch spread on
identical code, which is larger than the effect being measured, so a
one-arm-per-launch version of this experiment could not be read.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

# `vcg` swaps ONLY the wind factor, keeping the airspeed channel the law
# already reads, so it isolates Vc_ground/Vc_air. `tpn` also swaps that channel
# for true air speed, which is exact True PN. Running both separates the two
# error factors in one launch instead of confounding them.
MODES = ("none", "vcg", "tpn")

# Below this the aircraft is barely closing and the ratio Vc_ground/Vc_air is
# meaningless -- a near-zero denominator would hand the law an arbitrary gain.
# Falls back to the untreated airspeed and is COUNTED, so a run that spent its
# scoring interval in the fallback cannot be read as a treated one.
MIN_CLOSING_MPS = 2.0
MAX_SPEED_MPS = 400.0


@dataclass(frozen=True)
class OracleGain:
    """The speed handed to the law, and whether the treatment actually ran."""

    speed_mps: float
    applied: bool
    vc_ground_mps: float
    vc_air_mps: float


def substitute_speed(
    mode: str,
    *,
    channel_airspeed_mps: float,
    ground_speed_mps: float | None,
    course_deg: float | None,
    wind_speed_mps: float,
    wind_dir_deg: float,
    offset_ned_m: Sequence[float],
) -> OracleGain:
    """The speed the law should multiply by to fly True PN against the ground.

    Horizontal components only: the lateral command is a horizontal-plane law,
    and the vertical channel is a separate law this treatment does not touch.
    """
    untreated = OracleGain(channel_airspeed_mps, False, 0.0, 0.0)
    if mode not in MODES:
        raise ValueError(f"oracle mode must be one of {MODES}, got {mode!r}")
    if mode == "none" or ground_speed_mps is None or course_deg is None:
        return untreated

    north, east = float(offset_ned_m[0]), float(offset_ned_m[1])
    horizontal_m = math.hypot(north, east)
    if horizontal_m <= 0.0:
        return untreated
    # Unit vector along the LOS pointing AT the POI, so a closing speed
    # comes out positive.
    los_n, los_e = north / horizontal_m, east / horizontal_m

    course_rad = math.radians(course_deg)
    ground_n = ground_speed_mps * math.cos(course_rad)
    ground_e = ground_speed_mps * math.sin(course_rad)
    # SIM_WIND_DIR is the direction the wind comes FROM -- verified against the
    # measured crab angle rather than assumed: at wind_dir 6 deg with the nose
    # at 269, the track sat 16 deg left, which is where an 8 m/s wind blowing
    # towards 186 puts it.
    blowing_rad = math.radians(wind_dir_deg + 180.0)
    air_n = ground_n - wind_speed_mps * math.cos(blowing_rad)
    air_e = ground_e - wind_speed_mps * math.sin(blowing_rad)

    vc_ground = ground_n * los_n + ground_e * los_e
    vc_air = air_n * los_n + air_e * los_e
    if vc_ground < MIN_CLOSING_MPS or vc_air < MIN_CLOSING_MPS:
        return OracleGain(channel_airspeed_mps, False, vc_ground, vc_air)

    base = channel_airspeed_mps if mode == "vcg" else math.hypot(air_n, air_e)
    speed = base * vc_ground / vc_air
    if not math.isfinite(speed) or not 0.0 < speed <= MAX_SPEED_MPS:
        return OracleGain(channel_airspeed_mps, False, vc_ground, vc_air)
    return OracleGain(speed, True, vc_ground, vc_air)


__all__ = ["MODES", "OracleGain", "substitute_speed"]
