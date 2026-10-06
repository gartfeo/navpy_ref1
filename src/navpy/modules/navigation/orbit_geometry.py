"""Navigation-feasibility orbit radius (R_nav_min).

The closest standoff from which the terminal NAV dive can still
reach the selected delivery reference geometrically. This is an approach
constraint, not a physical docking or safe-handover test. Two binding
constraints, whichever dominates:

* turn-radius roll-out: the loiter velocity is tangent, so the aircraft
  must reverse ~90 deg and point inward before it can descend cleanly —
  ``kappa * V^2 / (g * tan(phi_max))``.
* descent geometry (usually dominant): the airframe pitch limit must
  convert the orbit altitude into closing range, else the dive rails at
  ``min_pitch``, overflies, and trips the passed-target reset —
  ``h / tan(eta * |min_pitch|)``.

Orbiting at this radius keeps the camera zoom minimal (widest FOV for
situational awareness) while leaving the dive valid. Sizing the orbit for
camera DETECTION range instead (the prior behavior) parked the aircraft
far enough that recognition was unreachable even at max zoom.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_GRAVITY = 9.81

# Margins — documented engineering safety factors, not tuning knobs:
# kappa covers L1 not commanding full bank instantly plus the capture
# lead distance; eta reserves pitch authority for the PID correction and
# wind on top of the LOS feedforward (0.65 of a -40 deg limit leaves
# ~13 deg of reserve at a ~27 deg dive).
_DEFAULT_KAPPA = 2.0
_DEFAULT_ETA = 0.65


@dataclass(frozen=True)
class OrbitNavigationLimits:
    """Vehicle envelope needed to size the navigation-feasible orbit.

    ``airspeed_mps`` loiter true airspeed; ``roll_limit_deg`` the bank
    limit (ROLL_LIMIT_DEG); ``min_pitch_deg`` the nose-down pitch limit
    (PTCH_LIM_MIN_DEG, negative). Callers must supply finite, in-range
    values — see ``NavController._orbit_limits`` for the validation that
    falls back to camera-range sizing when the vehicle limits are
    unavailable.
    """

    airspeed_mps: float
    roll_limit_deg: float
    min_pitch_deg: float
    kappa: float = _DEFAULT_KAPPA
    eta: float = _DEFAULT_ETA


def r_nav_min(
        limits: OrbitNavigationLimits, alt_agl_m: float, floor_m: float = 0.0,
) -> float:
    """Minimum valid orbit standoff (m): the max of the turn-radius and
    descent-geometry constraints, never below ``floor_m`` (the
    autopilot-sane loiter minimum). Degenerate terms (non-positive tan,
    non-positive altitude) drop out rather than dominate.
    """
    terms = [float(floor_m)]

    # Turn-radius term — only for a physical bank limit in (0, 90) deg.
    v = float(limits.airspeed_mps)
    phi_deg = abs(float(limits.roll_limit_deg))
    if v > 0.0 and 0.0 < phi_deg < 90.0:
        terms.append(
            limits.kappa * v * v / (_GRAVITY * math.tan(math.radians(phi_deg)))
        )

    # Descent term — only for a usable dive angle in (0, 90) deg.
    gamma_deg = float(limits.eta) * abs(float(limits.min_pitch_deg))
    if alt_agl_m > 0.0 and 0.0 < gamma_deg < 90.0:
        terms.append(float(alt_agl_m) / math.tan(math.radians(gamma_deg)))

    return max(terms)
