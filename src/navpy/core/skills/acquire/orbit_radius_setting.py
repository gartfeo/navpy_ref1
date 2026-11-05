"""The final-approach ORBIT floor a vision profile may set explicitly (HITL step 23).

Without the setting the floor is ``MIN_APPROACH_STANDOFF_M`` (500 m), unchanged. A HITL profile (its name holds
"hitl"; ``tests/modules/navigation/test_approach_geometry.py`` keeps it so) may replace it::

    "final_approach_orbit": {"radius_m": 170.0}

The setting replaces only the floor: ``r_nav_min``'s turn and descent terms still apply on top, so the planner
never flies an orbit its own envelope check calls infeasible. The HITL tools' pre-series orbit check refuses a
series whose setting fails the HITL geometry; navpy never sees its inputs. A malformed setting is
refused when the planner is built, never silently replaced by the default.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from navpy.core.skills.acquire.peer_orbit_plan import MIN_APPROACH_STANDOFF_M

FINAL_APPROACH_ORBIT_KEY = "final_approach_orbit"
RADIUS_KEY = "radius_m"


def final_approach_orbit_setting(profile: Mapping[str, object]) -> float | None:
    """The profile's explicit orbit floor in metres, or None when it sets none."""
    block = profile.get(FINAL_APPROACH_ORBIT_KEY) if isinstance(profile, Mapping) else None
    if block is None:
        return None
    if not isinstance(block, Mapping):
        raise ValueError(f"vision profile {FINAL_APPROACH_ORBIT_KEY} must be a mapping")
    unknown = set(block) - {RADIUS_KEY}
    if unknown:
        raise ValueError(f"unknown {FINAL_APPROACH_ORBIT_KEY} fields: {sorted(unknown)}")
    radius = block.get(RADIUS_KEY)
    if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) \
            or radius <= 0.0:
        raise ValueError(f"{FINAL_APPROACH_ORBIT_KEY}.{RADIUS_KEY} must be a positive number of metres")
    return float(radius)


def final_approach_orbit_floor(profile: Mapping[str, object]) -> float:
    """The floor the final-approach orbit is sized from: the explicit setting, else the product default."""
    setting = final_approach_orbit_setting(profile)
    return MIN_APPROACH_STANDOFF_M if setting is None else setting


__all__ = [
    "FINAL_APPROACH_ORBIT_KEY",
    "RADIUS_KEY",
    "final_approach_orbit_floor",
    "final_approach_orbit_setting",
]
