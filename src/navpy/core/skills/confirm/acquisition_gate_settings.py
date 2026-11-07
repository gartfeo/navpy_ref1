"""Lock and orbit tolerances a vision profile sets for its gates (HITL step 25).

Without the block nothing is gated, as before. A HITL profile sets::

    "final_approach_gates": {
        "lock_dwell_s": 2.0,
        "lock_max_image_error_deg": 1.0,
        "lock_max_frame_gap_s": 0.5,
        "orbit_radial_tolerance_m": 25.0,
        "orbit_dwell_s": 10.0
    }

The values are provisional until commissioning freezes them. A malformed block is refused when the
navigation is built, never silently replaced.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

FINAL_APPROACH_GATES_KEY = "final_approach_gates"


@dataclass(frozen=True)
class AcquisitionGateSettings:
    # Fresh visual lock: frames of one track, each with the POI within the image error of the
    # optical axis, no gap between new frames longer than the frame gap, for the dwell.
    lock_dwell_s: float
    lock_max_image_error_deg: float
    lock_max_frame_gap_s: float
    # Orbit established: the aircraft within the radial tolerance of the commanded circle for the dwell.
    orbit_radial_tolerance_m: float
    orbit_dwell_s: float


_FIELDS = tuple(AcquisitionGateSettings.__dataclass_fields__)


def acquisition_gate_settings(profile: Mapping[str, object]) -> AcquisitionGateSettings | None:
    """The profile's gate tolerances, or None when it sets none."""
    block = profile.get(FINAL_APPROACH_GATES_KEY) if isinstance(profile, Mapping) else None
    if block is None:
        return None
    if not isinstance(block, Mapping):
        raise ValueError(f"vision profile {FINAL_APPROACH_GATES_KEY} must be a mapping")
    unknown = set(block) - set(_FIELDS)
    missing = set(_FIELDS) - set(block)
    if unknown or missing:
        raise ValueError(
            f"{FINAL_APPROACH_GATES_KEY}: unknown {sorted(unknown)}, missing {sorted(missing)}"
        )
    values = {}
    for name in _FIELDS:
        value = block[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{FINAL_APPROACH_GATES_KEY}.{name} must be a positive number")
        values[name] = float(value)
    if values["lock_max_image_error_deg"] >= 90.0:
        raise ValueError(f"{FINAL_APPROACH_GATES_KEY}.lock_max_image_error_deg must be below 90")
    return AcquisitionGateSettings(**values)


__all__ = [
    "AcquisitionGateSettings",
    "FINAL_APPROACH_GATES_KEY",
    "acquisition_gate_settings",
]
