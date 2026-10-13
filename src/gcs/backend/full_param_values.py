"""Full-parameter value rules, results and errors (see ``full_params``)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from navpy.plugins.vehicle.ardupilot._mavftp.param_pck import (
    AP_TYPE_FLOAT, AP_TYPE_INT8, AP_TYPE_INT16, AP_TYPE_INT32,
)


# AP-packed type → MAVLink on-wire param type. AP codes (1..4) are NOT
# numerically equal to MAV_PARAM_TYPE_* (which start at 1 = MAV_PARAM_TYPE_UINT8).
# Per Codex pre-step finding 1: backend derives this from the cached snapshot
# rather than trusting the client.
def mav_param_type_for(ap_type: int) -> Optional[int]:
    from pymavlink.dialects.v20.ardupilotmega import (
        MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_REAL32,
    )
    return {
        AP_TYPE_INT8: MAV_PARAM_TYPE_INT8,
        AP_TYPE_INT16: MAV_PARAM_TYPE_INT16,
        AP_TYPE_INT32: MAV_PARAM_TYPE_INT32,
        AP_TYPE_FLOAT: MAV_PARAM_TYPE_REAL32,
    }.get(ap_type)


# AP integer storage ranges (signed). `param.pck` carries the storage type
# only, so this is the storage range — not logical @Range metadata.
_AP_INT_BOUNDS: dict[int, tuple[int, int]] = {
    AP_TYPE_INT8: (-128, 127),
    AP_TYPE_INT16: (-32768, 32767),
    AP_TYPE_INT32: (-(2 ** 31), 2 ** 31 - 1),
}


def coerce_to_stored(ap_type: int, value: object) -> float | int | None:
    """Coerce a value to what an AP param of this type would STORE.

    Integer types (int8/16/32) truncate toward zero and clamp to the signed
    storage range — mirroring the autopilot's `AP_Param::set_float` and the
    frontend `coerceValueForRecord`, so every full-param write path (cell
    editor, `.param` import, raw PUT) stages the same value. Float passes a
    finite value through. Non-finite / non-numeric returns ``None`` (rejected
    before send — the wire never sees it).

    Deliberately NOT `vehicle_mav._value_in_int_range`: that is the shared
    vehicle-layer guard, keyed by MAVLink wire types (incl. unsigned/64-bit)
    and validating only after integer conversion. This is the full-param
    boundary using AP storage semantics.
    """
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    bounds = _AP_INT_BOUNDS.get(ap_type)
    if bounds is None:
        return f  # float (ap_type 4) — finite passthrough
    lo, hi = bounds
    return min(hi, max(lo, math.trunc(f)))


# ---- DTOs --------------------------------------------------------------
@dataclass(frozen=True)
class ParamChange:
    name: str
    value: float | int


@dataclass
class CellResult:
    ok: bool
    value: float | int | None = None
    error: Optional[str] = None


@dataclass
class WriteBatchResult:
    snapshot_stale: bool
    results: dict[str, CellResult] = field(default_factory=dict)


@dataclass(frozen=True)
class ArmedToken:
    nonce: str
    expires_at_unix_s: float


# ---- Errors ------------------------------------------------------------
class FullParamError(Exception):
    """Backend full-param coordinator error."""


class ArmedWriteRejected(FullParamError):
    """Vehicle is armed and the request did not present a valid arm token."""


class UnknownParam(FullParamError):
    """Caller wrote a name that isn't in the cached snapshot."""


__all__ = [
    "ArmedToken", "ArmedWriteRejected", "CellResult", "FullParamError",
    "ParamChange", "UnknownParam", "WriteBatchResult", "coerce_to_stored",
    "mav_param_type_for",
]
