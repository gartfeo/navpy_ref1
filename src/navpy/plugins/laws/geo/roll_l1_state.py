"""Parameters and mutable memory for legacy Roll-L1 navigation."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RollL1Parameters:
    period: float
    damping: float
    xtrack_i_gain: float
    pitch_min: float
    pitch_max: float


@dataclass
class RollL1State:
    lat_accel_demand: float = 0.0
    l1_distance: float = 0.0
    nav_bearing: float = 0.0
    bearing_error: float = 0.0
    crosstrack_error: float = 0.0
    target_bearing_cd: float = 0.0
    last_nu: float = 0.0
    xtrack_integral: float = 0.0
    previous_integral_gain: float = 0.0
    last_update_us: float = 0.0
    data_is_stale: bool = True
    last_update_s: float = 0.0
