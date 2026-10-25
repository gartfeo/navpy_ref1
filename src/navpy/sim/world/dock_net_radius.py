"""The simulated dock net's radius: the owner-set default and its override.

The contact sensor (``dock_contact_sensor.py``) docks strictly inside it.
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping

# Owner-set net radius (2026-10-13). Contact is strictly inside it, matching
# the eval's strict `< 0.5 m` SNAP gate.
SIM_DOCK_NET_RADIUS_M = 0.5
# Sim-only override that shrinks the net for a forced-miss check
# (missed_dock.md): 0 makes every pass a miss. Never wider than the default,
# which the eval's DOCKED check assumes.
SIM_DOCK_NET_RADIUS_ENV = "NAVPY_SIM_DOCK_NET_RADIUS_M"


def sim_dock_net_radius_m(env: Mapping[str, str] | None = None) -> float:
    """The net radius: the override when set, else the owner-set default."""
    raw = (os.environ if env is None else env).get(SIM_DOCK_NET_RADIUS_ENV)
    if raw is None or not raw.strip():
        return SIM_DOCK_NET_RADIUS_M
    radius_m = valid_radius_m(float(raw))
    if radius_m > SIM_DOCK_NET_RADIUS_M:
        raise ValueError(f"{SIM_DOCK_NET_RADIUS_ENV} may only shrink the net, got {raw!r}")
    return radius_m


def valid_radius_m(radius_m: float) -> float:
    if not math.isfinite(radius_m) or radius_m < 0.0:
        raise ValueError(f"dock net radius must be finite and >= 0, got {radius_m!r}")
    return radius_m


__all__ = [
    "SIM_DOCK_NET_RADIUS_ENV",
    "SIM_DOCK_NET_RADIUS_M",
    "sim_dock_net_radius_m",
    "valid_radius_m",
]
