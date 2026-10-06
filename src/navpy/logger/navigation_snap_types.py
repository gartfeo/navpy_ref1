"""Data records for closest-approach navigation diagnostics."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.modules.common.models.location import Location


@dataclass
class ClosestSnap:
    """Snapshot of closest approach data."""

    dist: float = float("inf")
    h_dist: float = float("inf")
    v_dist: float = float("inf")
    h_min: float = float("inf")
    v_at_h_min: float = float("inf")
    component_lateral: float = float("inf")
    component_longitudinal: float = float("inf")
    component_vertical: float = float("inf")
    component_lateral_signed: float = float("inf")
    component_longitudinal_signed: float = float("inf")
    component_vertical_signed: float = float("inf")

    def clone(self) -> "ClosestSnap":
        return ClosestSnap(
            dist=self.dist,
            h_dist=self.h_dist,
            v_dist=self.v_dist,
            h_min=self.h_min,
            v_at_h_min=self.v_at_h_min,
            component_lateral=self.component_lateral,
            component_longitudinal=self.component_longitudinal,
            component_vertical=self.component_vertical,
            component_lateral_signed=self.component_lateral_signed,
            component_longitudinal_signed=self.component_longitudinal_signed,
            component_vertical_signed=self.component_vertical_signed,
        )

    def __str__(self) -> str:
        h_min_str = ""
        if self.h_min != float("inf") and abs(self.h_min - self.h_dist) > 0.5:
            h_min_str = f"; h_min={self.h_min:.1f}(v={self.v_at_h_min:.1f})"
        return (
            f"3d={self.dist:.1f}(h={self.h_dist:.1f}; v={self.v_dist:.1f})"
            f"{h_min_str}"
        )

    def status(self) -> str:
        if self.dist == float("inf"):
            return "N/A"
        h_min_str = ""
        if self.h_min != float("inf") and abs(self.h_min - self.h_dist) > 0.5:
            h_min_str = f"; {self.h_min:.1f}m [{self.v_at_h_min:.1f}]"
        return (
            f"{self.dist:.1f}m [{self.h_dist:.1f}; {self.v_dist:.1f}]"
            f"{h_min_str}"
        )

    def has_components(self) -> bool:
        return (
            math.isfinite(self.component_lateral)
            and math.isfinite(self.component_longitudinal)
            and math.isfinite(self.component_vertical)
        )


@dataclass(frozen=True)
class ClosestPointComponents:
    """Closest-point component decomposition for certification scoring."""

    c_star: Location
    h_dist: float
    v_dist: float
    slant: float
    ned_unit: np.ndarray
    lateral: float
    longitudinal: float
    vertical: float
    lateral_signed: float
    longitudinal_signed: float
    vertical_signed: float


__all__ = ["ClosestPointComponents", "ClosestSnap"]
