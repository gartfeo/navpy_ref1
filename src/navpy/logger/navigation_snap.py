"""Exact compatibility re-exports for closest-approach diagnostics."""

from navpy.logger.navigation_snap_geometry import (
    calc_distance,
    calc_h_v_dist,
    closest_horizontal_on_segment,
    closest_on_segment,
    closest_point_components_on_segment,
)
from navpy.logger.navigation_snap_tracker import ClosestApproachTracker
from navpy.logger.navigation_snap_types import ClosestPointComponents, ClosestSnap


__all__ = [
    "ClosestApproachTracker",
    "ClosestPointComponents",
    "ClosestSnap",
    "calc_distance",
    "calc_h_v_dist",
    "closest_horizontal_on_segment",
    "closest_on_segment",
    "closest_point_components_on_segment",
]
