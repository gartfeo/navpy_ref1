"""State transition for caller-owned closest-approach snapshots."""

from __future__ import annotations

from typing import Optional

from navpy.logger.navigation_snap_geometry import (
    closest_horizontal_on_segment,
    closest_point_components_on_segment,
    is_valid_location,
)
from navpy.logger.navigation_snap_types import ClosestSnap
from navpy.modules.common.models.location import Location


class ClosestApproachTracker:
    """Update caller-owned state without retaining a rich logging host."""

    def update(
        self,
        snap: ClosestSnap,
        previous: Optional[Location],
        current: Optional[Location],
        target: Optional[Location],
    ) -> Optional[Location]:
        if not is_valid_location(current) or not is_valid_location(target):
            return previous

        segment_start = previous if is_valid_location(previous) else current
        components = closest_point_components_on_segment(
            segment_start,
            current,
            target,
        )
        if components.slant < snap.dist:
            snap.dist = components.slant
            snap.h_dist = components.h_dist
            snap.v_dist = components.v_dist
            snap.component_lateral = components.lateral
            snap.component_longitudinal = components.longitudinal
            snap.component_vertical = components.vertical
            snap.component_lateral_signed = components.lateral_signed
            snap.component_longitudinal_signed = components.longitudinal_signed
            snap.component_vertical_signed = components.vertical_signed

        h_min, v_at_h_min = closest_horizontal_on_segment(
            segment_start,
            current,
            target,
        )
        if h_min < snap.h_min:
            snap.h_min = h_min
            snap.v_at_h_min = v_at_h_min
        return current


__all__ = ["ClosestApproachTracker"]
