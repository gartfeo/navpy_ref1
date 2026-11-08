"""How NAV left an armed dock episode, for the sim closest-approach record.

Display only (gcs_sim_closest_approach.md): read at NAV exit from the
episode's result latch and the POI's confirmation state; nothing here
reaches frames, the navigation law or the command path.
"""

from __future__ import annotations

from typing import Optional

from navpy.core.approach.vision_nav.episode_latch import (
    UNNAMED_FAILURE_REASON,
    GuidanceEpisodeResult,
    GuidanceResult,
)
from navpy.core.ports.dock_contact import DockContactIdentity, DockEpisodeEnd

# The ground station cancelled the confirmed task during the approach.
CANCELLED_REASON = "cancelled"


def dock_episode_end(
    identity: DockContactIdentity,
    docked: bool,
    result: Optional[GuidanceEpisodeResult],
    cancelled: bool,
) -> DockEpisodeEnd:
    """Docked wins; then the episode's own result; then a cancel."""
    if docked:
        return DockEpisodeEnd(docked=True)
    if result is not None and result.identity == identity:
        if result.outcome is GuidanceResult.MISSED:
            return DockEpisodeEnd(missed=True)
        if result.outcome is GuidanceResult.FAILED:
            return DockEpisodeEnd(
                failed=True,
                abort_reason=result.reason or UNNAMED_FAILURE_REASON,
            )
    if cancelled:
        return DockEpisodeEnd(abort_reason=CANCELLED_REASON)
    return DockEpisodeEnd()


__all__ = ["CANCELLED_REASON", "dock_episode_end"]
