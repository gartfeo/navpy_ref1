"""How NAV left an armed dock episode, for the sim closest-approach record.

Display only (gcs_sim_closest_approach.md): read at NAV exit from the
episode's terminal latch and the POI's confirmation state; nothing here
reaches frames, the navigation law or the command path.
"""

from __future__ import annotations

from typing import Optional

from navpy.core.approach.vision_nav.episode_latch import (
    UNNAMED_FAILURE_REASON,
    FinalApproachEpisodeTerminal,
    FinalApproachTerminal,
)
from navpy.core.ports.dock_contact import DockContactIdentity, DockEpisodeEnd

# The ground station cancelled the confirmed task during the approach.
CANCELLED_REASON = "cancelled"


def dock_episode_end(
    identity: DockContactIdentity,
    docked: bool,
    terminal: Optional[FinalApproachEpisodeTerminal],
    cancelled: bool,
) -> DockEpisodeEnd:
    """Docked wins; then the episode's own terminal; then a cancel."""
    if docked:
        return DockEpisodeEnd(docked=True)
    if terminal is not None and terminal.identity == identity:
        if terminal.terminal is FinalApproachTerminal.MISSED:
            return DockEpisodeEnd(missed=True)
        if terminal.terminal is FinalApproachTerminal.FAILED:
            return DockEpisodeEnd(
                failed=True,
                abort_reason=terminal.reason or UNNAMED_FAILURE_REASON,
            )
    if cancelled:
        return DockEpisodeEnd(abort_reason=CANCELLED_REASON)
    return DockEpisodeEnd()


__all__ = ["CANCELLED_REASON", "dock_episode_end"]
