"""Whether nav flies a final approach, for swarm task availability.

A UAV flying a final approach is BUSY for swarm auctions; every other
phase leaves it free for a peer task: searching, CONFIRM of an own POI,
the DDH return, ONHOLD, RESET and RECOVERY (docs/design/
swarm-task-assignment-ack.md, "Busy UAVs").
"""

from __future__ import annotations

from navpy.modules.nav.nav_state import NavState


def is_approaching(committed: NavState) -> bool:
    return committed is NavState.NAV


__all__ = ["is_approaching"]
