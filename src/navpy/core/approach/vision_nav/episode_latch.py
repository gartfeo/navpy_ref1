"""Episode-level terminal latch of one final approach (physical_docking.md).

It lives outside `VisualPassState`, so `invalidate_commands()` and a
continuity-key change cannot erase it; only RESET or the next episode
(`reset_phase`) clears it. First latch wins: a contact after a NAV_FAIL is
ignored, and a NAV_FAIL or detection loss after a contact is ignored. The
caller holds the runtime lock for every call.
"""

from __future__ import annotations

from enum import Enum

from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity


class FinalApproachTerminal(Enum):
    DOCKED = "docked"
    MISSED = "missed"
    FAILED = "failed"


class FinalApproachEpisodeLatch:
    def __init__(self) -> None:
        self._identity: DockContactIdentity | None = None
        self._terminal: FinalApproachTerminal | None = None
        self._miss_reported = False

    @property
    def docked(self) -> bool:
        return self._terminal is FinalApproachTerminal.DOCKED

    @property
    def commands_ended(self) -> bool:
        """Docked or missed: nothing may be commanded until RESET.

        A NAV_FAIL latches FAILED only to win the race with a contact; the
        navigation loop then resets the phase as it always did.
        """
        return self._terminal in (
            FinalApproachTerminal.DOCKED,
            FinalApproachTerminal.MISSED,
        )

    def begin(self, identity: DockContactIdentity) -> None:
        """Expect contact only for this armed (vehicle, task, dock, episode)."""
        self.reset()
        self._identity = identity

    def latch_docked(self, event: DockContactEvent) -> bool:
        if self._terminal is not None or event.identity != self._identity:
            return False
        self._terminal = FinalApproachTerminal.DOCKED
        return True

    def latch_missed(self) -> bool:
        if self._terminal is not None:
            return False
        self._terminal = FinalApproachTerminal.MISSED
        return True

    def latch_failure(self) -> bool:
        """Admit a NAV_FAIL unless the episode already docked."""
        if self.docked:
            return False
        if self._terminal is None:
            self._terminal = FinalApproachTerminal.FAILED
        return True

    def consume_missed(self) -> bool:
        """True once after an aft-dock miss, for the NAV_FAIL report."""
        if self._terminal is not FinalApproachTerminal.MISSED or self._miss_reported:
            return False
        self._miss_reported = True
        return True

    def reset(self) -> None:
        self._identity = None
        self._terminal = None
        self._miss_reported = False


__all__ = ["FinalApproachEpisodeLatch", "FinalApproachTerminal"]
