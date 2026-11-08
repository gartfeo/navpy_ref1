"""Episode-level terminal latch of one guidance (physical_docking.md).

It lives outside `VisualPassState`, so `invalidate_commands()` and a
continuity-key change cannot erase it; only RESET or the next episode
(`reset_phase`) clears it. First latch wins: a contact after a NAV_FAIL is
ignored, and a NAV_FAIL or detection loss after a contact is ignored. The
caller holds the runtime lock for every call. An observer, if given, sees
each latch (outcome and the armed identity, None when none was armed) under
that lock: it must not block (the HITL terminal handoff).

A NAV_FAIL of an armed episode ends it as a failed dock (missed_dock.md): it
ends commands, and its reason is kept for the NAV exit snapshot, so the state
machine reports why it failed. Unarmed (the real build) it only wins the race
with a contact, and the navigation loop resets the phase as before.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from navpy.core.approach.vision_nav import nav_fail_reason as fail_reason


class GuidanceTerminal(Enum):
    DOCKED = "docked"
    MISSED = "missed"
    FAILED = "failed"


TerminalObserver = Callable[[GuidanceTerminal, Optional[DockContactIdentity]], None]

# The reason of an aft-dock miss, as its NAV_FAIL line names it.
MISSED_DOCK_REASON = fail_reason.MISSED_DOCK
# A NAV_FAIL whose caller names no reason.
UNNAMED_FAILURE_REASON = fail_reason.NAV_FAIL


@dataclass(frozen=True)
class GuidanceEpisodeTerminal:
    """The armed episode's first terminal, taken at NAV exit before reset."""

    identity: DockContactIdentity
    terminal: GuidanceTerminal
    reason: str | None


class GuidanceEpisodeLatch:
    def __init__(self, on_terminal: TerminalObserver | None = None) -> None:
        self._identity: DockContactIdentity | None = None
        self._terminal: GuidanceTerminal | None = None
        self._reason: str | None = None
        self._miss_reported = False
        self._on_terminal = on_terminal

    @property
    def docked(self) -> bool:
        return self._terminal is GuidanceTerminal.DOCKED

    @property
    def commands_ended(self) -> bool:
        """Docked, missed, or an armed episode failed: nothing may be
        commanded until RESET or a new episode.

        A frame admitted after an armed NAV_FAIL is suppressed, not flown.
        """
        if self._terminal is GuidanceTerminal.FAILED:
            return self.failure_ends_commands
        return self._terminal is not None

    @property
    def failure_ends_commands(self) -> bool:
        """A FAILED latch ends commands only for an armed (sim) episode."""
        return (
            self._terminal is GuidanceTerminal.FAILED
            and self._identity is not None
        )

    def terminal_snapshot(self) -> GuidanceEpisodeTerminal | None:
        """The armed episode's terminal; None when none was armed or latched."""
        if self._identity is None or self._terminal is None:
            return None
        return GuidanceEpisodeTerminal(
            self._identity,
            self._terminal,
            self._reason,
        )

    def begin(self, identity: DockContactIdentity) -> None:
        """Expect contact only for this armed (vehicle, task, dock, episode)."""
        self.reset()
        self._identity = identity

    def latch_docked(self, event: DockContactEvent) -> bool:
        if self._terminal is not None or event.identity != self._identity:
            return False
        self._latch(GuidanceTerminal.DOCKED)
        return True

    def latch_missed(self) -> bool:
        if self._terminal is not None:
            return False
        self._reason = MISSED_DOCK_REASON
        self._latch(GuidanceTerminal.MISSED)
        return True

    def latch_failure(self, reason: str | None = None) -> bool:
        """Admit a NAV_FAIL unless the episode already docked.

        The first named reason is kept: an unnamed command failure that
        latched first takes the reason of the NAV_FAIL line that reports it.
        """
        if self.docked:
            return False
        if self._reason is None:
            self._reason = reason
        if self._terminal is None:
            self._latch(GuidanceTerminal.FAILED)
        return True

    def _latch(self, terminal: GuidanceTerminal) -> None:
        self._terminal = terminal
        if self._on_terminal is not None:
            self._on_terminal(terminal, self._identity)

    def consume_missed(self) -> bool:
        """True once after an aft-dock miss, for the NAV_FAIL report."""
        if self._terminal is not GuidanceTerminal.MISSED or self._miss_reported:
            return False
        self._miss_reported = True
        return True

    def reset(self) -> None:
        self._identity = None
        self._terminal = None
        self._reason = None
        self._miss_reported = False


__all__ = [
    "GuidanceEpisodeLatch",
    "GuidanceEpisodeTerminal",
    "GuidanceTerminal",
    "MISSED_DOCK_REASON",
    "TerminalObserver",
    "UNNAMED_FAILURE_REASON",
]
