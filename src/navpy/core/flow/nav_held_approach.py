"""A final approach held by NAV recovery (docs/design/navpy_recovery.md; owner ruling 2026-11-09, "New approach").

Recovery's hold mid-approach ends the in-flight final-approach episode without counting it as a failure: not
DOCK_FAILED, no one-shot finish. When the approach goes on after the hold (recovery resumed in AUTO), NavPy starts a
fresh one to the kept dock, with new source admission, command liveness and dock-contact episode; nothing latched by
the held episode carries over.

The NAV decision (``navigation_decision.py``) drives it: it ends the episode at its first decision while recovery
holds, and starts the fresh one at its first decision after. ``NavTransition`` binds the two actions at composition:
the episode's end (``NavExitCleanup.end_episode``) and NAV entry.
"""

from __future__ import annotations

from collections.abc import Callable


class HeldApproach:
    """The NAV thread's only: the decision and NAV entry run there."""

    def __init__(self) -> None:
        self._end: Callable[[], None] | None = None
        self._start: Callable[[], None] | None = None
        #: The held episode was ended, and no fresh one started yet.
        self.ended = False
        #: Recovery resumed and the fresh episode's NAV entry has not succeeded yet: the decision retries it.
        self.resume_pending = False

    def bind(self, end: Callable[[], None], start: Callable[[], None]) -> None:
        self._end, self._start = end, start

    def end_episode(self) -> None:
        """Once per hold: the in-flight episode ends, without failure."""
        if self.ended or self._end is None:
            return
        self._end()
        self.ended = True

    def start_fresh_episode(self) -> None:
        """NAV entry again, for the kept dock. Pending until it succeeds (``entered``): an entry that raises is
        retried at the next decision, as a NAV phase entry would be."""
        self.resume_pending = True
        if self._start is not None:
            self._start()

    def entered(self) -> None:
        """A NAV entry succeeded: a fresh episode, nothing held is left."""
        self.ended = False
        self.resume_pending = False

    def abandon(self) -> None:
        """The approach ends after the hold (the operator's mode stands): nothing to restart."""
        self.ended = False
        self.resume_pending = False


__all__ = ["HeldApproach"]
