"""Guidance held by NAV recovery (docs/design/navpy_recovery.md; owner ruling 2026-11-09, "New approach").

Recovery's hold mid-approach ends the in-flight guidance episode without counting it as a failure: not
DOCK_FAILED, no one-shot finish. When the approach goes on after the hold (recovery resumed in AUTO), NavPy starts a
fresh one to the kept dock, with new source admission, command liveness and dock-contact episode; nothing latched by
the held episode carries over.

The NAV decision (``navigation_decision.py``) drives it: it ends the episode at its first decision while recovery
holds, and starts the fresh one at its first decision after. ``NavTransition`` binds the two actions at composition:
the episode's end (``NavExitCleanup.end_episode``) and NAV entry.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel.exception_groups import ExceptionGroup


class HeldApproach:
    """The NAV thread's only: the decision and NAV entry run there."""

    def __init__(self) -> None:
        self._end: Callable[[], None] | None = None
        self._start: Callable[[], None] | None = None
        #: The held episode was ended; until a NAV entry succeeds or NAV exits, the approach may not go on except by
        #: recovery's resume grant (``RecoveryHold.resumed``).
        self.ended = False

    def bind(self, end: Callable[[], None], start: Callable[[], None]) -> None:
        self._end, self._start = end, start

    def end_episode(self) -> None:
        """Once per hold: the in-flight episode ends, without failure."""
        if self.ended or self._end is None:
            return
        self._end()
        self.ended = True

    def start_fresh_episode(self) -> None:
        """NAV entry again, for the kept dock. If it raises, what it completed is rolled back first (the partial
        episode ends as held, as the held one did), so a retry under the same grant starts clean."""
        if self._start is None:
            return
        try:
            self._start()
        except BaseException as failure:
            try:
                if self._end is not None:
                    self._end()
            except Exception as rollback:  # noqa: BLE001 - both are reported
                raise ExceptionGroup("fresh guidance entry and its rollback failed", [failure, rollback]) \
                    from None
            raise

    def clear(self) -> None:
        """A NAV entry succeeded, or NAV exited: nothing held is left."""
        self.ended = False


__all__ = ["HeldApproach"]
