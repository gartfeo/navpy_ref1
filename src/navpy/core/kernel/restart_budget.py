"""Limited tries and back-off for a part NavPy restarts fresh in place (docs/design/navpy_recovery.md, step 5).

Owner ruling 2026-11-06 (Q3): a part restarts fresh inside NavPy with limited tries and back-off, escalating to a
supervised restart of the whole process. Most outages are short, so the first restart is immediate; a part that
keeps failing is spaced out, then escalated. Times are ``runtime_time.monotonic`` (wall time, or the lockstep
clock in a lockstep run): a restart is NavPy's own work, not a flight timer.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

from navpy.core.kernel import runtime_time as rt

#: Wait before each restart try, from the failure that asks for it: the first is immediate (a short outage recovers
#: promptly), later ones back off so a part that fails at once again does not spin. Their count is the tries.
COMPONENT_RESTART_BACKOFF_S: tuple[float, ...] = (0.0, 1.0, 4.0)
#: An instance that ran this long without failing is healthy again: its next failure starts with every try. The
#: same 30 s NAV recovery waits in the hold (NAV_HOLD_RECOVERY_S): a part failing less often than that recovers.
COMPONENT_STABLE_S = 30.0


class RestartBudget:
    """Counts restart tries of one part. Not thread-safe: its owner serializes the calls."""

    def __init__(
        self,
        backoff_s: Sequence[float] = COMPONENT_RESTART_BACKOFF_S,
        stable_s: float = COMPONENT_STABLE_S,
        clock: Callable[[], float] = rt.monotonic,
    ) -> None:
        if not backoff_s:
            raise ValueError("a restart budget needs at least one try")
        self._backoff_s = tuple(float(wait) for wait in backoff_s)
        self._stable_s = float(stable_s)
        self._clock = clock
        self._tries = 0
        self._failed_at: float | None = None
        self._started_at: float | None = None

    @property
    def tries(self) -> int:
        """Restart tries used since the part was last healthy."""
        return self._tries

    @property
    def exhausted(self) -> bool:
        return self._tries >= len(self._backoff_s)

    def failed(self) -> None:
        """The running instance (or a restart try) failed now."""
        now = self._clock()
        if self._started_at is not None and now - self._started_at >= self._stable_s:
            self._tries = 0
        self._failed_at = now
        self._started_at = None

    def due(self) -> bool:
        """A try remains and its back-off since the last failure has passed."""
        if self.exhausted or self._failed_at is None:
            return False
        return self._clock() - self._failed_at >= self._backoff_s[self._tries]

    def attempt(self) -> int:
        """A restart try starts now; returns its number (1-based)."""
        self._tries += 1
        self._started_at = self._clock()
        return self._tries


__all__ = ["COMPONENT_RESTART_BACKOFF_S", "COMPONENT_STABLE_S", "RestartBudget"]
