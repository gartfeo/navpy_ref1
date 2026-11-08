"""Limited tries and back-off for a part NavPy restarts fresh in place (docs/design/navpy_recovery.md, step 5).

Owner ruling 2026-11-06 (Q3): a part restarts fresh inside NavPy with limited tries and back-off, escalating to a
supervised restart of the whole process. Most outages are short, so the first restart is immediate; a part that
keeps failing is spaced out, then escalated. Owners pass an aircraft-time clock (owner ruling Q1:
core/flow/nav_recovery_clock.AircraftTimeClock), so the back-off and the healthy time scale with sim speed-up as
the NAV recovery timers do; ``runtime_time.monotonic`` is only the default.
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
        self._running_since: float | None = None   # when the instance now running started successfully

    @property
    def tries(self) -> int:
        """Restart tries used since the part was last healthy."""
        return self._tries

    @property
    def exhausted(self) -> bool:
        return self._tries >= len(self._backoff_s)

    def failed(self) -> None:
        """The running instance (or a restart try) failed now. Only time since a successful start counts as
        healthy: a try that is slow to fail never earns the tries back."""
        now = self._clock()
        if self._running_since is not None and now - self._running_since >= self._stable_s:
            self._tries = 0
        self._failed_at = now
        self._running_since = None

    def due(self) -> bool:
        """A try remains and its back-off since the last failure has passed."""
        if self.exhausted or self._failed_at is None:
            return False
        return self._clock() - self._failed_at >= self._backoff_s[self._tries]

    def attempt(self) -> int:
        """A restart try starts now; returns its number (1-based)."""
        self._tries += 1
        self._running_since = None
        return self._tries

    def running(self) -> None:
        """The try's new instance started successfully: from now its time counts as healthy."""
        self._running_since = self._clock()


__all__ = ["COMPONENT_RESTART_BACKOFF_S", "COMPONENT_STABLE_S", "RestartBudget"]
