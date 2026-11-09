"""NavPy's claim on its own recovery LOITER, carried by its companion heartbeat (docs/design/navpy_recovery.md,
step 10, item 9; owner ruling 2026-11-09, "Yes, guard it").

While NAV recovery holds in a LOITER NavPy asked for and still owns, the companion heartbeat's ``custom_mode`` has
NAVPY_HOLD_CLAIM set. The autopilot's NavPy watchdog (scripts/lua/nav_watchdog.lua) then guards that LOITER as its
own hold if the heartbeat stops: RTL after AAS_NWD_RTL. A pilot's LOITER is never claimed. One claim per bus, as one
NavPy vehicle runs per bus (like its command fence); the GCS's own heartbeat never sets it.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt

#: The companion heartbeat's ``custom_mode`` bit: NavPy's recovery holds in its own LOITER (nav_watchdog.lua,
#: HOLD_CLAIM).
NAVPY_HOLD_CLAIM = 1


class CompanionHoldClaim:
    """Set by NAV recovery's step (any thread), read by the heartbeat sender."""

    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._claimed = False
        self._changed: Callable[[], object] = lambda: None

    def on_change(self, changed: Callable[[], object]) -> None:
        """``changed`` runs after each change: the heartbeat sender's wake, so the autopilot hears it at once."""
        self._changed = changed

    def set(self, claimed: bool) -> None:
        with self._lock:
            changed, self._claimed = self._claimed != bool(claimed), bool(claimed)
        if changed:
            self._changed()

    @property
    def custom_mode(self) -> int:
        with self._lock:
            return NAVPY_HOLD_CLAIM if self._claimed else 0


__all__ = ["NAVPY_HOLD_CLAIM", "CompanionHoldClaim"]
