"""NavPy's claim on its own recovery LOITER, carried by its companion heartbeat (docs/design/navpy_recovery.md,
step 10, item 9; owner ruling 2026-11-09, "Yes, guard it"; Codex round 2).

From the moment NAV recovery requests its hold's LOITER, and while it owns the LOITER shown, the companion
heartbeat's ``custom_mode`` carries the claim (core/flow/nav_recovery_ports.HoldClaim, read here by its shape):

    bit 0        NAVPY_HOLD_CLAIM
    bits 1-7     the LOITER request's token (mod 128)
    bits 8-23    the incident's hold budget left, in tenths of a second, rounded down

The autopilot's NavPy watchdog (scripts/lua/nav_watchdog.lua) takes a token once, at the switch into LOITER, and then
RTLs at the claim's receipt plus that budget unless the mode leaves LOITER first. One claim per bus, as one NavPy
vehicle runs per bus (like its command fence); the GCS's own heartbeat never sets it.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Protocol

from navpy.core.kernel import runtime_time as rt

#: ``custom_mode`` bit 0: NavPy's recovery holds in its own LOITER (nav_watchdog.lua, HOLD_CLAIM).
NAVPY_HOLD_CLAIM = 1
TOKEN_SHIFT, TOKEN_MASK = 1, 0x7F
BUDGET_SHIFT, BUDGET_MASK = 8, 0xFFFF
#: The budget's unit on the wire: tenths of a second.
BUDGET_UNIT_S = 0.1


class HoldClaim(Protocol):
    """NAV recovery's claim (core/flow/nav_recovery_ports.HoldClaim); plugins do not import core/flow."""

    @property
    def token(self) -> int: ...

    @property
    def budget_s(self) -> float: ...


def encode_hold_claim(claim: HoldClaim | None) -> int:
    """The heartbeat's ``custom_mode`` for ``claim`` (0: none). The budget is rounded down, never later than NavPy."""
    if claim is None:
        return 0
    budget = min(max(math.floor(claim.budget_s / BUDGET_UNIT_S + 1e-9), 0), BUDGET_MASK)
    return NAVPY_HOLD_CLAIM | (claim.token & TOKEN_MASK) << TOKEN_SHIFT | budget << BUDGET_SHIFT


def _request(custom_mode: int) -> int:
    """The claim bit and token: which request is claimed, if any."""
    return custom_mode & ((TOKEN_MASK << TOKEN_SHIFT) | NAVPY_HOLD_CLAIM)


class CompanionHoldClaim:
    """Set by NAV recovery's step (any thread), read by the heartbeat sender."""

    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._custom_mode = 0
        self._changed: Callable[[], object] = lambda: None

    def on_change(self, changed: Callable[[], object]) -> None:
        """``changed`` runs when a claim starts, ends or names another request: the heartbeat sender's wake, so the
        autopilot hears it at once. The budget alone falls every step and goes out at the heartbeat's own rate."""
        self._changed = changed

    def set(self, claim: HoldClaim | None) -> None:
        custom_mode = encode_hold_claim(claim)
        with self._lock:
            changed = _request(self._custom_mode) != _request(custom_mode)
            self._custom_mode = custom_mode
        if changed:
            self._changed()

    @property
    def custom_mode(self) -> int:
        with self._lock:
            return self._custom_mode


__all__ = ["BUDGET_UNIT_S", "NAVPY_HOLD_CLAIM", "CompanionHoldClaim", "encode_hold_claim"]
