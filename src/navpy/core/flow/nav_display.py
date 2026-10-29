"""What nav is doing, for the operator display only (gcs_mission_phase.md).

The post-commit publish hook builds one ``NavDisplay`` from the committed
state; the swarm heartbeat carries its bits to the GCS. No navigation,
decision or command code reads it (Pure Vision Approach Constraint).

Heartbeat state byte: bit 0 BUSY (the swarm's), bits 1-3 the phase, bits
4-7 the last outcome: 0 none, 1 docked, 2 + r failed with reason index r.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from navpy.core.approach.vision_nav.nav_fail_reason import (
    MAX_REASON_INDEX,
    NAV_FAIL_REASON_INDEX,
    UNKNOWN_REASON_INDEX,
    reason_index,
)
from navpy.core.flow.nav_state import NavigationTaskState, NavState


class NavPhase(IntEnum):
    NONE = 0
    HOLD = 1
    SEARCH = 2
    TO_TASK = 3
    HOMING = 4
    CONFIRM = 5
    APPROACH = 6
    RECOVERING = 7


OUTCOME_NONE = 0
OUTCOME_DOCKED = 1
OUTCOME_FAILED_BASE = 2

_PHASE_SHIFT = 1
_OUTCOME_SHIFT = 4


@dataclass(frozen=True)
class NavOutcome:
    """A final approach's end: docked, or failed with its reason."""

    docked: bool
    reason: Optional[str] = None

    def code(self) -> int:
        if self.docked:
            return OUTCOME_DOCKED
        index = reason_index(self.reason)
        if index > MAX_REASON_INDEX:
            index = UNKNOWN_REASON_INDEX
        return OUTCOME_FAILED_BASE + index


@dataclass(frozen=True)
class NavDisplay:
    phase: NavPhase = NavPhase.NONE
    outcome: Optional[NavOutcome] = None

    def state_bits(self) -> int:
        """Bits 1-7 of the heartbeat state byte; bit 0 stays the swarm's."""
        outcome = OUTCOME_NONE if self.outcome is None else self.outcome.code()
        return (int(self.phase) << _PHASE_SHIFT) | (outcome << _OUTCOME_SHIFT)


class NavOutcomeLatch:
    """The last final-approach outcome; only the next one replaces it.

    The task reset never touches it, so it outlives DOCKED, the climb, a
    return to search, disarm and the next approach. Navigation thread only.
    """

    def __init__(self) -> None:
        self.last: Optional[NavOutcome] = None

    def record_docked(self) -> None:
        self.last = NavOutcome(docked=True)

    def record_failed(self, reason: str) -> None:
        self.last = NavOutcome(docked=False, reason=reason)


def nav_phase(
    committed: NavState,
    peer_navigation: bool,
    returning: bool,
) -> NavPhase:
    """Phase table, first match wins."""
    if committed is NavState.CONFIRM:
        return NavPhase.CONFIRM
    if committed is NavState.NAV:
        return NavPhase.APPROACH
    if committed is NavState.DETECT:
        if peer_navigation:
            return NavPhase.TO_TASK
        if returning:
            return NavPhase.HOMING
        return NavPhase.SEARCH
    if committed in (NavState.RESET, NavState.RECOVERY):
        return NavPhase.RECOVERING
    return NavPhase.HOLD


class NavDisplayBuilder:
    """Build the display from the committed state, on the navigation thread.

    ``returning`` is the mission module's return flag, given by its
    composition.
    """

    def __init__(
        self,
        outcome: NavOutcomeLatch,
        navigation_task: NavigationTaskState,
        returning: Callable[[], bool],
    ) -> None:
        self._outcome = outcome
        self._navigation_task = navigation_task
        self._returning = returning

    def build(self, committed: NavState) -> NavDisplay:
        return NavDisplay(
            nav_phase(
                committed,
                self._navigation_task.peer_navigation,
                bool(self._returning()),
            ),
            self._outcome.last,
        )


@dataclass(frozen=True)
class DecodedNavDisplay:
    """A received state byte's display bits, by name (the GCS reads these)."""

    phase: Optional[str]
    outcome: Optional[str]  # "docked", "failed" or None
    fail_reason: Optional[str]  # a table name, "unknown", or None


_REASON_NAMES = {index: name for name, index in NAV_FAIL_REASON_INDEX.items()}


def decode_state_bits(code: int) -> DecodedNavDisplay:
    phase_code = (code >> _PHASE_SHIFT) & 0b111
    outcome_code = (code >> _OUTCOME_SHIFT) & 0b1111
    phase = None if phase_code == NavPhase.NONE else NavPhase(phase_code).name
    if outcome_code == OUTCOME_NONE:
        return DecodedNavDisplay(phase, None, None)
    if outcome_code == OUTCOME_DOCKED:
        return DecodedNavDisplay(phase, "docked", None)
    reason = _REASON_NAMES.get(outcome_code - OUTCOME_FAILED_BASE, "unknown")
    return DecodedNavDisplay(phase, "failed", reason)


__all__ = [
    "DecodedNavDisplay",
    "NavDisplay",
    "NavDisplayBuilder",
    "NavOutcome",
    "NavOutcomeLatch",
    "NavPhase",
    "OUTCOME_DOCKED",
    "OUTCOME_FAILED_BASE",
    "OUTCOME_NONE",
    "decode_state_bits",
    "nav_phase",
]
