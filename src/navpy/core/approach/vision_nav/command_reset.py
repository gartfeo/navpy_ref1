"""Authoritative reset of queued and retained guidance command state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from navpy.core.command.navigation_command_slot import NavigationCommandSlot
from navpy.core.approach.vision_nav.command_hold import (
    GuidanceCommandHoldStore,
)
from navpy.core.approach.vision_nav.command_postprocess import (
    GuidancePostprocessFence,
)
from navpy.core.approach.vision_nav.diagnostic_mailbox import (
    GuidanceDiagnosticMailbox,
)
from navpy.core.approach.vision_nav.docked_evidence import (
    GuidanceDockedEvidence,
)
from navpy.core.approach.vision_nav.queued_frame import GuidanceQueuedFrame
from navpy.core.kernel import runtime_time as rt


class GuidanceResettableState(Protocol):
    def reset(self) -> None: ...


@dataclass(frozen=True)
class GuidanceCommandResetPorts:
    lock: rt.RLock
    slot: NavigationCommandSlot
    mailbox: GuidanceDiagnosticMailbox
    hold: GuidanceCommandHoldStore
    law: GuidanceResettableState
    visual_pass: GuidanceResettableState
    status: GuidanceResettableState
    liveness: GuidanceResettableState
    postprocess_fence: GuidancePostprocessFence
    docked_evidence: GuidanceDockedEvidence
    episode: GuidanceResettableState


class GuidanceCommandReset:
    """Reset every state owner that can affect the next actuator call."""

    def __init__(self, ports: GuidanceCommandResetPorts) -> None:
        self._ports = ports

    def invalidate_commands(self) -> None:
        # The episode latch survives: a source reset or pause must not erase
        # a docking (physical_docking.md, "the latch belongs to the episode").
        with self._ports.lock:
            self._retire_and_flush_locked()

    def reset_phase(self) -> None:
        """RESET or the next episode: the only reset of the episode latch."""
        with self._ports.lock:
            # A failed drain keeps the episode (its result is the NAV exit's
            # evidence); the exit retries next cycle and resets it then.
            self._retire_and_flush_locked()
            self._ports.liveness.reset()
            self._ports.episode.reset()

    def retire_after_result(self) -> None:
        """Stop all actuation once the episode ended; keep the episode latch."""
        with self._ports.lock:
            self._invalidate_locked()
            self._ports.hold.clear()

    def _retire_and_flush_locked(self) -> None:
        self._invalidate_locked()
        staged = self._ports.docked_evidence.take_staged()
        try:
            self._ports.postprocess_fence.drain()
            # Earlier command evidence has drained; a DOCKED row the worker
            # never took is written now, before the phase's SNAP. A failure
            # propagates like a failed drain, never silently, and control
            # state is reset either way.
            if staged is not None:
                self._ports.docked_evidence.record_now(staged)
        finally:
            self._reset_control_locked()

    def _invalidate_locked(self) -> None:
        pending = self._ports.slot.invalidate()
        if isinstance(pending, GuidanceQueuedFrame):
            self._ports.mailbox.discard(pending.diagnostic_token)

    def _reset_control_locked(self) -> None:
        self._ports.hold.clear()
        self._ports.law.reset()
        self._ports.visual_pass.reset()
        self._ports.status.reset()


__all__ = [
    "GuidanceCommandReset",
    "GuidanceCommandResetPorts",
    "GuidanceResettableState",
]
