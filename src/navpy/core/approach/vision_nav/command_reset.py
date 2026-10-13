"""Authoritative reset of queued and retained final-approach command state."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Protocol

from navpy.core.command.navigation_command_slot import NavigationCommandSlot
from navpy.core.approach.vision_nav.command_hold import (
    FinalApproachCommandHoldStore,
)
from navpy.core.approach.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.core.approach.vision_nav.diagnostic_mailbox import (
    FinalApproachDiagnosticMailbox,
)
from navpy.core.approach.vision_nav.pass_evidence import FinalApproachPassEvidence
from navpy.core.approach.vision_nav.queued_frame import FinalApproachQueuedFrame


class FinalApproachResettableState(Protocol):
    def reset(self) -> None: ...


@dataclass(frozen=True)
class FinalApproachCommandResetPorts:
    lock: threading.RLock
    slot: NavigationCommandSlot
    mailbox: FinalApproachDiagnosticMailbox
    hold: FinalApproachCommandHoldStore
    law: FinalApproachResettableState
    visual_pass: FinalApproachResettableState
    status: FinalApproachResettableState
    liveness: FinalApproachResettableState
    postprocess_fence: FinalApproachPostprocessFence
    pass_evidence: FinalApproachPassEvidence


class FinalApproachCommandReset:
    """Reset every state owner that can affect the next actuator call."""

    def __init__(self, ports: FinalApproachCommandResetPorts) -> None:
        self._ports = ports

    def invalidate_commands(self) -> None:
        with self._ports.lock:
            self._retire_and_flush_locked()

    def reset_phase(self) -> None:
        with self._ports.lock:
            self._retire_and_flush_locked()
            self._ports.liveness.reset()

    def retire_after_pass(self) -> None:
        """Stop all actuation after a pass; keep the terminal pass latch."""
        with self._ports.lock:
            self._invalidate_locked()
            self._ports.hold.clear()

    def _retire_and_flush_locked(self) -> None:
        self._invalidate_locked()
        staged = self._ports.pass_evidence.take_staged()
        self._ports.postprocess_fence.drain()
        try:
            # Earlier command evidence has drained; a pass row the worker
            # never took is written now, before the phase's SNAP. A failure
            # propagates like a failed drain, never silently.
            if staged is not None:
                self._ports.pass_evidence.record_now(staged)
        finally:
            self._reset_control_locked()

    def _invalidate_locked(self) -> None:
        pending = self._ports.slot.invalidate()
        if isinstance(pending, FinalApproachQueuedFrame):
            self._ports.mailbox.discard(pending.diagnostic_token)

    def _reset_control_locked(self) -> None:
        self._ports.hold.clear()
        self._ports.law.reset()
        self._ports.visual_pass.reset()
        self._ports.status.reset()


__all__ = [
    "FinalApproachCommandReset",
    "FinalApproachCommandResetPorts",
    "FinalApproachResettableState",
]
