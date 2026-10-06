"""Authoritative reset of queued and retained final-approach command state."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Protocol

from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    FinalApproachCommandHoldStore,
)
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    FinalApproachDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.queued_frame import FinalApproachQueuedFrame


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


class FinalApproachCommandReset:
    """Reset every state owner that can affect the next actuator call."""

    def __init__(self, ports: FinalApproachCommandResetPorts) -> None:
        self._ports = ports

    def invalidate_commands(self) -> None:
        with self._ports.lock:
            self._invalidate_locked()
            self._ports.postprocess_fence.drain()
            self._reset_control_locked()

    def reset_phase(self) -> None:
        with self._ports.lock:
            self._invalidate_locked()
            self._ports.postprocess_fence.drain()
            self._reset_control_locked()
            self._ports.liveness.reset()

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
