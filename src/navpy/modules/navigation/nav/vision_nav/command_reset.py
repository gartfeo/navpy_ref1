"""Authoritative reset of queued and retained terminal command state."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Protocol

from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    TerminalCommandHoldStore,
)
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    TerminalPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    TerminalDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.queued_frame import TerminalQueuedFrame


class TerminalResettableState(Protocol):
    def reset(self) -> None: ...


@dataclass(frozen=True)
class TerminalCommandResetPorts:
    lock: threading.RLock
    slot: NavigationCommandSlot
    mailbox: TerminalDiagnosticMailbox
    hold: TerminalCommandHoldStore
    law: TerminalResettableState
    visual_pass: TerminalResettableState
    status: TerminalResettableState
    liveness: TerminalResettableState
    postprocess_fence: TerminalPostprocessFence


class TerminalCommandReset:
    """Reset every state owner that can affect the next actuator call."""

    def __init__(self, ports: TerminalCommandResetPorts) -> None:
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
        if isinstance(pending, TerminalQueuedFrame):
            self._ports.mailbox.discard(pending.diagnostic_token)

    def _reset_control_locked(self) -> None:
        self._ports.hold.clear()
        self._ports.law.reset()
        self._ports.visual_pass.reset()
        self._ports.status.reset()


__all__ = [
    "TerminalCommandReset",
    "TerminalCommandResetPorts",
    "TerminalResettableState",
]
