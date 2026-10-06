"""Out-of-band ownership of rich detector objects."""

from __future__ import annotations

import threading
from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    TerminalCommandTiming,
)
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class TerminalDiagnosticEntry:
    target: DetectedObject
    timing: TerminalCommandTiming


class TerminalDiagnosticMailbox:
    """Keep rich targets out of queued and command-fence payloads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._next_token = 1
        self._entries: dict[int, TerminalDiagnosticEntry] = {}

    def put(
        self,
        target: DetectedObject,
        timing: TerminalCommandTiming,
    ) -> int:
        with self._lock:
            token = self._next_token
            self._next_token += 1
            self._entries[token] = TerminalDiagnosticEntry(target, timing)
            return token

    def pop(self, token: int) -> TerminalDiagnosticEntry | None:
        with self._lock:
            return self._entries.pop(token, None)

    def timing(self, token: int) -> TerminalCommandTiming | None:
        with self._lock:
            entry = self._entries.get(token)
            return None if entry is None else entry.timing

    def discard(self, token: int) -> None:
        self.pop(token)

    @property
    def size(self) -> int:
        with self._lock:
            return len(self._entries)


__all__ = ["TerminalDiagnosticEntry", "TerminalDiagnosticMailbox"]
