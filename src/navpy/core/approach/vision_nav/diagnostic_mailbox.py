"""Out-of-band ownership of rich detector objects."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.approach.vision_nav.command_freshness import (
    GuidanceCommandTiming,
)
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.detect_data import DetectedObject


@dataclass(frozen=True)
class GuidanceDiagnosticEntry:
    poi: DetectedObject
    timing: GuidanceCommandTiming


class GuidanceDiagnosticMailbox:
    """Keep rich POIs out of queued and command-fence payloads."""

    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._next_token = 1
        self._entries: dict[int, GuidanceDiagnosticEntry] = {}

    def put(
        self,
        poi: DetectedObject,
        timing: GuidanceCommandTiming,
    ) -> int:
        with self._lock:
            token = self._next_token
            self._next_token += 1
            self._entries[token] = GuidanceDiagnosticEntry(poi, timing)
            return token

    def pop(self, token: int) -> GuidanceDiagnosticEntry | None:
        with self._lock:
            return self._entries.pop(token, None)

    def timing(self, token: int) -> GuidanceCommandTiming | None:
        with self._lock:
            entry = self._entries.get(token)
            return None if entry is None else entry.timing

    def discard(self, token: int) -> None:
        self.pop(token)

    @property
    def size(self) -> int:
        with self._lock:
            return len(self._entries)


__all__ = ["GuidanceDiagnosticEntry", "GuidanceDiagnosticMailbox"]
