"""A real ILogger that records its warnings and recovery reports, for tests of recovery reporting."""
from __future__ import annotations

from navpy.core.kernel.logging.logger_api import ConsoleLogger


class RecordingJournal:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def record(self, source: str, message: str) -> None:
        self.records.append((source, message))


class RecordingLogger(ConsoleLogger):
    """``ILogger.recovery`` as NavPy runs it: journaled, then a warning with its status text."""

    def __init__(self) -> None:
        super().__init__()
        self.journal = RecordingJournal()
        self.recovery_journal = self.journal
        self.warnings: list[dict[str, object]] = []

    def warning(self, msg, key="", status=None, dest=None) -> None:
        self.warnings.append({"msg": msg, "key": key, "status": status, "dest": dest})

    @property
    def statuses(self) -> list[object]:
        return [warning["status"] for warning in self.warnings]
