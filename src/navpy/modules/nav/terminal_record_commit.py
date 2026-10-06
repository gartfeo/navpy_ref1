"""One-time terminal confirmation record commit."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.modules.nav.terminal_source_contracts import NavSourceBatch
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication


class TerminalRecordDeferral(Protocol):
    def defer_or_fail(self, target: DetectedObject) -> None: ...


class TerminalRecordTargetResolver(Protocol):
    def source_event_target(
        self,
        event: Optional[DetectionPublication],
        active_target: DetectedObject,
    ) -> Optional[DetectedObject]: ...


@dataclass(frozen=True)
class TerminalRecordPorts:
    uses_vision_nav: Callable[[], bool]
    terminal_recorded: Callable[[], bool]
    mark_terminal_recorded: Callable[[], None]
    record_detection: Callable[[DetectedObject], bool]
    debug: Callable[[str], None]
    mark_no_detection: Callable[[DetectedObject], None]


class TerminalRecordCommit:
    """Commit the first accepted visual record before command dispatch."""

    def __init__(
        self,
        ports: TerminalRecordPorts,
        source: TerminalRecordTargetResolver,
        deadline: TerminalRecordDeferral,
    ) -> None:
        self._ports = ports
        self._source = source
        self._deadline = deadline

    def commit(
        self,
        batch: NavSourceBatch,
        fresh_target: Optional[DetectedObject],
        active_target: DetectedObject,
    ) -> bool:
        if (
            not self._ports.uses_vision_nav()
            or self._ports.terminal_recorded()
        ):
            return True
        record_target = fresh_target
        if batch.source_driven:
            record_target = self._source.source_event_target(
                batch.latest_event,
                active_target,
            )
            if record_target is None:
                self._ports.mark_no_detection(active_target)
                return False
        if record_target is None:
            return False
        if not self._ports.record_detection(record_target):
            if batch.source_driven:
                self._ports.debug(
                    "terminal_record_rejected_source_event "
                    f"obj={record_target.identity.obj_id}"
                )
            self._deadline.defer_or_fail(record_target)
            return False
        self._ports.mark_terminal_recorded()
        return True


__all__ = [
    "TerminalRecordCommit",
    "TerminalRecordDeferral",
    "TerminalRecordPorts",
    "TerminalRecordTargetResolver",
]
