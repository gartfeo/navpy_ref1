"""Fresh-source release gate for approved terminal approaches."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.nav.confirmation_reporting import ConfirmDebugReporter
from navpy.modules.nav.detection_freshness import DetectionFreshnessPolicy
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.target_identity import get_target_task_id
from navpy.modules.vision.target_priority import find_target_by_task_id


class TerminalReleaseSourceQuery(Protocol):
    """Source queries required before releasing an approved navigation_task."""

    def find_active_target_detection(self) -> DetectedObject | None: ...

    def target_uses_source_driven_events(
        self,
        *targets: DetectedObject | None,
    ) -> bool: ...

    def pending_active_target_events(self) -> list[DetectionPublication]: ...


class TerminalReleaseGate:
    """Require a fresh same-target frame before operator approval releases."""

    def __init__(
        self,
        source: TerminalReleaseSourceQuery,
        freshness: DetectionFreshnessPolicy,
        debug: ConfirmDebugReporter,
    ) -> None:
        self._source = source
        self._freshness = freshness
        self._debug = debug

    def is_ready(self, active_target: DetectedObject) -> bool:
        fresh_target = self._source.find_active_target_detection()
        if fresh_target is None:
            self._debug.log(
                "operator_approved_waiting_detection "
                f"obj={active_target.identity.obj_id}"
            )
            return False
        source_driven = self._source.target_uses_source_driven_events(
            fresh_target,
            active_target,
        )
        task_id = get_target_task_id(active_target)
        if not source_driven:
            return self._freshness.is_target_fresh_for_confirm(task_id)
        events = self._source.pending_active_target_events()
        latest_event = events[-1] if events else None
        event_target = (
            find_target_by_task_id(latest_event.detected_targets, task_id)
            if latest_event is not None
            else None
        )
        if (
            event_target is not None
            and not self._freshness.detection_is_fresh_for_confirm(
                event_target,
                require_source_receipt=True,
            )
        ):
            event_target = None
        if event_target is None:
            self._debug.log(
                "operator_approved_waiting_source_event "
                f"obj={active_target.identity.obj_id}"
            )
        return event_target is not None


__all__ = ["TerminalReleaseGate", "TerminalReleaseSourceQuery"]
