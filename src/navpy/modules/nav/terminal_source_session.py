"""Source-event state for final approach.

This module deliberately knows nothing about the navigation state machine or
vehicle commands. It only classifies and filters detector publications.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Protocol

from navpy.modules.nav.terminal_source_contracts import ActiveSourceEvents
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.target_identity import get_target_task_id
from navpy.modules.vision.target_priority import find_target_by_task_id


class SourceEventCapability(Protocol):
    """Narrow detector surface needed to classify a target source."""

    has_source_driven_detection_events: bool

    def target_uses_source_driven_events(
            self,
            target: DetectedObject,
    ) -> Optional[bool]: ...


class TerminalSourceSession:
    """Classify source-event capability and active-target publications."""

    def __init__(
            self,
            events: SourceEventCapability,
    ) -> None:
        self._events = events

    def target_uses_source_events(
            self,
            *targets: Optional[DetectedObject],
    ) -> bool:
        for target in targets:
            if target is None:
                continue
            result = self._events.target_uses_source_driven_events(target)
            if result is True or result is False:
                return result
        return self._events.has_source_driven_detection_events is True

    def active_events(
            self,
            active_target: Optional[DetectedObject],
            pending_events: Iterable[DetectionPublication],
    ) -> ActiveSourceEvents:
        if active_target is None:
            return ActiveSourceEvents(())

        task_id = get_target_task_id(active_target)
        active_source_name = active_target.pixel.source_name
        matching = []
        for response in pending_events:
            detected = find_target_by_task_id(
                response.detected_targets,
                task_id,
            )
            same_empty_source = (
                detected is None
                and active_source_name is not None
                and response.source_name == active_source_name
            )
            if detected is not None or same_empty_source:
                matching.append(response)
        return ActiveSourceEvents(tuple(matching))

    @staticmethod
    def detection_receipt_age_s(target: DetectedObject) -> Optional[float]:
        raw_timestamp = target.timing.source_receipt_timestamp_s
        if not isinstance(raw_timestamp, (int, float)) or isinstance(
                raw_timestamp,
                bool,
        ):
            return None
        now_provider = target.timing.source_receipt_now_s
        if not callable(now_provider):
            return math.inf
        try:
            timestamp_s = float(raw_timestamp)
            now_s = float(now_provider())
        except Exception:  # noqa: BLE001 - external receipt clock fails closed
            return math.inf
        if not math.isfinite(timestamp_s) or not math.isfinite(now_s):
            return math.inf
        return now_s - timestamp_s
