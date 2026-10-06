"""Structured navigation-event recording."""

from __future__ import annotations

from typing import Any, Callable, Mapping

from navpy.logger.navigation_event_rows import build_event_line
from navpy.logger.navigation_log_streams import NavigationLogStreams
from navpy.logger.log_events import LogEvent


class NavigationEventRecorder:
    """Format and write event rows without owning logger-wide state."""

    def __init__(
        self,
        streams: NavigationLogStreams,
        timestamp: Callable[[], str],
        flush_interval: int,
    ) -> None:
        self._streams = streams
        self._timestamp = timestamp
        self._flush_interval = flush_interval

    def capture_timestamp(self) -> str:
        return self._timestamp()

    def record(
        self,
        event: LogEvent,
        payload: Mapping[str, Any],
        *,
        timestamp: str | None = None,
    ) -> None:
        self._streams.write_event(
            event,
            build_event_line(
                self._timestamp() if timestamp is None else timestamp,
                event,
                payload,
            ),
            flush_interval=self._flush_interval,
        )

    def has_compact_stream(self) -> bool:
        return self._streams.has_compact()
