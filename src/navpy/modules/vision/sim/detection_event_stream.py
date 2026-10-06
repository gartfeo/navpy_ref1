"""Exclusive, bounded simulator source-publication stream."""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable
from typing import Sequence

from navpy.modules.vision.models.detection_event_lease import (
    DetectionLeaseDispatch,
)
from navpy.modules.vision.models.detection_publication import DetectionPublication


class PublicationEventLease:
    """Waitable queue that exclusively receives source publications."""

    def __init__(
        self,
        stream: "DetectionEventStream",
        initial: Sequence[DetectionPublication],
        reset_handler: Callable[[], None] | None,
    ) -> None:
        self._stream = stream
        self._events: deque[DetectionPublication] = deque(initial)
        self._reset_handler = reset_handler
        self._closed = False

    @property
    def closed(self) -> bool:
        with self._stream.condition:
            return self._closed

    def wait_and_dispatch(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> bool | None:
        stream = self._stream
        with stream.condition:
            while stream.clearing_locked or (
                not self._closed and not self._events
            ):
                stream.condition.wait()
            if self._closed:
                return None
            publication = self._next_publication_locked()
            stream.begin_dispatch_locked()
        try:
            return handler(publication)
        finally:
            stream.finish_dispatch()

    def dispatch_available(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> DetectionLeaseDispatch:
        """Claim at most one current publication without waiting."""
        stream = self._stream
        with stream.condition:
            if self._closed:
                return DetectionLeaseDispatch.CLOSED
            if stream.clearing_locked or not self._events:
                return DetectionLeaseDispatch.EMPTY
            publication = self._next_publication_locked()
            stream.begin_dispatch_locked()
        try:
            return (
                DetectionLeaseDispatch.ACCEPTED
                if handler(publication)
                else DetectionLeaseDispatch.REJECTED
            )
        finally:
            stream.finish_dispatch()

    def close(self) -> None:
        self._stream.close_lease(self)

    def append_locked(self, publication: DetectionPublication) -> None:
        self._events.append(publication)

    def _next_publication_locked(self) -> DetectionPublication:
        """Deliver reset boundaries, otherwise the newest visual state."""
        for index, publication in enumerate(self._events):
            if publication.source_discontinuity:
                for _ in range(index):
                    self._events.popleft()
                return self._events.popleft()
        latest = self._events.pop()
        self._events.clear()
        return latest

    def event_count_locked(self) -> int:
        return len(self._events)

    def drain_locked(self) -> list[DetectionPublication]:
        publications = list(self._events)
        self._events.clear()
        return publications

    def mark_closed_locked(self) -> None:
        self._closed = True

    def reset_handler_locked(self) -> Callable[[], None] | None:
        return self._reset_handler


class DetectionEventStream:
    """Own publication ordering, leasing, and reset dispatch fences."""

    def __init__(self, condition: threading.Condition) -> None:
        self.condition = condition
        self._events: deque[DetectionPublication] = deque()
        self._lease: PublicationEventLease | None = None
        self._stopped = False
        self._clearing = False
        self._active_dispatches = 0
        self._discontinuity_pending = False

    @property
    def clearing_locked(self) -> bool:
        return self._clearing

    @property
    def stopped_locked(self) -> bool:
        return self._stopped

    def pending_count_locked(self) -> int:
        lease_count = (
            0 if self._lease is None else self._lease.event_count_locked()
        )
        return len(self._events) + lease_count

    def publish_locked(self, publication: DetectionPublication) -> None:
        discontinuity = bool(
            publication.source_discontinuity or self._discontinuity_pending
        )
        self._discontinuity_pending = False
        if discontinuity != publication.source_discontinuity:
            publication = DetectionPublication(
                publication.detected_targets,
                publication.source_timestamp_s,
                publication.source_receipt_timestamp_s,
                publication.source_name,
                discontinuity,
            )
        if self._lease is None:
            self._events.append(publication)
        else:
            self._lease.append_locked(publication)

    def drain_locked(self) -> list[DetectionPublication]:
        publications = list(self._events)
        self._events.clear()
        self.condition.notify_all()
        return publications

    def open_lease(
        self,
        reset_handler: Callable[[], None] | None,
    ) -> PublicationEventLease:
        with self.condition:
            while self._clearing and not self._stopped:
                self.condition.wait()
            if self._stopped:
                raise RuntimeError("source publication stream is stopped")
            if self._lease is not None:
                raise RuntimeError("source publication stream is already leased")
            initial = tuple(self._events)
            reset_required = self._discontinuity_pending or any(
                publication.source_discontinuity
                for publication in initial
            )
            lease = PublicationEventLease(
                self,
                initial,
                reset_handler,
            )
            self._events.clear()
            self._lease = lease
            self.condition.notify_all()
        if reset_required and reset_handler is not None:
            try:
                reset_handler()
            except BaseException:
                lease.close()
                raise
        return lease

    def close_lease(self, lease: PublicationEventLease) -> None:
        with self.condition:
            if self._lease is not lease:
                lease.mark_closed_locked()
                self.condition.notify_all()
                return
            lease.mark_closed_locked()
            self.condition.notify_all()
            self._wait_for_dispatches_locked()
            self._events.extend(lease.drain_locked())
            self._lease = None
            self.condition.notify_all()

    def begin_dispatch_locked(self) -> None:
        self._active_dispatches += 1
        self.condition.notify_all()

    def finish_dispatch(self) -> None:
        with self.condition:
            self._active_dispatches -= 1
            self.condition.notify_all()

    def begin_clear_locked(
        self,
        *,
        mark_discontinuity: bool,
    ) -> tuple[Callable[[], None] | None, list[DetectionPublication]]:
        self._clearing = True
        self._wait_for_dispatches_locked()
        reset_handler = (
            None
            if self._lease is None
            else self._lease.reset_handler_locked()
        )
        dropped = self._drain_all_locked(close_lease=False)
        self._discontinuity_pending = bool(mark_discontinuity)
        return reset_handler, dropped

    def begin_stop_locked(
        self,
    ) -> tuple[Callable[[], None] | None, list[DetectionPublication]]:
        self._stopped = True
        self._clearing = True
        if self._lease is not None:
            self._lease.mark_closed_locked()
        self.condition.notify_all()
        self._wait_for_dispatches_locked()
        reset_handler = (
            None
            if self._lease is None
            else self._lease.reset_handler_locked()
        )
        dropped = self._drain_all_locked(close_lease=True)
        self._discontinuity_pending = False
        return reset_handler, dropped

    def finish_reset_locked(self) -> None:
        """Release the shared clear/stop publication barrier."""
        self._clearing = False
        self.condition.notify_all()

    def _wait_for_dispatches_locked(self) -> None:
        while self._active_dispatches:
            self.condition.wait()

    def _drain_all_locked(
        self,
        *,
        close_lease: bool,
    ) -> list[DetectionPublication]:
        dropped = list(self._events)
        self._events.clear()
        if self._lease is not None:
            dropped.extend(self._lease.drain_locked())
            if close_lease:
                self._lease.mark_closed_locked()
                self._lease = None
        return dropped


__all__ = ["DetectionEventStream", "PublicationEventLease"]
