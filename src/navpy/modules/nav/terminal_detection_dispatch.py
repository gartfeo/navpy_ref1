"""Event admission, watchdog, and dispatch execution for final approach."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional, Protocol

from navpy.modules.nav.terminal_source_watchdog import (
    TerminalSourceReceiptWatchdog,
)
from navpy.modules.vision.models.detection_event_lease import (
    DetectionEventLease,
    DetectionLeaseDispatch,
)
from navpy.modules.vision.models.detection_publication import DetectionPublication


class TerminalEventHandler(Protocol):
    def handle(self, publication: DetectionPublication) -> bool: ...

    def handle_reset(self) -> None: ...


DispatchFailureReporter = Callable[[str, Optional[Exception]], None]


class TerminalDetectionDispatch:
    """Own event admission and watchdog state independently of lease lifetime."""

    def __init__(
        self,
        handler: TerminalEventHandler,
        wall_s: Callable[[], float],
        receipt_max_wall_age_s: Callable[[], float],
    ) -> None:
        self._handler = handler
        self._fence = threading.RLock()
        self._stop_event = threading.Event()
        self._activation_event = threading.Event()
        self._accepting_events = False
        self._watchdog = TerminalSourceReceiptWatchdog(
            wall_s,
            receipt_max_wall_age_s,
        )

    def prepare(self) -> None:
        self._stop_event.clear()
        self._activation_event.clear()
        with self._fence:
            self._accepting_events = False
            self._watchdog.disarm()

    def activate(self) -> None:
        with self._fence:
            self._accepting_events = True
            self._watchdog.arm()
        self._activation_event.set()

    def request_stop(self) -> None:
        self._stop_event.set()
        self._activation_event.set()

    def fence(self, deadline_s: float) -> None:
        remaining_s = max(0.0, deadline_s - time.monotonic())
        if not self._fence.acquire(timeout=remaining_s):
            raise TimeoutError(
                "terminal event dispatch fence did not quiesce in time"
            )
        try:
            self._accepting_events = False
            self._watchdog.disarm()
        finally:
            self._fence.release()

    def expire_receipt(self) -> bool:
        with self._fence:
            if not self._accepting_events or not self._watchdog.expired():
                return False
            self._stop_event.set()
            self._activation_event.set()
            self._accepting_events = False
            self._watchdog.disarm()
            return True

    def dispatch_available(
        self,
        lease: DetectionEventLease,
        report_failure: DispatchFailureReporter,
    ) -> bool:
        """Claim at most one publication on the current command deadline."""
        if self._stop_event.is_set() or not self._activation_event.is_set():
            return False
        try:
            result = lease.dispatch_available(self.handle_publication)
        except Exception as error:  # noqa: BLE001 - daemon boundary
            self.request_stop()
            report_failure("dispatch_failed", error)
            return False
        if result is DetectionLeaseDispatch.EMPTY:
            return False
        if result is DetectionLeaseDispatch.ACCEPTED:
            return True
        if result is DetectionLeaseDispatch.REJECTED:
            self.request_stop()
            return True
        if result is DetectionLeaseDispatch.CLOSED:
            stopping = self._stop_event.is_set()
            self.request_stop()
            if not stopping:
                report_failure("source_stream_closed", None)
            return False
        raise RuntimeError(f"unknown detection lease dispatch result: {result!r}")

    def handle_publication(self, publication: DetectionPublication) -> bool:
        with self._fence:
            if not self._accepting_events:
                return False
            self._watchdog.arm()
            return self._handler.handle(publication)

    def handle_source_reset(self) -> None:
        with self._fence:
            if self._accepting_events:
                self._watchdog.arm()
            self._handler.handle_reset()

    def reset_commands(self) -> None:
        self._handler.handle_reset()


__all__ = [
    "DispatchFailureReporter",
    "TerminalDetectionDispatch",
    "TerminalEventHandler",
]
