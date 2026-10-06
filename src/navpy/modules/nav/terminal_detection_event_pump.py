"""Exclusive detector-to-navigation event transport during pure-vision NAV."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable

from navpy.modules.nav.terminal_detection_dispatch import (
    TerminalDetectionDispatch,
    TerminalEventHandler,
)
from navpy.modules.nav.terminal_detection_pump_session import (
    TerminalPumpSession,
    stop_terminal_pump_session,
)
from navpy.modules.vision.models.detection_event_lease import DetectionEventLease


TERMINAL_PUMP_STOP_TIMEOUT_S = 2.0


@dataclass(frozen=True)
class TerminalPumpFailure:
    reason: str
    error: Exception | None = None


@dataclass(frozen=True)
class TerminalDetectionEventPumpPorts:
    open_lease: Callable[[Callable[[], None]], DetectionEventLease | None]
    report_failure: Callable[[TerminalPumpFailure], None]
    wall_s: Callable[[], float]
    receipt_max_wall_age_s: Callable[[], float]


class TerminalDetectionEventPump:
    """Own one lease sampled by the navigation command scheduler."""

    def __init__(
        self,
        ports: TerminalDetectionEventPumpPorts,
        handler: TerminalEventHandler,
    ) -> None:
        self._ports = ports
        self._handler = handler
        self._lock = threading.RLock()
        self._session: TerminalPumpSession | None = None
        self._prepared = False
        self._dispatch = TerminalDetectionDispatch(
            handler,
            ports.wall_s,
            ports.receipt_max_wall_age_s,
        )

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._prepared

    def prepare(self) -> bool:
        """Claim the stream while leaving publication admission paused."""
        with self._lock:
            if self._session is not None:
                return self._prepared
            try:
                lease = self._ports.open_lease(self._dispatch.handle_source_reset)
            except Exception as error:  # noqa: BLE001 - fail closed at boundary
                self._fail("lease_open_failed", error)
                return False
            if lease is None:
                self._fail("lease_unavailable")
                return False
            self._dispatch.prepare()
            self._session = TerminalPumpSession(lease)
            self._prepared = True
            return True

    def activate(self) -> bool:
        """Allow a successfully prepared lease to dispatch publications."""
        with self._lock:
            if not self._prepared:
                self._fail("lease_not_prepared")
                return False
            self._dispatch.activate()
            return True

    def fail_if_source_receipt_expired(self) -> bool:
        """Fence the stream and fail if no receipt arrived within its budget."""
        with self._lock:
            if not self._prepared or not self._dispatch.expire_receipt():
                return False
            self._prepared = False
            session = self._session
            self._dispatch.request_stop()
        cleanup_error: Exception | None = None
        try:
            if session is not None:
                self._stop_session(session)
        except Exception as error:  # noqa: BLE001 - latch despite cleanup failure
            cleanup_error = error
        self._ports.report_failure(
            TerminalPumpFailure("source_receipt_timeout", cleanup_error)
        )
        return True

    def close(self) -> None:
        with self._lock:
            session = self._session
            self._prepared = False
            if session is None:
                self._dispatch.request_stop()
                return
            self._dispatch.request_stop()
        self._stop_session(session)

    def dispatch_available(self) -> bool:
        """Admit at most one newest publication without blocking for input."""
        with self._lock:
            session = self._session
            if not self._prepared or session is None:
                return False
            lease = session.lease
        return self._dispatch.dispatch_available(lease, self._fail)

    def _stop_session(self, session: TerminalPumpSession) -> None:
        def retire() -> None:
            with self._lock:
                if self._session is session:
                    self._session = None

        stop_terminal_pump_session(
            session,
            self._dispatch,
            TERMINAL_PUMP_STOP_TIMEOUT_S,
            retire,
        )

    def _fail(
        self,
        reason: str,
        error: Exception | None = None,
    ) -> None:
        try:
            self._handler.handle_reset()
        except Exception as reset_error:  # noqa: BLE001 - safety boundary
            if error is None:
                error = reset_error
        self._ports.report_failure(TerminalPumpFailure(reason, error))


__all__ = [
    "TerminalDetectionEventPump",
    "TerminalDetectionEventPumpPorts",
    "TerminalEventHandler",
    "TerminalPumpFailure",
]
