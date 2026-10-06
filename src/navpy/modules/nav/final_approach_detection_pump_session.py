"""Retryable ownership transaction for one final-approach event-pump session."""

from __future__ import annotations

import threading
import time
from typing import Callable

from navpy.exception_groups import ExceptionGroup
from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.nav.final_approach_detection_dispatch import (
    FinalApproachDetectionDispatch,
)
from navpy.modules.vision.models.detection_event_lease import DetectionEventLease


class FinalApproachPumpSession:
    """Retain lease and dispatch ownership until every stop step completes."""

    def __init__(
        self,
        lease: DetectionEventLease,
    ) -> None:
        self.lease = lease
        self._lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._lease_close_thread: threading.Thread | None = None
        self._lease_close_launch: ThreadLaunchGate | None = None
        self._lease_close_error: BaseException | None = None
        self._lease_closed = False
        self._commands_reset = False

    def acquire_stop(self, deadline_s: float) -> bool:
        return self._stop_lock.acquire(
            timeout=max(0.0, deadline_s - time.monotonic())
        )

    def release_stop(self) -> None:
        self._stop_lock.release()

    @property
    def complete(self) -> bool:
        return self._commands_reset and self._lease_closed

    def reset_commands(self, reset: Callable[[], None]) -> None:
        if self._commands_reset:
            return
        reset()
        self._commands_reset = True

    def close_lease(self, deadline_s: float) -> bool:
        with self._lock:
            if self._lease_closed:
                return True
            thread = self._lease_close_thread
            launch = self._lease_close_launch
            if thread is None:
                launch = ThreadLaunchGate()
                thread = threading.Thread(
                    target=self._run_lease_close,
                    args=(launch,),
                    name="final-approach-pump-lease-close",
                    daemon=True,
                )
                self._lease_close_thread = thread
                self._lease_close_launch = launch
                try:
                    thread.start()
                except BaseException:
                    if launch.cancel_before_commit():
                        self._lease_close_thread = None
                        self._lease_close_launch = None
                    raise

        remaining_s = max(0.0, deadline_s - time.monotonic())
        if not launch.committed:
            launch.wait_until_entered(remaining_s)
        if not launch.committed:
            if launch.cancel_before_commit():
                with self._lock:
                    if self._lease_close_thread is thread:
                        self._lease_close_thread = None
                        self._lease_close_launch = None
            return False
        if thread is threading.current_thread():
            return False
        if thread.is_alive():
            thread.join(timeout=max(0.0, deadline_s - time.monotonic()))
        if thread.is_alive():
            return False
        with self._lock:
            error = self._lease_close_error
            self._lease_close_error = None
            self._lease_close_thread = None
            self._lease_close_launch = None
            if error is None:
                self._lease_closed = True
        if error is not None:
            raise error
        return True

    def _run_lease_close(self, launch: ThreadLaunchGate) -> None:
        if not launch.enter():
            return
        try:
            self.lease.close()
        except BaseException as error:
            with self._lock:
                self._lease_close_error = error


def stop_final_approach_pump_session(
    session: FinalApproachPumpSession,
    dispatch: FinalApproachDetectionDispatch,
    timeout_s: float,
    retire: Callable[[], None],
) -> None:
    """Run one bounded, retryable cleanup transaction."""
    deadline_s = time.monotonic() + timeout_s
    if not session.acquire_stop(deadline_s):
        raise TimeoutError(
            "final-approach event pump cleanup transaction did not become available"
        )
    try:
        dispatch.fence(deadline_s)
        errors: list[Exception] = []
        try:
            session.reset_commands(dispatch.reset_commands)
        except Exception as error:  # noqa: BLE001 - retryable action
            errors.append(error)
        try:
            if not session.close_lease(deadline_s):
                errors.append(TimeoutError(
                    "final-approach detection lease did not close in time"
                ))
        except Exception as error:  # noqa: BLE001 - retryable action
            errors.append(error)
        if session.complete:
            retire()
        if len(errors) == 1:
            raise errors[0]
        if errors:
            raise ExceptionGroup("final-approach event pump cleanup failed", errors)
        if not session.complete:
            raise TimeoutError("final-approach event pump cleanup remains incomplete")
    finally:
        session.release_stop()


__all__ = ["FinalApproachPumpSession", "stop_final_approach_pump_session"]
