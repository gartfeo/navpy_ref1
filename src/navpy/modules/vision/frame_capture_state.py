"""Synchronized frame-capture generation state and publication admission."""

from __future__ import annotations

import threading
from enum import Enum, auto

import numpy as np

from navpy.modules.vision.frame_capture_ports import CaptureStamp
from navpy.modules.vision.frame_capture_session import CaptureSession
from navpy.modules.vision.frame_publication import FramePublicationStore


class _CapturePhase(Enum):
    STOPPED = auto()
    STARTING = auto()
    RUNNING = auto()
    STOPPING = auto()


class FrameCaptureState:
    """Own lifecycle serialization and atomic frame-publication admission."""

    def __init__(self, publications: FramePublicationStore) -> None:
        self._publications = publications
        self._condition = threading.Condition(threading.RLock())
        self._phase = _CapturePhase.STOPPED
        self._session: CaptureSession | None = None
        self._teardown_owner = False

    @property
    def is_running(self) -> bool:
        with self._condition:
            return self._phase is _CapturePhase.RUNNING

    @property
    def has_session(self) -> bool:
        with self._condition:
            return self._session is not None

    def begin_start(self) -> bool:
        with self._condition:
            if self._phase is not _CapturePhase.STOPPED:
                return False
            self._phase = _CapturePhase.STARTING
            return True

    def complete_push_start(self) -> None:
        with self._condition:
            self._phase = _CapturePhase.RUNNING
            self._publications.set_running(True)
            self._condition.notify_all()

    def install_starting(self, session: CaptureSession) -> None:
        with self._condition:
            self._session = session

    def commit_start(self, session: CaptureSession) -> None:
        with self._condition:
            if self._session is session:
                self._phase = _CapturePhase.RUNNING
                self._publications.set_running(True)
            session.ready_event.set()
            self._condition.notify_all()

    def retain_entered_start_failure(self, session: CaptureSession) -> None:
        with self._condition:
            if self._session is session:
                self._phase = _CapturePhase.STOPPING
                self._publications.set_running(False)
            self._condition.notify_all()

    def fail_start(self, session: CaptureSession | None = None) -> None:
        with self._condition:
            if session is None or self._session is session:
                self._session = None
                self._phase = _CapturePhase.STOPPED
                self._publications.set_running(False)
            self._condition.notify_all()

    def claim_teardown(self) -> CaptureSession | None:
        with self._condition:
            while self._phase is _CapturePhase.STARTING or self._teardown_owner:
                self._condition.wait()
            if self._phase is _CapturePhase.STOPPED:
                return None
            session = self._session
            if session is None:
                self._phase = _CapturePhase.STOPPED
                self._publications.set_running(False)
                self._condition.notify_all()
                return None
            self._teardown_owner = True
            self._phase = _CapturePhase.STOPPING
            self._publications.set_running(False)
            session.stop_event.set()
            return session

    def release_teardown_claim(self) -> None:
        with self._condition:
            self._teardown_owner = False
            self._condition.notify_all()

    def mark_worker_stopping(self, session: CaptureSession) -> None:
        with self._condition:
            if self._session is session:
                self._phase = _CapturePhase.STOPPING
                self._publications.set_running(False)
                session.stop_event.set()
                self._condition.notify_all()

    def owns(self, session: CaptureSession) -> bool:
        with self._condition:
            return self._session is session

    def retire(self, session: CaptureSession) -> None:
        with self._condition:
            if self._session is session:
                self._session = None
                self._phase = _CapturePhase.STOPPED
                self._publications.set_running(False)
                self._condition.notify_all()

    def publish_if_current(
        self,
        session: CaptureSession,
        frame: np.ndarray | None,
        capture: CaptureStamp | None = None,
    ) -> None:
        with self._condition:
            if (
                self._phase is not _CapturePhase.RUNNING
                or self._session is not session
                or session.stop_event.is_set()
            ):
                return
            self._publications.publish(frame, capture)


__all__ = ["FrameCaptureState"]
