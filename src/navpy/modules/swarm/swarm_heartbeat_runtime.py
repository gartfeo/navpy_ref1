"""Thread lifecycle for periodic swarm heartbeats."""

from __future__ import annotations

import threading
from typing import Optional, Protocol

from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.swarm.task_msg_refs import MsgRef


HEARTBEAT_INTERVAL_S = 1.0
HEARTBEAT_JOIN_TIMEOUT_S = 2.0


class HeartbeatEmitter(Protocol):
    def heartbeat(self) -> Optional[MsgRef]: ...


class SwarmHeartbeatRuntime:
    """Own one restartable, failure-persistent heartbeat generation."""

    def __init__(self, emitter: HeartbeatEmitter) -> None:
        self._emitter = emitter
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._started = False
        self._failure: BaseException | None = None

    def start(self) -> bool:
        with self._lock:
            if self._failure is not None:
                raise self._failure
            if self._started:
                return False
            if self._thread is not None:
                if (
                    self._launch is not None
                    and not self._launch.committed
                ) or self._thread.is_alive():
                    raise RuntimeError(
                        "Cannot start heartbeat while the prior generation is alive"
                    )
                self._thread = None
                self._launch = None
            stop = threading.Event()
            launch = ThreadLaunchGate()
            self._stop = stop
            self._started = True
            thread = threading.Thread(
                target=self._run_generation,
                args=(stop, launch),
                daemon=True,
            )
            self._thread = thread
            self._launch = launch
            try:
                thread.start()
            except BaseException:
                self._started = False
                stop.set()
                if launch.cancel_before_commit() and self._thread is thread:
                    self._thread = None
                    self._launch = None
                raise
        return True

    def is_started(self) -> bool:
        with self._lock:
            return self._started

    def stop(self) -> None:
        with self._lock:
            self._started = False
            thread = self._thread
            launch = self._launch
            self._stop.set()
            if (
                thread is not None
                and launch is not None
                and launch.cancel_before_commit()
            ):
                self._thread = None
                self._launch = None
                return
        if thread is None:
            return
        thread.join(timeout=HEARTBEAT_JOIN_TIMEOUT_S)
        if thread.is_alive():
            raise RuntimeError("Swarm heartbeat thread did not stop")
        with self._lock:
            if self._thread is thread:
                self._thread = None
                self._launch = None

    def heartbeat_thread(self) -> threading.Thread | None:
        """Inspection seam for lifecycle tests; no mutable state is exposed."""
        with self._lock:
            return self._thread

    def raise_if_failed(self) -> None:
        """Raise the exact first fatal heartbeat failure persistently."""
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise failure

    def _run_generation(
        self,
        stop: threading.Event,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        try:
            self._heartbeat_loop(stop)
        except BaseException as failure:
            with self._lock:
                if self._failure is None:
                    self._failure = failure
                if self._stop is stop:
                    self._started = False
            stop.set()

    def _heartbeat_loop(self, stop: threading.Event) -> None:
        while not stop.wait(HEARTBEAT_INTERVAL_S):
            self._emitter.heartbeat()


__all__ = ["HeartbeatEmitter", "SwarmHeartbeatRuntime"]
