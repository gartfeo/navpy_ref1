"""Thread lifecycle for periodic swarm heartbeats.

A generation that fails restarts fresh when the health check asks (``raise_if_failed``), with limited tries and
back-off (core/kernel/restart_budget.py, docs/design/navpy_recovery.md step 5c): a failed send or encode must not
silence this node for good. Only when the tries are used up, or for a BaseException, does the failure stay and
reach the health check, which escalates. After the owner's stop nothing restarts and the failure is reported.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional, Protocol

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.restart_budget import RestartBudget
from navpy.core.kernel.thread_launch import ThreadLaunchGate
from navpy.core.swarm.task_msg_refs import MsgRef


HEARTBEAT_INTERVAL_S = 1.0
HEARTBEAT_JOIN_TIMEOUT_S = 2.0


class HeartbeatEmitter(Protocol):
    def heartbeat(self) -> Optional[MsgRef]: ...


class SwarmHeartbeatRuntime:
    """Own one heartbeat generation; restart it fresh after a failure, within its tries."""

    def __init__(
        self,
        emitter: HeartbeatEmitter,
        report: Callable[[str], None] | None = None,
        budget: RestartBudget | None = None,
    ) -> None:
        self._emitter = emitter
        self._report = report
        self._budget = RestartBudget() if budget is None else budget
        self._wanted = False                 # the owner started it and has not stopped it
        self._lock = rt.Lock()
        self._stop = rt.Event()
        self._thread: rt.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._started = False
        self._failure: BaseException | None = None

    def start(self) -> bool:
        with self._lock:
            if self._failure is not None:
                raise self._failure
            if self._started:
                return False
            self._launch_locked()
            self._wanted = True
        return True

    def _launch_locked(self) -> None:
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
        stop = rt.Event()
        launch = ThreadLaunchGate()
        self._stop = stop
        self._started = True
        thread = rt.Thread(
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

    def is_started(self) -> bool:
        with self._lock:
            return self._started

    def stop(self) -> None:
        with self._lock:
            self._wanted = False
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

    def heartbeat_thread(self) -> rt.Thread | None:
        """Inspection seam for lifecycle tests; no mutable state is exposed."""
        with self._lock:
            return self._thread

    def raise_if_failed(self) -> None:
        """Restart a failed generation when its try is due; raise the failure only when it cannot be restarted."""
        with self._lock:
            failure = self._failure
            if failure is None:
                return
            if not self._restartable_locked(failure):
                raise failure
            if not self._budget.due() or self._generation_alive_locked():
                return     # back-off not over, or the failed generation is still leaving
            attempt = self._budget.attempt()
            self._failure = None
            try:
                self._launch_locked()
            except Exception as error:  # noqa: BLE001 - a failed try, counted
                self._failure = error
                self._budget.failed()
                message = f"Swarm heartbeat restart {attempt} failed: {error!r}"
            else:
                message = f"Swarm heartbeat restarted (try {attempt}) after: {failure!r}"
        self._safe_report(message)

    def _restartable_locked(self, failure: BaseException) -> bool:
        return isinstance(failure, Exception) and self._wanted and not self._budget.exhausted

    def _generation_alive_locked(self) -> bool:
        thread, launch = self._thread, self._launch
        if thread is None:
            return False
        return (launch is not None and not launch.committed) or thread.is_alive()

    def _safe_report(self, message: str) -> None:
        if self._report is None:
            return
        try:
            self._report(message)
        except Exception:  # noqa: BLE001 - reporting must not stop the restart
            pass

    def _run_generation(
        self,
        stop: rt.Event,
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
                    self._budget.failed()
                if self._stop is stop:
                    self._started = False
            stop.set()

    def _heartbeat_loop(self, stop: rt.Event) -> None:
        while not stop.wait(HEARTBEAT_INTERVAL_S):
            self._emitter.heartbeat()


__all__ = ["HeartbeatEmitter", "SwarmHeartbeatRuntime"]
