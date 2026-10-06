"""Lifecycle ownership for the ArduPilot-rate navigation command worker."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from typing import ContextManager

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.navigation.navigation_command_worker import (
    AUTOPILOT_NAVIGATION_LOOP_HZ,
    NO_COMMAND_LOOP_OBSERVER,
    CommandLoopObserver,
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.modules.navigation.navigation_postprocess_dispatcher import (
    NavigationPostprocessDispatcher,
)
from navpy.modules.navigation.navigation_runtime import CommandWorkRuntime


NAVIGATION_COMMAND_JOIN_TIMEOUT_S = 2.0


def _no_source_dispatch() -> bool:
    return False


def _ignore_command_cadence(_active: bool) -> None:
    return None


class NavigationCommandSession:
    """Start, stop, and monitor one fixed-rate command worker generation."""

    def __init__(
        self,
        command_event: threading.Event,
        runtime_session: Callable[[], ContextManager[CommandWorkRuntime]],
        wall_period_s: Callable[[float], float],
        logger: ILogger,
        scheduler_rate_hz: float = AUTOPILOT_NAVIGATION_LOOP_HZ,
        source_dispatch: Callable[[], bool] = _no_source_dispatch,
        set_process_command_cadence_active: Callable[[bool], None] = (
            _ignore_command_cadence
        ),
        command_loop: CommandLoopObserver = NO_COMMAND_LOOP_OBSERVER,
    ) -> None:
        self._stop_event = threading.Event()
        self._command_event = command_event
        self._postprocess = NavigationPostprocessDispatcher(logger)
        self._worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
            stop_event=self._stop_event,
            command_event=command_event,
            wall_period_s=wall_period_s,
            runtime_session=runtime_session,
            logger=logger,
            postprocess_submit=self._postprocess.submit,
            source_dispatch=source_dispatch,
            set_process_command_cadence_active=(
                set_process_command_cadence_active
            ),
            scheduler_period_s=_scheduler_period_s(scheduler_rate_hz),
            command_loop=command_loop,
        ))
        self._thread: threading.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._postprocess.start()
        self._stop_event.clear()
        launch = ThreadLaunchGate()
        thread = threading.Thread(
            target=self._run_generation,
            args=(launch,),
            daemon=True,
        )
        self._thread = thread
        self._launch = launch
        self._started = True
        try:
            thread.start()
        except BaseException:
            self._stop_event.set()
            self._command_event.set()
            if launch.cancel_before_commit():
                self._thread = None
                self._launch = None
                self._started = False
            self._postprocess.close()
            raise

    def stop(self) -> None:
        self._stop_event.set()
        self._command_event.set()
        thread = self._thread
        launch = self._launch
        if thread is None:
            self._postprocess.close()
            return
        if launch is not None and launch.cancel_before_commit():
            self._thread = None
            self._launch = None
            self._started = False
            self._postprocess.close()
            return
        if thread is threading.current_thread():
            raise TimeoutError("navigation command worker cannot join itself")
        if thread.is_alive():
            thread.join(timeout=NAVIGATION_COMMAND_JOIN_TIMEOUT_S)
        if thread.is_alive():
            raise TimeoutError(
                "navigation command worker did not stop; runtime remains owned"
            )
        self._postprocess.close()
        self._thread = None
        self._launch = None
        self._started = False

    def raise_if_failed(self) -> None:
        self._postprocess.raise_if_failed()

    def _run_generation(self, launch: ThreadLaunchGate) -> None:
        if launch.enter():
            self._worker.run()


def _scheduler_period_s(rate_hz: float) -> float:
    if (
        isinstance(rate_hz, bool)
        or not isinstance(rate_hz, (int, float))
        or not math.isfinite(float(rate_hz))
        or float(rate_hz) <= 0.0
    ):
        raise ValueError("navigation scheduler rate must be finite and positive")
    return 1.0 / float(rate_hz)


__all__ = [
    "NAVIGATION_COMMAND_JOIN_TIMEOUT_S",
    "NavigationCommandSession",
]
