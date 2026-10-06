"""Sense/decide/act cycle and ArduPilot-rate scheduler loop."""

from __future__ import annotations

import threading
import time
import traceback
from collections.abc import Callable, Mapping

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.high_resolution_deadline import wait_until_deadline
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.nav_clock import NavClock
from navpy.modules.nav.nav_constants import NAV_LOOP_PERIOD_S
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.navigation_decision import NavigationDecision
from navpy.modules.nav.navigation_transition_handler import (
    NavigationTransitionHandler,
)
from navpy.modules.vision.detector_ports import DetectionEventPort, DetectionSnapshotPort
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detection_publication import DetectionPublication


NAV_LOOP_JOIN_TIMEOUT_S = 2.0


class DetectionSensor:
    """Read one detector response and drain source-frame publications."""

    def __init__(
        self,
        snapshot: DetectionSnapshotPort,
        events: DetectionEventPort,
        detections: DetectionSnapshot,
        phase: NavPhaseState,
    ) -> None:
        self._snapshot = snapshot
        self._events = events
        self._detections = detections
        self._phase = phase

    def sense(self) -> None:
        request = DetectRequest()
        pending_events: list[DetectionPublication] = list(
            self._events.drain_detection_events(request)
        )
        response = self._snapshot.get_detect_data(request)
        self._detections.replace(
            response.detected_targets,
            response.primary_target,
            pending_events,
            append_events=self._phase.current == NavState.NAV,
        )


class StateActionDispatcher:
    """Apply transitions once, then dispatch the current phase action."""

    def __init__(
        self,
        phase: NavPhaseState,
        transitions: NavigationTransitionHandler,
        actions: Mapping[NavState, Callable[[], None]],
    ) -> None:
        self._phase = phase
        self._transitions = transitions
        self._actions = dict(actions)
        expected = set(NavState)
        provided = set(self._actions)
        if provided != expected:
            missing = ", ".join(
                state.name for state in NavState if state not in provided
            )
            extra = ", ".join(
                repr(state) for state in provided if state not in expected
            )
            details = "; ".join(
                part
                for part in (f"missing: {missing}" if missing else "", f"extra: {extra}" if extra else "")
                if part
            )
            raise ValueError(f"action map must cover every NavState ({details})")

    def act(self) -> None:
        phase = self._phase.snapshot()
        requested = phase.current
        previous = phase.previous
        if requested != previous:
            outcome = self._transitions.on_change(previous, requested)
            self._phase.commit_transition(
                expected_previous=previous,
                requested=requested,
                effective=outcome.effective_state,
                oneshot_completed=outcome.oneshot_completed,
            )
        self._actions[self._phase.current]()


class NavigationCycle:
    """Run one deterministic sense/decide/act iteration."""

    def __init__(
        self,
        vehicle_armed: Callable[[], bool],
        detections: DetectionSnapshot,
        sensor: DetectionSensor,
        decision: NavigationDecision,
        actions: StateActionDispatcher,
    ) -> None:
        self._vehicle_armed = vehicle_armed
        self._detections = detections
        self._sensor = sensor
        self._decision = decision
        self._actions = actions

    def run(self) -> None:
        if self._vehicle_armed():
            self._sensor.sense()
        else:
            self._detections.clear()
        self._decision.decide()
        self._actions.act()


class NavigationLoop:
    """Schedule navigation at the requested ArduPilot loop frequency."""

    def __init__(
        self,
        cycle: NavigationCycle,
        clock: NavClock,
        logger: ILogger,
        monotonic_s: Callable[[], float] = time.perf_counter,
        sleep_s: Callable[[float], None] = time.sleep,
    ) -> None:
        self._cycle = cycle
        self._clock = clock
        self._logger = logger
        self._monotonic_s = monotonic_s
        self._sleep_s = sleep_s
        self.period_s = NAV_LOOP_PERIOD_S
        self.stop_event = threading.Event()
        self._failure_lock = threading.Lock()
        self._failure: BaseException | None = None
        self.thread = threading.Thread(target=self._run_thread, daemon=True)

    def start(self, loop_rate_hz: float) -> None:
        if loop_rate_hz > 0:
            self.period_s = 1.0 / loop_rate_hz
        if not self.thread.is_alive():
            self.stop_event.clear()
            self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=NAV_LOOP_JOIN_TIMEOUT_S)
        if self.thread.is_alive():
            raise RuntimeError(
                "navigation loop failed to terminate within "
                f"{NAV_LOOP_JOIN_TIMEOUT_S:.1f}s"
            )

    def is_stopping(self) -> bool:
        return self.stop_event.is_set()

    def raise_if_failed(self) -> None:
        with self._failure_lock:
            failure = self._failure
        if failure is not None:
            raise failure

    def _run_thread(self) -> None:
        try:
            self._run()
        except BaseException as error:
            # Publish the fatal failure before fencing the worker so an owner
            # that observes shutdown can never miss the corresponding cause.
            with self._failure_lock:
                self._failure = error
                self.stop_event.set()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            started = self._monotonic_s()
            try:
                self._cycle.run()
            except OSError as error:
                # Recover documented operational I/O, connection, and timeout
                # failures without hiding programming errors. The interruptible
                # stop_event wait below still makes shutdown deterministic.
                self._logger.error(f"nav loop error: {error}")
                traceback.print_exc()
            elapsed = self._monotonic_s() - started
            wall_period = self._clock.scheduler_wall_period(self.period_s)
            if elapsed - wall_period > 0.5:
                self._logger.warning(
                    f"Loop time exceeded: {elapsed:.3f} s",
                    key="nav",
                    dest=LogStatusDest.DRONE,
                )
            remaining_s = max(0.0, wall_period - elapsed)
            if remaining_s > 0.0 and not self.stop_event.is_set():
                wait_until_deadline(
                    started + wall_period,
                    stop_event=self.stop_event,
                    monotonic_s=self._monotonic_s,
                    sleep_s=self._sleep_s,
                )


__all__ = [
    "DetectionSensor",
    "NavigationCycle",
    "NavigationLoop",
    "StateActionDispatcher",
]
