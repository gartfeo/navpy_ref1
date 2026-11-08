"""Sense/decide/act cycle and ArduPilot-rate scheduler loop."""

from __future__ import annotations

import math
import traceback
from collections.abc import Callable, Mapping

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.worker_failure import WorkerFailureLatch
from navpy.core.kernel.high_resolution_deadline import (
    bounded_scheduler_wall_period_s,
    wait_for_slot,
)
from navpy.core.flow.detection_snapshot import DetectionSnapshot
from navpy.core.flow.nav_clock import NavClock
from navpy.core.flow.nav_constants import NAV_LOOP_PERIOD_S
from navpy.core.flow.nav_state import NavPhaseState, NavState
from navpy.core.flow.navigation_decision import NavigationDecision
from navpy.core.flow.navigation_transition_handler import (
    NavigationTransitionHandler,
)
from navpy.core.perception.detector_ports import DetectionEventPort, DetectionSnapshotPort
from navpy.core.ports.detect_request import DetectRequest
from navpy.core.ports.detection_publication import DetectionPublication


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
            response.detected_pois,
            response.primary_poi,
            pending_events,
            append_events=self._phase.current == NavState.NAV,
        )


class StateActionDispatcher:
    """Apply transitions once, publish the committed phase, then act."""

    def __init__(
        self,
        phase: NavPhaseState,
        transitions: NavigationTransitionHandler,
        actions: Mapping[NavState, Callable[[], None]],
        publish_phase: Callable[[NavState], None],
    ) -> None:
        self._phase = phase
        self._transitions = transitions
        self._actions = dict(actions)
        self._publish_phase = publish_phase
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
        committed = self._phase.current
        # A raising transition publishes nothing: the last value stands
        # until the next commit.
        self._publish_phase(committed)
        self._actions[committed]()


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
        monotonic_s: Callable[[], float] = rt.perf_counter,
        sleep_s: Callable[[float], None] = rt.sleep,
        on_period: Callable[[bool], None] = lambda cycle_ok: None,
        progress: ProgressBeacon | None = None,
    ) -> None:
        self._cycle = cycle
        # Busy from the cycle through recovery's step, idle while pacing: the watchdog's view (nav_watchdog.py).
        self.progress = ProgressBeacon() if progress is None else progress
        self._clock = clock
        self._logger = logger
        # Told each period whether the cycle ran without an error; NAV
        # recovery (nav_recovery.py) decides the hold from it.
        self._on_period = on_period
        self._last_reported_error: tuple[str, str] | None = None
        self._monotonic_s = monotonic_s
        self._sleep_s = sleep_s
        self.period_s = NAV_LOOP_PERIOD_S
        self.stop_event = rt.Event()
        self._failure = WorkerFailureLatch()
        self.thread = rt.Thread(target=self._run_thread, daemon=True)

    def start(self, loop_rate_hz: float) -> None:
        # A rate that is NaN, infinite, non-positive or so tiny that its
        # period overflows keeps the default period. Each cycle bounds the
        # wall period after the speedup conversion (todo 291).
        if math.isfinite(loop_rate_hz) and loop_rate_hz > 0:
            period_s = 1.0 / loop_rate_hz
            if math.isfinite(period_s):
                self.period_s = period_s
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
        self._failure.raise_if_failed()

    def _run_thread(self) -> None:
        try:
            self._run()
        except BaseException as error:
            # Publish the fatal failure before fencing the worker so an owner
            # that observes shutdown can never miss the corresponding cause.
            self._failure.record(error)
            self.stop_event.set()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            started = self._monotonic_s()
            with self.progress.busy():
                cycle_ok = self._run_cycle()
                try:
                    self._on_period(cycle_ok)
                except Exception:  # noqa: BLE001 - recovery must not end the loop
                    traceback.print_exc()
            try:
                self._pace(started)
            except Exception:  # noqa: BLE001 - a timing fault must not end the loop
                traceback.print_exc()
                # Pace by the default period so the loop cannot spin.
                self.stop_event.wait(NAV_LOOP_PERIOD_S)

    def _run_cycle(self) -> bool:
        try:
            self._cycle.run()
            return True
        except OSError as error:
            # Operational I/O, connection and timeout failures: logged, and
            # counted by recovery like any other failed cycle.
            try:
                self._logger.error(f"nav loop error: {error}")
            except Exception:  # noqa: BLE001 - a failing logger must not end the loop
                traceback.print_exc()
            return False
        except Exception as error:  # noqa: BLE001 - fail closed, keep flying
            self._recover(error)
            return False

    def _pace(self, started: float) -> None:
        elapsed = self._monotonic_s() - started
        wall_period = bounded_scheduler_wall_period_s(
            self._clock.scheduler_wall_period(self.period_s),
            fallback_s=self.period_s,
        )
        if elapsed - wall_period > 0.5:
            self._logger.warning(
                f"Loop time exceeded: {elapsed:.3f} s",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
        # Skip the wait only for a measured overrun: a non-finite elapsed
        # time still waits, and wait_for_slot paces it (todo 291).
        overran = math.isfinite(elapsed) and elapsed >= wall_period
        if not overran and not self.stop_event.is_set():
            wait_for_slot(
                started + wall_period,
                wall_period,
                stop_event=self.stop_event,
                monotonic_s=self._monotonic_s,
                sleep_s=self._sleep_s,
            )

    def _recover(self, error: Exception) -> None:
        """Log and report a cycle failure; a repeating error is reported once.

        A cycle error never ends the loop (2026-11-05 owner ruling: no
        scenario may crash NavPy) and no longer fails the approach by itself:
        NavPy keeps its state, and recovery holds after NAV_HOLD_TRIGGER_S of
        failed periods (docs/design/navpy_recovery.md, step 4).
        """
        key = (type(error).__name__, str(error))
        repeated = key == self._last_reported_error
        self._last_reported_error = key
        try:
            if not repeated:
                self._logger.error(
                    f"nav loop internal error: {key[0]}: {key[1]}\n"
                    + "".join(traceback.format_exception(error))
                )
                self._logger.warning(
                    f"NavPy internal error: {key[0]}",
                    key="nav",
                    dest=LogStatusDest.DRONE,
                )
        except Exception:  # noqa: BLE001 - a failing logger must not end the loop
            traceback.print_exc()


__all__ = [
    "DetectionSensor",
    "NavigationCycle",
    "NavigationLoop",
    "StateActionDispatcher",
]
