"""Background execution loop for queued navigation commands."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, ContextManager, Protocol

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.high_resolution_deadline import wait_until_deadline
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_postprocess_dispatcher import (
    PostprocessJob,
)
from navpy.modules.navigation.navigation_runtime import CommandWorkRuntime


AUTOPILOT_NAVIGATION_LOOP_HZ = 50.0
AUTOPILOT_NAVIGATION_PERIOD_S = 1.0 / AUTOPILOT_NAVIGATION_LOOP_HZ


def _run_postprocess_inline(job: PostprocessJob) -> None:
    job()


def _ignore_command_cadence(_active: bool) -> None:
    return None


def _no_source_dispatch() -> bool:
    return False


class CommandLoopObserver(Protocol):
    """Record-only view of this loop, for the determinism trace.

    Strictly one-way: the loop tells the observer what it did and never
    asks it anything, so no recorder value can reach a command. An
    iteration index is this worker's own loop count -- it skips a missed
    deadline, so it is NOT an autopilot slot index.
    """

    def note_iteration(self, iteration: int) -> None:
        ...

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None:
        ...


class _IgnoreCommandLoop:
    """The default: unobserved. Two no-op calls per pass, no branch."""

    __slots__ = ()

    def note_iteration(self, iteration: int) -> None: ...

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None: ...


NO_COMMAND_LOOP_OBSERVER = _IgnoreCommandLoop()


@dataclass(frozen=True)
class NavigationCommandWorkerPorts:
    """Narrow synchronization and runtime ports used by the worker."""

    stop_event: threading.Event
    command_event: threading.Event
    wall_period_s: Callable[[float], float]
    runtime_session: Callable[[], ContextManager[CommandWorkRuntime]]
    logger: ILogger
    postprocess_submit: Callable[[PostprocessJob], None] = _run_postprocess_inline
    source_dispatch: Callable[[], bool] = _no_source_dispatch
    command_loop: CommandLoopObserver = NO_COMMAND_LOOP_OBSERVER
    set_process_command_cadence_active: Callable[[bool], None] = (
        _ignore_command_cadence
    )
    scheduler_period_s: float = AUTOPILOT_NAVIGATION_PERIOD_S
    # On supported Windows Python runtimes ``time.monotonic`` can advance in
    # ~15.6 ms steps.  Accelerated SITL needs sub-tick AP deadlines (4 ms at
    # 10x), so use the high-resolution monotonic counter for scheduling only.
    # Observation/source timestamps remain untouched in their producer clocks.
    monotonic_s: Callable[[], float] = time.perf_counter
    sleep_s: Callable[[float], None] = time.sleep


class NavigationCommandWorker:
    """Issue the first command after one AP slot, then hold fixed deadlines."""

    def __init__(self, ports: NavigationCommandWorkerPorts) -> None:
        self._ports = ports

    def run(self) -> None:
        ports = self._ports
        period_s = self._wall_period_s()
        next_deadline_s = ports.monotonic_s() + period_s
        command_cadence_active = False
        iteration = 0
        try:
            while not ports.stop_event.is_set():
                signaled, stopped = self._wait_for_deadline(next_deadline_s)
                if stopped:
                    break
                # Numbered at the TOP of the pass, not before the source
                # dispatch: the command issued below and the dispatch that
                # follows it belong to the SAME iteration, and attributing
                # them to different ones would compare unlike things.
                iteration += 1
                self._note_iteration(iteration)
                postprocess_job: PostprocessJob | None = None
                continuing_work = False
                with ports.runtime_session() as runtime:
                    if ports.stop_event.is_set():
                        break
                    has_work = (
                        signaled or runtime.has_command_pending_or_in_flight()
                    )
                    if has_work and not command_cadence_active:
                        ports.set_process_command_cadence_active(True)
                        command_cadence_active = True
                    work = runtime.take_work() if has_work else None
                    if work is not None:
                        issued: CalcData | None = None
                        # Cleared only by a normal return, so anything
                        # that escapes execute_work is recorded as a
                        # crash -- including a BaseException, which the
                        # handler below deliberately does not catch.
                        raised = True
                        try:
                            issued = runtime.execute_work(work)
                            raised = False
                        except Exception as exc:
                            ports.logger.error(
                                f"Error in final-approach loop: {exc}", exc
                            )
                        finally:
                            # The object execute_work RETURNED, digested
                            # by the observer. Never recomputed here: a
                            # recomputed command measures this code, not
                            # the command the vehicle was given.
                            self._note_command(iteration, issued, raised)
                            # Capture the immutable evidence closure before the
                            # lease is released, but execute it only after leaving
                            # the runtime session.  Attempt both operations even
                            # if one cleanup fails so a final-approach postprocess fence
                            # cannot be stranded by a lease-release error.
                            try:
                                postprocess_job = runtime.postprocess_job(work)
                            except Exception as exc:
                                ports.logger.error(
                                    "Error preparing final-approach postprocess: "
                                    f"{exc}",
                                    exc,
                                )
                            try:
                                runtime.finish_work(work)
                            except Exception as exc:
                                ports.logger.error(
                                    f"Error finishing final-approach work: {exc}",
                                    exc,
                                )
                    continuing_work = (
                        runtime.has_command_pending_or_in_flight()
                    )

                if postprocess_job is not None:
                    try:
                        ports.postprocess_submit(postprocess_job)
                    except Exception as exc:
                        ports.logger.error(
                            f"Error in final-approach postprocess: {exc}",
                            exc,
                        )

                source_work_taken = False
                if not ports.stop_event.is_set():
                    try:
                        # The current deadline belongs to the command already
                        # staged by the prior source sample. Admit one newest
                        # sample only after issue and postprocess submission so
                        # it can affect the following ArduPilot slot, never this
                        # one.
                        source_work_taken = bool(ports.source_dispatch())
                    except Exception as exc:
                        ports.logger.error(
                            f"Error dispatching final-approach source: {exc}",
                            exc,
                        )

                cadence_work = continuing_work or source_work_taken
                if cadence_work and not command_cadence_active:
                    ports.set_process_command_cadence_active(True)
                    command_cadence_active = True
                elif not cadence_work and command_cadence_active:
                    ports.set_process_command_cadence_active(False)
                    command_cadence_active = False

                next_period_s = self._wall_period_s()
                completed_s = ports.monotonic_s()
                if not math.isclose(
                    next_period_s,
                    period_s,
                    rel_tol=1e-9,
                    abs_tol=1e-12,
                ):
                    # A verified SIM_SPEEDUP change begins a new wall-cadence
                    # epoch. Source clocks remain untouched.
                    period_s = next_period_s
                    next_deadline_s = completed_s + period_s
                else:
                    scheduled_s = next_deadline_s + period_s
                    next_deadline_s = _next_fixed_deadline(
                        scheduled_s,
                        completed_s,
                        period_s,
                    )
        finally:
            if command_cadence_active:
                ports.set_process_command_cadence_active(False)

    def _note_iteration(self, iteration: int) -> None:
        """Observation must never reach the command it observes."""
        try:
            self._ports.command_loop.note_iteration(iteration)
        except Exception as exc:  # noqa: BLE001
            self._ports.logger.error(
                f"Error observing command iteration: {exc}", exc
            )

    def _note_command(
        self, iteration: int, command: Any, raised: bool
    ) -> None:
        try:
            self._ports.command_loop.note_command(
                iteration, command, raised
            )
        except Exception as exc:  # noqa: BLE001
            self._ports.logger.error(
                f"Error observing issued command: {exc}", exc
            )

    def _wait_for_deadline(self, deadline_s: float) -> tuple[bool, bool]:
        """Sleep to the AP deadline without Windows Event timeout quantization."""
        ports = self._ports
        reached = wait_until_deadline(
            deadline_s,
            stop_event=ports.stop_event,
            monotonic_s=ports.monotonic_s,
            sleep_s=ports.sleep_s,
        )
        if not reached:
            return False, True
        return bool(ports.command_event.wait(timeout=0.0)), False

    def _wall_period_s(self) -> float:
        return max(
            0.0,
            float(self._ports.wall_period_s(self._ports.scheduler_period_s)),
        )


def _next_fixed_deadline(
    scheduled_s: float,
    completed_s: float,
    period_s: float,
) -> float:
    """Return the first unmissed deadline on the existing AP wall grid."""
    if period_s <= 0.0:
        return completed_s
    if completed_s <= math.nextafter(scheduled_s, math.inf):
        return scheduled_s
    slots_late = math.ceil((completed_s - scheduled_s) / period_s)
    candidate = scheduled_s + slots_late * period_s
    # Protect against a downward-rounded product. Never replay a missed slot.
    return candidate + period_s if candidate < completed_s else candidate


__all__ = [
    "AUTOPILOT_NAVIGATION_LOOP_HZ",
    "AUTOPILOT_NAVIGATION_PERIOD_S",
    "NO_COMMAND_LOOP_OBSERVER",
    "CommandLoopObserver",
    "NavigationCommandWorker",
    "NavigationCommandWorkerPorts",
]
