from __future__ import annotations

import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from navpy.modules.common.models.location import Location
from navpy.modules.nav.peer_dispatch_quiescence import (
    await_peer_dispatch_quiescence,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import (
    get_target_task_id,
    same_target_identity,
)


# A transition may spend at most five nominal 50 Hz scheduler periods waiting
# for peer side effects.  Beyond that, fail closed instead of stalling navigation.
PEER_RESET_QUIESCENCE_TIMEOUT_S = 0.1
PEER_STOP_TIMEOUT_S = 2.0


@dataclass(frozen=True)
class PeerTargetDispatchPorts:
    """Narrow side-effect ports used outside the navigation command loop."""

    active_target: Callable[[], Optional[DetectedObject]]
    resolve_location: Callable[[DetectedObject], Optional[Location]]
    status_exists: Callable[[DetectedObject], bool]
    mark_notified: Callable[[DetectedObject], None]
    notify_targets: Callable[[list[DetectedObject]], None]
    warn: Callable[[str], None]
    error: Callable[[str, Exception], None]


@dataclass(frozen=True)
class _DispatchJob:
    generation: int
    target: DetectedObject


@dataclass(frozen=True)
class _PreparedDispatch:
    job: _DispatchJob
    location: Location | None


def _run_fenced_if_current(
    fence: threading.Lock,
    jobs: Sequence[_DispatchJob],
    is_current: Callable[[_DispatchJob], bool],
    effect: Callable[[], None],
) -> None:
    with fence:
        if any(is_current(job) for job in jobs):
            effect()


class _PeerTargetDispatcher:
    """Prepare and commit peer effects behind one generation fence."""

    def __init__(
        self,
        ports: PeerTargetDispatchPorts,
        fence: threading.Lock,
        is_current: Callable[[_DispatchJob], bool],
    ) -> None:
        self._ports = ports
        self._fence = fence
        self._is_current = is_current

    def prepare(
        self,
        jobs: Sequence[_DispatchJob],
    ) -> list[_PreparedDispatch]:
        active = self._ports.active_target()
        prepared: list[_PreparedDispatch] = []
        for job in jobs:
            if not self._is_current(job):
                continue
            target = job.target
            if active is not None and same_target_identity(target, active):
                continue
            if self._ports.status_exists(target):
                continue
            location = target.geo.projected_target_location
            if location is None:
                location = self._ports.resolve_location(target)
                if location is None:
                    _run_fenced_if_current(
                        self._fence,
                        [job],
                        self._is_current,
                        lambda: self._ports.warn(
                            "Peer target missing geo fix; skipping notify"
                        ),
                    )
                    continue
            if self._is_current(job):
                prepared.append(_PreparedDispatch(job, location))
        return prepared

    def report_error(
        self,
        jobs: Sequence[_DispatchJob],
        error: Exception,
    ) -> None:
        _run_fenced_if_current(
            self._fence,
            jobs,
            self._is_current,
            lambda: self._ports.error("Peer target dispatch failed", error),
        )

    def commit(self, prepared: Sequence[_PreparedDispatch]) -> None:
        with self._fence:
            current = [
                item for item in prepared if self._is_current(item.job)
            ]
            if not current:
                return
            try:
                targets: list[DetectedObject] = []
                for item in current:
                    target = item.job.target
                    if target.geo.projected_target_location is None:
                        target.set_p_t_g_loc(item.location)
                    targets.append(target)
                self._ports.notify_targets(targets)
                for target in targets:
                    self._ports.mark_notified(target)
            except OSError as exc:
                self._ports.error("Peer target dispatch failed", exc)


class PeerTargetDispatchWorker:
    """Run at most two NAV peer effects behind a generation reset fence."""

    MAX_PENDING_TARGETS = 2

    def __init__(self, ports: PeerTargetDispatchPorts) -> None:
        self._ports = ports
        self._condition = threading.Condition()
        self._execution_fence = threading.Lock()
        self._pending: OrderedDict[int, _DispatchJob] = OrderedDict()
        self._generation = 0
        self._stopped = False
        self._failure: BaseException | None = None
        self._dispatcher = _PeerTargetDispatcher(
            ports,
            self._execution_fence,
            self._job_is_current,
        )
        self._thread = threading.Thread(
            target=self._run_thread,
            name="nav-peer-target-dispatch",
            daemon=True,
        )

    def start(self) -> None:
        """Launch only after the owning runtime has stored this worker."""
        with self._condition:
            if self._stopped:
                self.raise_if_failed()
                raise RuntimeError("Peer target dispatch cannot restart")
        try:
            self._thread.start()
        except BaseException as failure:
            raise self._record_failure(failure)

    def submit(self, targets: Sequence[DetectedObject]) -> None:
        """Queue opaque target snapshots without geo, status, or network I/O."""
        with self._condition:
            if self._stopped:
                return
            generation = self._generation
            for target in targets:
                task_id = get_target_task_id(target)
                if task_id is None:
                    continue
                if (
                    task_id not in self._pending
                    and len(self._pending) >= self.MAX_PENDING_TARGETS
                ):
                    continue
                self._pending[task_id] = _DispatchJob(generation, target)
            if self._pending:
                self._condition.notify()

    def reset(
        self,
        timeout_s: float = PEER_RESET_QUIESCENCE_TIMEOUT_S,
    ) -> None:
        """Fence the old generation or fail without completing the transition."""
        with self._condition:
            self._generation += 1
            self._pending.clear()
        await_peer_dispatch_quiescence(
            self._execution_fence,
            timeout_s,
            "reset",
            self._record_failure,
        )

    def stop(self, timeout_s: float = PEER_STOP_TIMEOUT_S) -> None:
        """Fence effects, reject future work, and join the daemon worker."""
        deadline_s = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            self._generation += 1
            self._pending.clear()
            self._stopped = True
            self._condition.notify_all()
        await_peer_dispatch_quiescence(
            self._execution_fence,
            max(0.0, deadline_s - time.monotonic()),
            "stop",
            self._record_failure,
        )
        if self._thread.is_alive():
            self._thread.join(timeout=max(0.0, deadline_s - time.monotonic()))
        if self._thread.is_alive():
            failure = TimeoutError(
                "Peer target dispatch did not stop within "
                f"{max(0.0, float(timeout_s)):.3f}s"
            )
            raise self._record_failure(failure)

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def raise_if_failed(self) -> None:
        """Raise the exact first fatal worker failure on every health read."""
        with self._condition:
            failure = self._failure
        if failure is not None:
            raise failure

    def _run_thread(self) -> None:
        try:
            self._run()
        except BaseException as failure:
            self._record_failure(failure)

    def _record_failure(self, failure: BaseException) -> BaseException:
        with self._condition:
            if self._failure is None:
                self._failure = failure
            self._generation += 1
            self._pending.clear()
            self._stopped = True
            self._condition.notify_all()
            return self._failure

    def _run(self) -> None:
        while True:
            jobs = self._take_pending()
            if jobs is None:
                return
            try:
                prepared = self._dispatcher.prepare(jobs)
            except OSError as exc:
                self._dispatcher.report_error(jobs, exc)
                continue
            if not prepared:
                continue
            self._dispatcher.commit(prepared)

    def _take_pending(self) -> Optional[list[_DispatchJob]]:
        with self._condition:
            while not self._pending and not self._stopped:
                self._condition.wait()
            if self._stopped:
                return None
            jobs = list(self._pending.values())
            self._pending.clear()
            return jobs

    def _job_is_current(self, job: _DispatchJob) -> bool:
        with self._condition:
            return not self._stopped and job.generation == self._generation
