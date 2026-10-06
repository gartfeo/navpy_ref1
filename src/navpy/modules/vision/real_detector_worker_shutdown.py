"""Bounded detector-worker quiescence without releasing live dependencies."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Sequence

from navpy.modules.common.thread_launch import ThreadLaunchGate


@dataclass(frozen=True)
class WorkerThread:
    name: str
    thread: threading.Thread
    launch: ThreadLaunchGate


@dataclass(frozen=True)
class WorkerQuiescence:
    alive: tuple[WorkerThread, ...]
    errors: tuple[BaseException, ...]

    @property
    def is_complete(self) -> bool:
        return not self.alive


def wait_for_worker_quiescence(
    threads: Sequence[WorkerThread],
    context: str,
    warning: Callable[[str], None],
    timeout_s: float,
) -> WorkerQuiescence:
    """Spend one total deadline waiting, retaining every live worker."""
    timeout_s = max(0.0, float(timeout_s))
    deadline_s = time.monotonic() + timeout_s
    alive: list[WorkerThread] = []
    errors: list[BaseException] = []
    for owned in threads:
        name = owned.name
        thread = owned.thread
        if owned.launch.cancel_before_commit():
            continue
        try:
            was_alive = thread.is_alive()
        except BaseException as error:
            errors.append(error)
            alive.append(owned)
            try:
                warning(
                    f"Detector: {name} thread state is unknown during "
                    f"{context}; retaining dependencies for retry"
                )
            except BaseException:
                pass
            continue
        if not was_alive:
            continue
        try:
            thread.join(timeout=max(0.0, deadline_s - time.monotonic()))
        except BaseException as error:
            errors.append(error)
        try:
            is_alive = thread.is_alive()
        except BaseException as error:
            errors.append(error)
            alive.append(owned)
            try:
                warning(
                    f"Detector: {name} thread state is unknown during "
                    f"{context}; retaining dependencies for retry"
                )
            except BaseException:
                pass
            continue
        if not is_alive:
            continue
        alive.append(owned)
        timeout = TimeoutError(
            f"Detector: {name} thread did not stop during {context} "
            f"within {timeout_s:.3f}s"
        )
        errors.append(timeout)
        try:
            warning(f"{timeout}; retaining dependencies for retry")
        except BaseException:
            pass
    return WorkerQuiescence(tuple(alive), tuple(errors))


__all__ = ["WorkerQuiescence", "WorkerThread", "wait_for_worker_quiescence"]
