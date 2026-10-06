"""Clock, locking, polling, and subprocess primitives for demo evaluation."""

from __future__ import annotations

import os
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Iterator, Mapping, Sequence, TypeVar

from scripts.eval_gcs_demo_constants import EVALUATOR_LOCK_PATH, ROOT
from scripts.eval_gcs_demo_models import RegressionError, finite_number
from scripts.eval_gcs_demo_ports import ClockPort


T = TypeVar("T")


class SystemClock:
    @staticmethod
    def monotonic() -> float:
        return time.monotonic()

    @staticmethod
    def unix() -> float:
        return time.time()

    @staticmethod
    def sleep(seconds: float) -> None:
        time.sleep(seconds)


@contextmanager
def exclusive_evaluator_lock(
    lock_path: Path = EVALUATOR_LOCK_PATH,
) -> Iterator[None]:
    """Prevent two drivers from controlling one worktree stack."""
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    handle = lock_path.open("a+b")
    acquired = False
    try:
        handle.seek(0, os.SEEK_END)
        if handle.tell() == 0:
            handle.write(b"\0")
            handle.flush()
        handle.seek(0)
        try:
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            raise RegressionError(
                "another GCS demo evaluator is already running for this "
                f"worktree (lock: {lock_path})"
            ) from error
        acquired = True
        yield
    finally:
        if acquired:
            handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def wait_until(
    description: str,
    timeout_s: float,
    probe: Callable[[], T | None],
    *,
    interval_s: float = 0.25,
    transient_exceptions: tuple[type[Exception], ...] = (),
    clock: ClockPort | None = None,
) -> T:
    if not description:
        raise ValueError("wait description must not be empty")
    timeout = finite_number("poll timeout", timeout_s)
    interval = finite_number("poll interval", interval_s)
    if interval <= 0:
        raise ValueError("poll interval must be positive")
    if not all(
        isinstance(error_type, type) and issubclass(error_type, Exception)
        for error_type in transient_exceptions
    ):
        raise TypeError("transient_exceptions must contain Exception types")
    timer = clock or SystemClock()
    deadline = timer.monotonic() + timeout
    last_error: Exception | None = None
    while timer.monotonic() < deadline:
        try:
            value = probe()
        except transient_exceptions as error:
            last_error = error
        else:
            if value:
                return value
        timer.sleep(min(interval, max(0.0, deadline - timer.monotonic())))
    suffix = f"; last error: {last_error}" if last_error is not None else ""
    raise RegressionError(f"timed out waiting for {description}{suffix}")


def run_process(
    command: Sequence[str],
    env: Mapping[str, str],
    *,
    timeout_s: float,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(command),
        cwd=ROOT,
        env=dict(env),
        text=True,
        capture_output=True,
        timeout=timeout_s,
        check=False,
    )


__all__ = [
    "SystemClock",
    "exclusive_evaluator_lock",
    "run_process",
    "wait_until",
]
