"""Bounded SIYI poller and payload shutdown primitives."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Protocol

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.thread_launch import ThreadLaunchGate


@dataclass(frozen=True)
class PollStopResult:
    stopped: bool


class PayloadShutdownPort(Protocol):
    def shutdown_payload(self) -> bool: ...


def stop_poll_thread(
    thread: threading.Thread | None,
    stop_event: threading.Event | None,
    launch: ThreadLaunchGate | None,
    timeout_s: float,
    logger: ILogger,
) -> PollStopResult:
    if stop_event is not None:
        stop_event.set()
    if thread is None:
        return PollStopResult(True)
    if launch is not None and launch.cancel_before_commit():
        return PollStopResult(True)
    if not thread.is_alive():
        return PollStopResult(True)
    thread.join(timeout=max(0.0, float(timeout_s)))
    if thread.is_alive():
        logger.warning("SIYI polling thread did not stop in time")
        return PollStopResult(False)
    return PollStopResult(True)


def require_acknowledgement(label: str, result: object) -> None:
    if result is not True:
        raise RuntimeError(f"SIYI {label} command was not acknowledged")


def shutdown_payload(sdk: PayloadShutdownPort) -> None:
    disconnected = sdk.shutdown_payload()
    if disconnected is False:
        raise TimeoutError("SIYI SDK threads did not stop during disconnect")


__all__ = [
    "PollStopResult",
    "PayloadShutdownPort",
    "require_acknowledgement",
    "shutdown_payload",
    "stop_poll_thread",
]
