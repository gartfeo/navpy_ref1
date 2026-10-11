"""Bounded execution-fence acquisition for peer POI effects."""

from __future__ import annotations

import threading
from collections.abc import Callable


def await_peer_dispatch_quiescence(
    fence: threading.Lock,
    timeout_s: float,
    operation: str,
    record_failure: Callable[[BaseException], BaseException],
) -> None:
    timeout_s = max(0.0, float(timeout_s))
    acquired = fence.acquire(timeout=timeout_s)
    if acquired:
        fence.release()
        return
    failure = TimeoutError(
        "Peer POI dispatch did not quiesce for "
        f"{operation} within {timeout_s:.3f}s"
    )
    raise record_failure(failure)


__all__ = ["await_peer_dispatch_quiescence"]
