"""A GCS request's budget for its vehicle exchanges (``docs/design/radio_budget.md``).

Todo 103 (parameters) and todo 105 (mission transfers): a provisional
:data:`GCS_EXCHANGE_ATTEMPTS` sends per step, one absolute deadline for the
request, and an end when the autopilot has gone silent. The deadline bounds
server work, not the response: callers keep their HTTP timeout above it.
Lockstep counts it in sim time.
"""
from __future__ import annotations

import math
from collections.abc import Iterator
from contextlib import contextmanager

from fastapi import HTTPException

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.runtime_time import LockPort
from navpy.core.ports.parameter_exchange import ExchangePolicy

#: Sends per exchange step: a provisional budget from an estimated 44 % per-send
#: failure of a parameter echo at the ``stress`` byte loss, 0.44^12 < 1e-4 if
#: sends fail independently (todo 103); mission steps reuse it (todo 105).
GCS_EXCHANGE_ATTEMPTS = 12
#: ``GET /mission``'s budget when it names none; the eval's HTTP timeout is 90 s.
DEFAULT_MISSION_REQUEST_S = 85.0
#: ``POST /upload``'s budget for all its vehicles; the eval's HTTP timeout is 240 s.
MISSION_UPLOAD_REQUEST_S = 225.0


def gcs_policy(budget_s: float) -> ExchangePolicy:
    """A request's policy: its deadline is ``budget_s`` from now."""
    return ExchangePolicy(GCS_EXCHANGE_ATTEMPTS, rt.monotonic() + budget_s, stop_on_silence=True)


def requested_policy(timeout_s: object, default_s: float) -> ExchangePolicy:
    """The policy for a request's ``timeout_s`` (``default_s`` when absent); 400 if invalid."""
    raw = default_s if timeout_s is None else timeout_s
    try:
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise TypeError(raw)
        budget = float(raw)  # an oversized JSON integer overflows here
    except (TypeError, OverflowError):
        budget = math.nan
    if not math.isfinite(budget) or budget <= 0:
        raise HTTPException(status_code=400, detail="timeout_s must be a finite positive number")
    return gcs_policy(budget)


def expired(policy: ExchangePolicy | None) -> bool:
    """Whether the request's deadline has passed: start no more vehicle work."""
    return policy is not None and policy.deadline_s is not None and rt.monotonic() >= policy.deadline_s


@contextmanager
def held_within(lock: LockPort, policy: ExchangePolicy | None) -> Iterator[bool]:
    """Hold ``lock`` if it comes free before the policy's deadline; yields whether it did.

    Past the deadline nothing is held, even a free lock: the caller must not
    start a clear or a transfer it cannot finish (todo 105)."""
    if policy is None or policy.deadline_s is None:
        acquired = lock.acquire()
    elif expired(policy):
        acquired = False
    else:
        acquired = lock.acquire(timeout=max(0.0, policy.deadline_s - rt.monotonic()))
    try:
        yield acquired and not expired(policy)
    finally:
        if acquired:
            lock.release()


__all__ = [
    "DEFAULT_MISSION_REQUEST_S", "GCS_EXCHANGE_ATTEMPTS", "MISSION_UPLOAD_REQUEST_S",
    "expired", "gcs_policy", "held_within", "requested_policy",
]
