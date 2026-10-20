"""The send-and-wait loop of one parameter exchange under an :class:`ExchangePolicy`.

``docs/design/radio_budget.md``, todo 97: the deadline caps the lock wait and
every answer wait and is checked again before each send; with
``stop_on_silence`` the link is checked after taking the lock and before each
send. An exchange that sent nothing is ``unattempted``; one that sent without
a matching answer is ``unconfirmed``.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.runtime_time import LockPort
from navpy.core.ports.parameter_exchange import (
    ExchangeOutcome,
    ExchangePolicy,
    ExchangeReason,
    ParameterExchange,
)
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.message_store import MessageSample


def run_exchange(
    policy: ExchangePolicy,
    link: HeartbeatState,
    send: Callable[[], None],
    wait: Callable[[float], MessageSample | None],
    timeout: float,
    *,
    lock: LockPort | None = None,
    on_miss: Callable[[int], None] | None = None,
) -> tuple[ParameterExchange, MessageSample | None]:
    """``wait(deadline)`` returns the matching answer or ``None``; ``on_miss(attempt)``
    hears each send that got none."""
    if lock is not None and not _acquire(lock, policy):
        return ParameterExchange(ExchangeOutcome.UNATTEMPTED, ExchangeReason.DEADLINE, 0), None
    try:
        return _exchange(policy, link, send, wait, timeout, on_miss)
    finally:
        if lock is not None:
            lock.release()


def _acquire(lock: LockPort, policy: ExchangePolicy) -> bool:
    if policy.deadline_s is None:
        return lock.acquire()
    return lock.acquire(timeout=max(0.0, policy.deadline_s - rt.monotonic()))


def _exchange(
    policy: ExchangePolicy,
    link: HeartbeatState,
    send: Callable[[], None],
    wait: Callable[[float], MessageSample | None],
    timeout: float,
    on_miss: Callable[[int], None] | None,
) -> tuple[ParameterExchange, MessageSample | None]:
    sent, reason = 0, ExchangeReason.NO_ECHO
    while sent < policy.attempts:
        if policy.stop_on_silence and link.autopilot_silent:
            reason = ExchangeReason.LINK_LOST
            break
        if policy.deadline_s is not None and rt.monotonic() >= policy.deadline_s:
            reason = ExchangeReason.DEADLINE
            break
        send()
        sent += 1
        deadline = rt.monotonic() + timeout
        if policy.deadline_s is not None:
            deadline = min(deadline, policy.deadline_s)
        sample = wait(deadline)
        if sample is not None:
            return ParameterExchange(
                ExchangeOutcome.CONFIRMED, None, sent, float(sample.message.param_value),
            ), sample
        if on_miss is not None:
            on_miss(sent)
    outcome = ExchangeOutcome.UNCONFIRMED if sent else ExchangeOutcome.UNATTEMPTED
    return ParameterExchange(outcome, reason, sent), None


__all__ = ["run_exchange"]
