"""The send-and-wait loop of one parameter exchange under an :class:`ExchangePolicy`.

``docs/design/radio_budget.md``, todo 103: the deadline caps the lock wait and
every answer wait and is checked again before each send; with
``stop_on_silence`` the link is checked after taking the lock and before each
send. The answer cursor is taken after the lock, so an answer to an earlier
exchange never confirms this one. An exchange that sent nothing is
``unattempted``; one that sent without a matching answer is ``unconfirmed``,
with why it stopped: deadline, link lost, or the budget spent (no echo).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.runtime_time import LockPort
from navpy.core.ports.parameter_exchange import (
    ExchangeOutcome,
    ExchangePolicy,
    ExchangeReason,
    ParameterExchange,
)
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.message_store import MessageSample, MessageStore


@dataclass(frozen=True)
class ExchangeIo:
    """One exchange's wire side: ``send()`` sends once; ``matches`` picks its answer."""

    label: str
    send: Callable[[], None]
    matches: Callable[[object], bool]


def run_exchange(
    policy: ExchangePolicy,
    link: HeartbeatState,
    messages: MessageStore,
    io: ExchangeIo,
    timeout: float,
    *,
    lock: LockPort | None = None,
    log: Callable[[str], None] | None = None,
) -> tuple[ParameterExchange, MessageSample | None]:
    """With ``log``, a GCS exchange (``stop_on_silence``) logs every send and its outcome;
    any other logs its outcome when it took more than one send or failed."""
    if lock is not None and not _acquire(lock, policy):
        result: tuple[ParameterExchange, MessageSample | None] = (
            ParameterExchange(ExchangeOutcome.UNATTEMPTED, ExchangeReason.DEADLINE, 0), None)
    else:
        try:
            result = _exchange(policy, link, messages, io, timeout, log)
        finally:
            if lock is not None:
                lock.release()
    exchange = result[0]
    if log is not None and (policy.stop_on_silence or exchange.attempts > 1 or not exchange.confirmed):
        why = "" if exchange.reason is None else f" ({exchange.reason.value})"
        log(f"{io.label}: {exchange.outcome.value}{why} after {exchange.attempts}/{policy.attempts} sends")
    return result


def _acquire(lock: LockPort, policy: ExchangePolicy) -> bool:
    if policy.deadline_s is None:
        return lock.acquire()
    return lock.acquire(timeout=max(0.0, policy.deadline_s - rt.monotonic()))


def _stop_reason(policy: ExchangePolicy, link: HeartbeatState) -> ExchangeReason | None:
    if policy.deadline_s is not None and rt.monotonic() >= policy.deadline_s:
        return ExchangeReason.DEADLINE
    if policy.stop_on_silence and link.autopilot_silent:
        return ExchangeReason.LINK_LOST
    return None


def _exchange(
    policy: ExchangePolicy,
    link: HeartbeatState,
    messages: MessageStore,
    io: ExchangeIo,
    timeout: float,
    log: Callable[[str], None] | None,
) -> tuple[ParameterExchange, MessageSample | None]:
    # Taken under the caller's lock; an answer to an earlier send of this exchange still counts.
    cursor = messages.cursor()
    sent = 0
    reason = _stop_reason(policy, link)
    while reason is None and sent < policy.attempts:
        io.send()
        sent += 1
        deadline = rt.monotonic() + timeout
        if policy.deadline_s is not None:
            deadline = min(deadline, policy.deadline_s)
        sample = messages.wait_after("PARAM_VALUE", cursor, io.matches, deadline=deadline)
        if log is not None and (policy.stop_on_silence or sample is None):
            log(f"{io.label} send {sent}/{policy.attempts}: {'echo' if sample is not None else 'no echo'}")
        if sample is not None:
            return ParameterExchange(
                ExchangeOutcome.CONFIRMED, None, sent, float(sample.message.param_value),
            ), sample
        # Checked after every miss, the last one too, so a cut wait is never "no echo".
        reason = _stop_reason(policy, link)
    outcome = ExchangeOutcome.UNCONFIRMED if sent else ExchangeOutcome.UNATTEMPTED
    return ParameterExchange(outcome, reason or ExchangeReason.NO_ECHO, sent), None


__all__ = ["ExchangeIo", "run_exchange"]
