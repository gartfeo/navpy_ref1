"""One parameter read or write exchange: its budget and its outcome.

``docs/design/radio_budget.md``, todo 103. A caller that must not wait on a
dead link (the GCS routes) passes an :class:`ExchangePolicy`: an attempt
budget, an absolute deadline shared by every parameter of its request, and
an end when the autopilot has sent no frame for its heartbeat timeout. The
default policy is the plain protocol: :data:`PARAM_PROTOCOL_ATTEMPTS`, no
deadline, no link check. Mission transfers run their steps under the same
policy (todo 105); ``cancelled`` ends an exchange once a stop is requested.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum

from navpy.core.ports.vehicle_control_interfaces import PARAM_PROTOCOL_ATTEMPTS


@dataclass(frozen=True)
class ExchangePolicy:
    attempts: int = PARAM_PROTOCOL_ATTEMPTS
    #: ``runtime_time.monotonic()`` after which nothing more is sent or waited for.
    deadline_s: float | None = None
    #: End the exchange when the autopilot has been silent past its heartbeat timeout.
    stop_on_silence: bool = False
    #: End the exchange once this is true (a NavPy stop request).
    cancelled: Callable[[], bool] | None = None


def with_cancel(policy: ExchangePolicy, cancelled: Callable[[], bool] | None) -> ExchangePolicy:
    """``policy`` that also ends once ``cancelled`` is true."""
    if cancelled is None:
        return policy
    own = policy.cancelled
    if own is None:
        return replace(policy, cancelled=cancelled)
    return replace(policy, cancelled=lambda: own() or cancelled())


class ExchangeOutcome(str, Enum):
    #: A matching answer came.
    CONFIRMED = "confirmed"
    #: Sent, no matching answer: a write may still be stored.
    UNCONFIRMED = "unconfirmed"
    #: Nothing was sent.
    UNATTEMPTED = "unattempted"


class ExchangeReason(str, Enum):
    NO_ECHO = "no_echo"
    LINK_LOST = "link_lost"
    DEADLINE = "deadline"
    INVALID = "invalid"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class ParameterExchange:
    outcome: ExchangeOutcome
    #: Why it is not confirmed; ``None`` when it is.
    reason: ExchangeReason | None
    #: Sends made.
    attempts: int
    #: The answer's value, when confirmed.
    value: float | None = None

    @property
    def confirmed(self) -> bool:
        return self.outcome is ExchangeOutcome.CONFIRMED


__all__ = ["ExchangeOutcome", "ExchangePolicy", "ExchangeReason", "ParameterExchange", "with_cancel"]
