"""Mission-protocol waits and attempts (``docs/design/radio_budget.md``, todo 105).

As QGroundControl (``PlanManager.h``) and the mavlink.io mission protocol:
250 ms for an item, 1.5 s for a count, request or ACK, 1 + 5 retries. A
resend after a late reply is harmless: requests are idempotent and the
reply cursor is taken before the first send.
"""

from __future__ import annotations

from navpy.core.ports.parameter_exchange import ExchangePolicy

MISSION_ITEM_TIMEOUT_S = 0.25
MISSION_ACK_TIMEOUT_S = 1.5
MISSION_PROTOCOL_ATTEMPTS = 6
#: ArduPilot drops an upload after 8 s without an item
#: (``MissionItemProtocol.h`` ``upload_timeout_ms``).
UPLOAD_ABANDON_S = 8.0


def default_mission_policy() -> ExchangePolicy:
    """The plain protocol: no deadline, no link check."""
    return ExchangePolicy(attempts=MISSION_PROTOCOL_ATTEMPTS)


__all__ = [
    "MISSION_ACK_TIMEOUT_S",
    "MISSION_ITEM_TIMEOUT_S",
    "MISSION_PROTOCOL_ATTEMPTS",
    "UPLOAD_ABANDON_S",
    "default_mission_policy",
]
