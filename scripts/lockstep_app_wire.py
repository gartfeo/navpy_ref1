"""What the app relay (``lockstep_app_relay``) carries, as data.

One HTTP request becomes one ``{"kind": "http", ...}`` payload
(:func:`relay_payload`); one answer from the run becomes one HTTP status and
JSON body (:func:`http_reply`). :class:`TelemetryState` keeps the latest
vehicle state from the backend's broadcasts for a page that opens later.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from typing import Any

from navpy.sim.lockstep.part_wire_codec import dumps

DEADLINE_HEADER = b"x-navpy-deadline-s"
CLIENT_ID_HEADER = b"x-gcs-request-id"
#: HTTP status of each coordinator or backend outcome that is not a response.
CANCELLED_STATUS, DEADLINE_STATUS, ENDED_STATUS = 499, 504, 503


def relay_payload(scope: dict, body: bytes) -> dict[str, Any]:
    """The run's payload for one request; ``ValueError`` when it cannot enter the run."""
    headers = dict(scope.get("headers", []))
    # As sent, still percent-encoded: the backend decodes it once.
    raw = (scope.get("raw_path") or scope["path"].encode("utf-8")).decode("ascii")
    query = scope.get("query_string", b"").decode("ascii")
    payload: dict[str, Any] = {"kind": "http", "method": scope["method"],
                               "target": raw + (f"?{query}" if query else "")}
    if body:
        payload["json"] = json.loads(body)
    if DEADLINE_HEADER in headers:
        deadline = float(headers[DEADLINE_HEADER])
        if not (math.isfinite(deadline) and deadline > 0):
            raise ValueError(f"deadline {deadline}")
        payload["deadline_s"] = deadline
    dumps(payload)  # the journal's encoding: no NaN or infinity (``1e400``)
    return payload


def http_reply(answer: object) -> tuple[int, bytes]:
    """The HTTP status and JSON body of one answer from the run."""
    if isinstance(answer, dict) and isinstance(answer.get("status"), int) \
            and isinstance(answer.get("body"), str):
        return answer["status"], answer["body"].encode("utf-8")
    if answer == {"cancelled": True}:
        return CANCELLED_STATUS, b'{"detail":"cancelled"}'
    if answer == {"deadline": True}:
        return DEADLINE_STATUS, b'{"detail":"sim-time deadline passed"}'
    return 502, json.dumps({"detail": f"unexpected answer {answer!r}"}).encode("utf-8")


async def send_json(send: Callable, status: int, body: bytes) -> None:
    await send({"type": "http.response.start", "status": status,
                "headers": [(b"content-type", b"application/json"),
                            (b"content-length", str(len(body)).encode())]})
    await send({"type": "http.response.body", "body": body})


class TelemetryState:
    """The latest vehicle state, merged as ``telemetryStore.js`` merges broadcasts."""

    def __init__(self) -> None:
        self._vehicles: dict[Any, dict] = {}
        self._last: dict[str, Any] | None = None

    def observe(self, message: dict[str, Any]) -> None:
        if message.get("type") != "telemetry" or not isinstance(message.get("vehicles"), list):
            return
        for vehicle in message["vehicles"]:
            self._vehicles[vehicle.get("sys_id")] = {**self._vehicles.get(vehicle.get("sys_id"), {}),
                                                     **vehicle}
        if isinstance(message.get("active_ids"), list):
            active = set(message["active_ids"])
            self._vehicles = {key: value for key, value in self._vehicles.items() if key in active}
        self._last = message

    def snapshot(self) -> str | None:
        if self._last is None:
            return None
        return json.dumps({**self._last, "vehicles": list(self._vehicles.values())})


__all__ = ["CANCELLED_STATUS", "CLIENT_ID_HEADER", "DEADLINE_HEADER", "DEADLINE_STATUS",
           "ENDED_STATUS", "TelemetryState", "http_reply", "relay_payload", "send_json"]
