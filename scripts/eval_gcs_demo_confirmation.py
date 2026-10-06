"""Round-correlated operator confirmation over `/ws/telemetry`."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Callable

from gcs.backend.task_confirm_uid import LEGACY_ROUND_UID, parse_round_uid

from scripts.eval_gcs_demo_models import finite_number, positive_int
from scripts.eval_gcs_demo_ports import (
    ClockPort,
    EventStreamPort,
    JsonApiPort,
    JsonValue,
)
from scripts.eval_gcs_demo_process import SystemClock


@dataclass(frozen=True)
class ConfirmationRequest:
    sys_id: int
    local_task_id: int
    round_uid: str


@dataclass(frozen=True)
class PendingConfirmation:
    request: ConfirmationRequest
    observed_monotonic_s: float
    observed_unix_s: float


class ConfirmationTimeout(TimeoutError):
    """Preserve request and approval state when the demo misses its deadline."""

    def __init__(
        self,
        expected_sys_ids: set[int],
        seen: set[tuple[int, int, str]],
        approvals: list[dict[str, JsonValue]],
        *,
        missing_snap_sysids: list[int] | None = None,
    ) -> None:
        self.expected_sys_ids = set(expected_sys_ids)
        self.seen = set(seen)
        self.requests = tuple(ConfirmationRequest(*key) for key in sorted(seen))
        self.approvals = [dict(row) for row in approvals]
        self.missing_snap_sysids = (
            None if missing_snap_sysids is None else tuple(missing_snap_sysids)
        )
        observed = {request.sys_id for request in self.requests}
        approved = {int(row["sys_id"]) for row in self.approvals}
        message = (
            "timed out waiting for operator-confirmed final-approach SNAPs; "
            f"missing confirmation request sysids {sorted(expected_sys_ids - observed)}; "
            f"request seen but unapproved sysids {sorted(observed - approved)}; "
            f"approved sysids {sorted(approved)}"
        )
        if missing_snap_sysids is not None:
            message += f"; missing SNAP sysids {missing_snap_sysids}"
        super().__init__(message)


def parse_task_confirm_event(raw: bytes | str) -> ConfirmationRequest | None:
    if isinstance(raw, bytes):
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValueError("telemetry event is not UTF-8") from error
    elif type(raw) is str:
        text = raw
    else:
        raise TypeError("telemetry event must be bytes or str")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid telemetry JSON: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("telemetry event must be a JSON object")
    if payload.get("type") != "task_confirm_request":
        return None
    sys_id = positive_int("confirmation sys_id", payload.get("sys_id"))
    task_id = positive_int("confirmation task_id", payload.get("task_id"))
    uid = payload.get("round_uid")
    if type(uid) is not str:
        raise ValueError("confirmation round_uid must be an exact string")
    if uid == LEGACY_ROUND_UID:
        raise ValueError("legacy confirmation round_uid cannot certify the demo")
    parse_round_uid(uid)
    return ConfirmationRequest(sys_id, task_id, uid)


def _validate_expected(expected_sys_ids: set[int]) -> None:
    if not expected_sys_ids:
        raise ValueError("confirmation workflow requires expected vehicles")
    for sys_id in expected_sys_ids:
        positive_int("expected confirmation sys_id", sys_id)


def _response_body(response: object) -> tuple[int, dict[str, JsonValue]]:
    if (
        type(response) is not tuple
        or len(response) != 2
        or type(response[0]) is not int
        or not isinstance(response[1], dict)
    ):
        raise ValueError("task confirmation POST returned an invalid response")
    return response[0], response[1]


def run_confirmation_workflow(
    *,
    event_stream: EventStreamPort,
    api: JsonApiPort,
    expected_sys_ids: set[int],
    trigger: Callable[[], None],
    stop_when: Callable[[], bool],
    requested_delay_s: float,
    timeout_s: float,
    clock: ClockPort | None = None,
    on_request_observed: Callable[[ConfirmationRequest], None] | None = None,
    before_approve: Callable[[ConfirmationRequest], None] | None = None,
    on_approval_attempted: Callable[[ConfirmationRequest], None] | None = None,
    on_approval_accepted: Callable[[dict[str, JsonValue]], None] | None = None,
) -> list[dict[str, JsonValue]]:
    """Subscribe, trigger, dedupe by round, delay, and echo exact identity."""
    _validate_expected(expected_sys_ids)
    delay = finite_number("requested confirmation delay", requested_delay_s)
    timeout = finite_number("confirmation timeout", timeout_s)
    timer = clock or SystemClock()
    deadline = timer.monotonic() + timeout
    pending: dict[tuple[int, int, str], PendingConfirmation] = {}
    seen: set[tuple[int, int, str]] = set()
    approvals: list[dict[str, JsonValue]] = []
    with event_stream:
        trigger()
        while timer.monotonic() < deadline:
            try:
                raw = event_stream.receive(min(0.05, max(0.0, deadline - timer.monotonic())))
            except (TimeoutError, StopIteration):
                raw = None
            if raw is not None:
                request = parse_task_confirm_event(raw)
                if request is not None:
                    if request.sys_id not in expected_sys_ids:
                        raise ValueError(
                            f"unexpected confirmation vehicle {request.sys_id}"
                        )
                    key = (
                        request.sys_id,
                        request.local_task_id,
                        request.round_uid,
                    )
                    if key not in seen:
                        if on_request_observed is not None:
                            on_request_observed(request)
                        seen.add(key)
                        pending[key] = PendingConfirmation(
                            request,
                            timer.monotonic(),
                            timer.unix(),
                        )

            now = timer.monotonic()
            due = [
                item
                for item in pending.values()
                if now - item.observed_monotonic_s >= delay
            ]
            for item in due:
                request = item.request
                if before_approve is not None:
                    before_approve(request)
                payload: dict[str, JsonValue] = {
                    "sys_id": request.sys_id,
                    "task_id": request.local_task_id,
                    "is_confirmed": True,
                    "action": "approve",
                    "round_uid": request.round_uid,
                }
                if on_approval_attempted is not None:
                    on_approval_attempted(request)
                status, body = _response_body(
                    api.post_json("/api/control/task_confirm", payload)
                )
                if status != 200 or body.get("status") != "approved":
                    raise ValueError(f"task confirmation was not approved: {status}, {body}")
                approved_at = timer.monotonic()
                record: dict[str, JsonValue] = {
                    **payload,
                    "request_observed": True,
                    "response": "approved",
                    "request_observed_at_unix_s": item.observed_unix_s,
                    "approved_at_unix_s": timer.unix(),
                    "requested_delay_s": delay,
                    "actual_delay_s": approved_at - item.observed_monotonic_s,
                    "nav_before_approval": False,
                }
                approvals.append(record)
                if on_approval_accepted is not None:
                    on_approval_accepted(dict(record))
                del pending[
                    (request.sys_id, request.local_task_id, request.round_uid)
                ]
            if stop_when():
                return approvals
            timer.sleep(min(0.05, max(0.0, deadline - timer.monotonic())))
    raise ConfirmationTimeout(expected_sys_ids, seen, approvals)


__all__ = [
    "ConfirmationRequest",
    "ConfirmationTimeout",
    "PendingConfirmation",
    "parse_task_confirm_event",
    "run_confirmation_workflow",
]
