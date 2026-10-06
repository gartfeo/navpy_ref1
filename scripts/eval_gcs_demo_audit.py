"""Strict operator-approval and navigation-speedup audit evidence."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

from gcs.backend.task_confirm_uid import LEGACY_ROUND_UID, parse_round_uid

from scripts.eval_gcs_demo_models import finite_number, positive_int
from scripts.eval_gcs_demo_ports import JsonValue


_APPROVAL_FIELDS = {
    "sys_id",
    "task_id",
    "is_confirmed",
    "action",
    "round_uid",
    "request_observed",
    "response",
    "request_observed_at_unix_s",
    "approved_at_unix_s",
    "requested_delay_s",
    "actual_delay_s",
    "nav_before_approval",
}
_NUMBER = r"-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_NUMBER_RE = re.compile(_NUMBER)


@dataclass(frozen=True)
class _MarkerValue:
    at: int
    token: str
    value: float | None


def _marker_values(text: str, marker: str) -> list[_MarkerValue]:
    evidence: list[_MarkerValue] = []
    for match in re.finditer(re.escape(marker), text):
        token_match = re.match(r"[^\s,;]+", text[match.end():])
        token = "" if token_match is None else token_match.group(0)
        valid = _NUMBER_RE.fullmatch(token)
        value = None if valid is None else float(token)
        if value is not None and not math.isfinite(value):
            value = None
        evidence.append(_MarkerValue(match.start(), token, value))
    return evidence


def validate_approval_records(raw: object) -> list[dict[str, JsonValue]]:
    if not isinstance(raw, list):
        raise ValueError("approval records must be a list")
    records: list[dict[str, JsonValue]] = []
    seen: set[tuple[int, int, str]] = set()
    for index, item in enumerate(raw):
        if not isinstance(item, dict) or set(item) != _APPROVAL_FIELDS:
            raise ValueError(f"approval record {index} has an invalid schema")
        sys_id = positive_int("approval sys_id", item["sys_id"])
        task_id = positive_int("approval task_id", item["task_id"])
        uid = item["round_uid"]
        if type(uid) is not str or uid == LEGACY_ROUND_UID:
            raise ValueError(f"approval record {index} has an invalid round_uid")
        parse_round_uid(uid)
        key = sys_id, task_id, uid
        if key in seen:
            raise ValueError(f"duplicate approval round {key}")
        seen.add(key)
        if (
            item["is_confirmed"] is not True
            or item["action"] != "approve"
            or item["request_observed"] is not True
            or item["response"] != "approved"
            or item["nav_before_approval"] is not False
        ):
            raise ValueError(f"approval record {index} is not an audited approval")
        observed = finite_number(
            "approval request_observed_at_unix_s",
            item["request_observed_at_unix_s"],
        )
        approved = finite_number(
            "approval approved_at_unix_s",
            item["approved_at_unix_s"],
        )
        requested = finite_number(
            "approval requested_delay_s", item["requested_delay_s"]
        )
        actual = finite_number("approval actual_delay_s", item["actual_delay_s"])
        if actual < requested:
            raise ValueError(f"approval record {index} was posted before its delay")
        unix_elapsed = approved - observed
        if unix_elapsed < requested or abs(unix_elapsed - actual) > 0.10:
            raise ValueError(f"approval record {index} has inconsistent timestamps")
        records.append(item)
    return records


def load_approval_records(path: Path) -> list[dict[str, JsonValue]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid approval artifact {path}: {error}") from error
    if not isinstance(payload, dict) or set(payload) != {"approvals"}:
        raise ValueError("approval artifact must contain only approvals")
    return validate_approval_records(payload["approvals"])


def navigation_speedup_errors(
    navigation_text: str,
    *,
    requested: float,
    launch_speed: float,
) -> list[str]:
    requested_value = finite_number("requested navigation speedup", requested)
    launch_value = finite_number("launch speedup", launch_speed)
    if launch_value <= 0.0:
        raise ValueError("launch speedup must be positive")
    args = _marker_values(navigation_text, "nav_sim_speedup=")
    writes = _marker_values(navigation_text, "SIM_SPEEDUP=")
    snap_at = navigation_text.find("SNAP(VISION-NAV")
    errors: list[str] = []
    if requested_value == 0.0:
        if args:
            errors.append("disabled navigation speedup unexpectedly supplied -gsu")
        if writes:
            errors.append(
                "unexpected navigation SIM_SPEEDUP writes while GSU is disabled: "
                f"{[item.token for item in writes]}"
            )
        return errors
    malformed_args = [item.token for item in args if item.value is None]
    malformed_writes = [item.token for item in writes if item.value is None]
    if malformed_args:
        errors.append(f"malformed -gsu argument evidence: {malformed_args}")
    if malformed_writes:
        errors.append(f"malformed SIM_SPEEDUP write evidence: {malformed_writes}")
    matching_args = [
        item.at
        for item in args
        if item.value is not None
        and math.isclose(item.value, requested_value, abs_tol=1e-9)
    ]
    if len(args) != 1 or len(matching_args) != 1:
        errors.append("missing or duplicate requested -gsu argument evidence")
    if len(writes) != 2:
        errors.append("positive navigation speedup requires exactly apply and restore writes")
        return errors
    apply, restore = writes
    if apply.value is None or not math.isclose(
        apply.value,
        requested_value,
        abs_tol=1e-9,
    ):
        errors.append(
            f"navigation speedup applied {apply.token!r} != requested "
            f"{requested_value}"
        )
    if restore.value is None or not math.isclose(
        restore.value,
        launch_value,
        abs_tol=1e-9,
    ):
        errors.append(
            f"navigation speedup restored {restore.token!r} != launch "
            f"{launch_value}"
        )
    if snap_at < 0:
        errors.append("missing final-approach SNAP after navigation speedup restore")
    elif matching_args and not matching_args[0] < apply.at < restore.at < snap_at:
        errors.append("navigation speedup evidence is not argument -> apply -> restore -> SNAP")
    return errors


__all__ = [
    "navigation_speedup_errors",
    "load_approval_records",
    "validate_approval_records",
]
