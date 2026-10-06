"""Vehicle parameter control and certificate identity capture."""

from __future__ import annotations

import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from pymavlink import mavutil

from scripts import eval_certificate as cert

from eval_navigation_models import MissionItem
from eval_navigation_telemetry import message_from_target


def set_param(
    master: Any,
    name: str,
    value: float,
    param_type: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    *,
    timeout_s: float = 5.0,
) -> bool:
    """Set one parameter and require an echoed value within tolerance."""
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        param_type,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is None or not message_from_target(
            message, master.target_system
        ):
            continue
        if _param_name(message) != name:
            continue
        try:
            echoed = float(getattr(message, "param_value"))
        except (TypeError, ValueError):
            return False
        tolerance = max(0.01, abs(float(value)) * 0.01)
        return abs(echoed - float(value)) <= tolerance
    return False


def read_param(
    master: Any,
    name: str,
    *,
    timeout_s: float = 5.0,
) -> Any | None:
    """Read one named parameter from the selected vehicle."""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is None or not message_from_target(
            message, master.target_system
        ):
            continue
        if _param_name(message) != name:
            continue
        try:
            return float(getattr(message, "param_value"))
        except (TypeError, ValueError):
            return getattr(message, "param_value", None)
    return None


def _param_name(message: Any) -> str:
    raw = getattr(message, "param_id", "")
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="ignore")
    return str(raw).strip("\x00")


def param_value_row(
    message: Any,
) -> tuple[str, float, int | None, int | None] | None:
    """Return name, value, index, and count from a usable PARAM_VALUE."""
    name = _param_name(message)
    if not name:
        return None
    try:
        value = float(getattr(message, "param_value"))
    except (TypeError, ValueError):
        return None
    raw_index = getattr(message, "param_index", None)
    index = (
        int(raw_index)
        if isinstance(raw_index, (int, float))
        and not isinstance(raw_index, bool)
        and 0 <= raw_index < 0xFFFF
        else None
    )
    raw_count = getattr(message, "param_count", None)
    count = (
        int(raw_count)
        if isinstance(raw_count, (int, float))
        and not isinstance(raw_count, bool)
        and raw_count > 0
        else None
    )
    return name, value, index, count


def download_all_params(
    master: Any,
    *,
    timeout_s: float = 60.0,
    idle_timeout_s: float = 5.0,
    retry_timeout_s: float = 10.0,
) -> cert.ParameterSnapshot:
    """Download and repair a complete parameter snapshot for certification."""
    master.mav.param_request_list_send(
        master.target_system, master.target_component
    )
    parameters: dict[str, float] = {}
    seen_indices: set[int] = set()
    expected: int | None = None
    last_seen = time.time()

    def take(message: Any) -> None:
        nonlocal expected, last_seen
        row = param_value_row(message)
        if row is None:
            return
        name, value, index, count = row
        parameters[name] = value
        if index is not None:
            seen_indices.add(index)
        if count is not None:
            expected = count
        last_seen = time.time()

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is None:
            if parameters and time.time() - last_seen >= idle_timeout_s:
                break
            continue
        if not message_from_target(message, master.target_system):
            continue
        take(message)
        if expected is not None and len(seen_indices) >= expected:
            break
    missing = missing_param_indices(expected, seen_indices)
    retried = len(missing)
    if missing:
        _repair_missing_params(
            master,
            missing,
            expected=expected,
            seen_indices=seen_indices,
            take=take,
            retry_timeout_s=retry_timeout_s,
        )
        missing = missing_param_indices(expected, seen_indices)
    return cert.ParameterSnapshot(
        values=parameters,
        vehicle_param_count=expected,
        missing_indices=missing,
        retried_indices=retried,
    )


def _repair_missing_params(
    master: Any,
    missing: tuple[int, ...],
    *,
    expected: int | None,
    seen_indices: set[int],
    take: Any,
    retry_timeout_s: float,
) -> None:
    for index in missing:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            b"",
            index,
        )
    deadline = time.time() + retry_timeout_s
    while time.time() < deadline and missing_param_indices(
        expected, seen_indices
    ):
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is not None and message_from_target(
            message, master.target_system
        ):
            take(message)


def missing_param_indices(
    expected: int | None,
    seen: set[int],
) -> tuple[int, ...]:
    """Return promised parameter indices not yet delivered."""
    if expected is None:
        return ()
    return tuple(index for index in range(expected) if index not in seen)


def certificate_identity(
    master: Any,
    *,
    mission: Sequence[MissionItem],
    sysid: int,
    enabled: bool,
    worktree: Path,
) -> dict[str, Any] | None:
    """Capture all build/configuration identities for a certificate run."""
    if not enabled:
        return None
    snapshot = download_all_params(master)
    return {
        "navpy": cert.navpy_identity(worktree),
        "mission": cert.mission_identity(mission),
        "parameters": cert.parameter_snapshot_identity(snapshot),
        "parameter_values": dict(sorted(snapshot.values.items())),
        "autopilot": cert.autopilot_identity(master),
        "ardupilot_source": cert.wsl_ardupilot_identity(),
        "eeprom": cert.eeprom_identity(sysid),
    }


def certificate_invalid_reason_for(
    certificate: bool,
    *,
    measured_rate: Any,
    requested_speedup: Any,
    identity: dict[str, Any] | None,
) -> str:
    """Return why a run is inadmissible to a certificate, if applicable."""
    if not certificate:
        return ""
    reasons = [
        reason
        for reason in (
            cert.certificate_rate_error(measured_rate, requested_speedup),
            *cert.certificate_identity_errors(identity),
        )
        if reason
    ]
    return "; ".join(reasons)


def identity_fingerprint(
    identity: dict[str, Any] | None,
) -> dict[str, str]:
    """Flatten cross-repetition identity values onto one CSV row."""
    if not identity:
        return {key: "" for key in cert.CERTIFICATE_IDENTITY_KEYS}
    navpy = identity.get("navpy") or {}
    autopilot = identity.get("autopilot") or {}
    return {
        "identity_navpy_commit": str(navpy.get("commit") or ""),
        "identity_mission_sha": str(
            (identity.get("mission") or {}).get("sha256") or ""
        ),
        "identity_parameters_sha": str(
            (identity.get("parameters") or {}).get("sha256") or ""
        ),
        "identity_autopilot": "|".join(
            str(autopilot.get(field) or "")
            for field in (
                "flight_sw_version",
                "flight_custom_version",
                "board_version",
            )
        ),
    }
