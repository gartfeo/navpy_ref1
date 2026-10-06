"""NavPy, mission, parameter, and vehicle-reported build identities."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Protocol, Sequence

from scripts.eval_certificate_process import run_text
from scripts.eval_certificate_values import canonical_digest, failed_reasons


MISSION_IDENTITY_FIELDS: tuple[str, ...] = (
    "seq",
    "command",
    "frame",
    "lat_deg",
    "lon_deg",
    "mission_alt_m",
    "param1",
    "param2",
    "param3",
    "param4",
    "current",
    "autocontinue",
    "mission_type",
)

_MAV_CMD_REQUEST_MESSAGE = 512
_AUTOPILOT_VERSION_MSG_ID = 148


class AutopilotVersionMessage(Protocol):
    flight_sw_version: int | None
    board_version: int | None
    flight_custom_version: bytes | bytearray | Sequence[int] | None
    os_custom_version: bytes | bytearray | Sequence[int] | None


class AutopilotMavPort(Protocol):
    def command_long_send(self, *values: int | float) -> None: ...


class AutopilotVersionProbe(Protocol):
    target_system: int
    target_component: int
    mav: AutopilotMavPort

    def recv_match(
        self,
        *,
        type: str,
        blocking: bool,
        timeout: float,
    ) -> AutopilotVersionMessage | None: ...


def navpy_identity(worktree: Path) -> dict[str, Any]:
    """Identify the NavPy tree that produced a run and whether it was clean."""
    commit, commit_error = run_text(["git", "rev-parse", "HEAD"], cwd=worktree)
    branch, _ = run_text(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        cwd=worktree,
    )
    status, status_error = run_text(
        ["git", "status", "--porcelain"],
        cwd=worktree,
    )
    return {
        "worktree": str(worktree),
        "commit": commit,
        "branch": branch,
        "dirty": None if status is None else bool(status),
        "dirty_files": (
            None
            if status is None
            else [
                line.strip().split(" ", 1)[1].strip()
                for line in status.splitlines()
                if " " in line.strip()
            ]
        ),
        "unavailable": failed_reasons(
            commit=commit_error,
            status=status_error,
        ),
    }


def mission_identity(items: Iterable[Any]) -> dict[str, Any]:
    """Hash all execution-relevant fields downloaded from the vehicle."""
    rows = [
        [getattr(item, name, None) for name in MISSION_IDENTITY_FIELDS]
        for item in items
    ]
    return {"item_count": len(rows), "sha256": canonical_digest(rows)}


@dataclass(frozen=True)
class ParameterSnapshot:
    """A parameter download together with explicit completeness evidence."""

    values: Mapping[str, float]
    vehicle_param_count: int | None = None
    missing_indices: tuple[int, ...] = ()
    retried_indices: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.values, Mapping):
            raise TypeError("values must be a parameter mapping")
        for name, value in self.values.items():
            if not isinstance(name, str) or not name:
                raise ValueError(f"parameter name must be non-empty: {name!r}")
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"parameter {name} must be numeric, not {value!r}")
            if not math.isfinite(float(value)):
                raise ValueError(f"parameter {name} must be finite, not {value!r}")
        _validate_optional_count(self.vehicle_param_count, "vehicle_param_count")
        _validate_count(self.retried_indices, "retried_indices")
        for index in self.missing_indices:
            _validate_count(index, "missing index")
        if len(set(self.missing_indices)) != len(self.missing_indices):
            raise ValueError("missing_indices must not contain duplicates")

    @property
    def received_count(self) -> int:
        return len(self.values)

    @property
    def complete(self) -> bool:
        return (
            self.vehicle_param_count is not None
            and not self.missing_indices
            and self.received_count >= self.vehicle_param_count
        )


def _validate_count(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer, not {value!r}")
    if value < 0:
        raise ValueError(f"{name} must be non-negative, not {value}")


def _validate_optional_count(value: Any, name: str) -> None:
    if value is not None:
        _validate_count(value, name)


def parameter_snapshot_identity(snapshot: ParameterSnapshot) -> dict[str, Any]:
    """Hash a full, explicitly complete-or-incomplete parameter snapshot."""
    if not isinstance(snapshot, ParameterSnapshot):
        raise TypeError("snapshot must be a ParameterSnapshot")
    ordered = {
        name: float(value)
        for name, value in sorted(snapshot.values.items())
    }
    payload = {
        "values": ordered,
        "complete": snapshot.complete,
        "vehicle_param_count": snapshot.vehicle_param_count,
        "missing_indices": list(snapshot.missing_indices),
    }
    return {
        "parameter_count": snapshot.received_count,
        "vehicle_param_count": snapshot.vehicle_param_count,
        "complete": snapshot.complete,
        "missing_index_count": len(snapshot.missing_indices),
        "missing_indices": list(snapshot.missing_indices),
        "retried_indices": snapshot.retried_indices,
        "sha256": canonical_digest(payload),
    }


def autopilot_identity(
    master: AutopilotVersionProbe,
    *,
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    """Read the build identity reported by the running vehicle."""
    from scripts.eval_certificate_values import require_finite

    timeout = require_finite(
        timeout_s,
        name="timeout_s",
        minimum=0.0,
        minimum_inclusive=False,
    )
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            _MAV_CMD_REQUEST_MESSAGE,
            0,
            _AUTOPILOT_VERSION_MSG_ID,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    except (AttributeError, OSError) as exc:
        return {
            "available": False,
            "reason": f"could not request AUTOPILOT_VERSION ({exc})",
        }
    try:
        message = master.recv_match(
            type="AUTOPILOT_VERSION",
            blocking=True,
            timeout=timeout,
        )
    except OSError as exc:
        return {
            "available": False,
            "reason": f"link error reading AUTOPILOT_VERSION ({exc})",
        }
    if message is None:
        return {
            "available": False,
            "reason": "vehicle did not report AUTOPILOT_VERSION",
        }
    return {
        "available": True,
        "flight_sw_version": getattr(message, "flight_sw_version", None),
        "board_version": getattr(message, "board_version", None),
        "flight_custom_version": _hexify(
            getattr(message, "flight_custom_version", None)
        ),
        "os_custom_version": _hexify(
            getattr(message, "os_custom_version", None)
        ),
    }


def _hexify(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).hex()
    if isinstance(value, (list, tuple)):
        output: list[int] = []
        for part in value:
            if isinstance(part, bool) or not isinstance(part, int) or not 0 <= part <= 255:
                return None
            output.append(part)
        return bytes(output).hex()
    return str(value)


__all__ = [
    "MISSION_IDENTITY_FIELDS",
    "ParameterSnapshot",
    "autopilot_identity",
    "mission_identity",
    "navpy_identity",
    "parameter_snapshot_identity",
]
