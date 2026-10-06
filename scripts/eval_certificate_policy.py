"""Certificate admissibility, consistency, and no-selection aggregation."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from scripts.eval_certificate_statistics import summarize
from scripts.eval_certificate_values import finite_number


CERTIFICATE_RATE_TOLERANCE = 0.10
_HEX_DIGITS = frozenset("0123456789abcdefABCDEF")

CERTIFICATE_IDENTITY_KEYS: tuple[str, ...] = (
    "identity_navpy_commit",
    "identity_mission_sha",
    "identity_parameters_sha",
    "identity_autopilot",
)

CERTIFICATE_METRIC_KEYS: tuple[str, ...] = (
    "snap_dist_3d_m",
    "coordinate_dist_3d_m",
    "snap_minus_coordinate_m",
    "measured_clock_rate",
    "launch_measured_clock_rate",
    "obs_source_gap_p99_ms",
    "cmd_source_gap_p99_ms",
    "cmd_wall_gap_p99_ms",
    "cmd_fresh_count",
    "cmd_held_count",
    "cmd_age_p95_ms",
    "cmd_age_max_ms",
)


def certificate_rate_error(rate: Any, requested: Any) -> str | None:
    """Return why measured source/wall clock rate is inadmissible."""
    wanted = finite_number(requested)
    if wanted is None or wanted <= 0:
        return "no requested speedup to check the measured rate against"
    measured = finite_number(rate)
    if measured is None:
        return "engagement clock rate was not measured"
    low = wanted * (1.0 - CERTIFICATE_RATE_TOLERANCE)
    high = wanted * (1.0 + CERTIFICATE_RATE_TOLERANCE)
    if not low <= measured <= high:
        return (
            f"engagement clock rate {measured:.2f}x is outside "
            f"{low:.2f}-{high:.2f}x for a {wanted:g}x certificate"
        )
    return None


def certificate_identity_errors(identity: Mapping[str, Any] | None) -> list[str]:
    """Return why a run identity cannot support a certificate."""
    if not identity:
        return ["no identity was gathered for this run"]
    errors: list[str] = []
    navpy = identity.get("navpy") or {}
    if not isinstance(navpy, Mapping):
        errors.append("NavPy identity is malformed")
    else:
        dirty = navpy.get("dirty")
        if dirty is None:
            errors.append("NavPy working-tree state could not be determined")
        elif dirty is not False:
            errors.append("NavPy working tree was dirty or its dirty flag was invalid")
        if not _nonempty_text(navpy.get("commit")):
            errors.append("NavPy commit could not be determined")
    mission = identity.get("mission") or {}
    if not isinstance(mission, Mapping):
        errors.append("mission identity is malformed")
    else:
        item_count = mission.get("item_count")
        if (
            isinstance(item_count, bool)
            or not isinstance(item_count, int)
            or item_count <= 0
            or not _nonempty_text(mission.get("sha256"))
        ):
            errors.append("mission identity is missing an item count or digest")
    parameters = identity.get("parameters") or {}
    if not isinstance(parameters, Mapping):
        errors.append("parameter snapshot identity is malformed")
    else:
        if parameters.get("complete") is not True:
            errors.append(
                "parameter snapshot incomplete "
                f"({parameters.get('parameter_count')} of "
                f"{parameters.get('vehicle_param_count')})"
            )
        if not _nonempty_text(parameters.get("sha256")):
            errors.append("parameter snapshot digest is missing")
    autopilot = identity.get("autopilot") or {}
    if not isinstance(autopilot, Mapping):
        errors.append("firmware identity is malformed")
    elif autopilot.get("available") is not True:
        errors.append(
            "firmware identity unavailable "
            f"({autopilot.get('reason') or 'no AUTOPILOT_VERSION'})"
        )
    elif not (
        _valid_flight_sw_version(autopilot.get("flight_sw_version"))
        and _valid_flight_custom_version(autopilot.get("flight_custom_version"))
    ):
        errors.append("firmware build identifier is missing")
    return errors


def _nonempty_text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _valid_flight_sw_version(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, int)
        and 0 < value <= 0xFFFFFFFF
    )


def _valid_flight_custom_version(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 16
        and all(character in _HEX_DIGITS for character in value)
        and any(character != "0" for character in value)
    )


def certificate_consistency_errors(
    rows: Sequence[Mapping[str, Any]],
) -> list[str]:
    """Return ways repetitions do not identify one build/mission/config."""
    errors: list[str] = []
    for key in CERTIFICATE_IDENTITY_KEYS:
        seen = {str(row.get(key, "")) for row in rows}
        if "" in seen:
            errors.append(f"{key} is missing from one or more repetitions")
        if len(seen) > 1:
            errors.append(
                f"{key} changed between repetitions: "
                + ", ".join(sorted(repr(value) for value in seen))
            )
    errors.extend(_label_consistency_errors(rows))
    return errors


def _label_consistency_errors(rows: Sequence[Mapping[str, Any]]) -> list[str]:
    errors: list[str] = []
    indices = [row.get("index") for row in rows]
    if any(isinstance(value, bool) or not isinstance(value, int) for value in indices):
        errors.append("certificate index labels must be integers")
    elif len(set(indices)) > 1:
        errors.append("certificate index changed between repetitions")
    repetitions = [row.get("repetition") for row in rows]
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value <= 0
        for value in repetitions
    ):
        errors.append("repetition labels must be positive integers")
    elif len(set(repetitions)) != len(repetitions):
        errors.append("repetition labels must be unique")
    elif sorted(repetitions) != list(range(1, len(repetitions) + 1)):
        errors.append("repetition labels must be contiguous from 1")
    return errors


def _row_evidence_errors(row: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if type(row.get("passed")) is not bool:
        errors.append("run passed result must be an exact boolean")
    if "certificate_invalid_reason" not in row:
        errors.append("certificate admissibility result is missing")
    else:
        invalid_reason = row.get("certificate_invalid_reason")
        if not isinstance(invalid_reason, str):
            errors.append("certificate admissibility result must be a string")
        elif invalid_reason != "":
            errors.append(invalid_reason)
    if "error" not in row:
        errors.append("run error result is missing")
    else:
        run_error = row.get("error")
        if not isinstance(run_error, str):
            errors.append("run error result must be a string")
        elif run_error != "":
            errors.append(f"run error: {run_error}")
    if finite_number(row.get("dist_3d_m")) is None:
        errors.append("SNAP distance is missing or non-finite")
    if finite_number(row.get("coordinate_dist_3d_m")) is None:
        errors.append("coordinate distance is missing or non-finite")
    requested = finite_number(row.get("speedup"))
    if requested is None or requested <= 0:
        errors.append("requested speedup is missing, non-finite, or non-positive")
    rate_error = certificate_rate_error(row.get("measured_clock_rate"), requested)
    if rate_error is not None:
        errors.append(rate_error)
    for key in CERTIFICATE_IDENTITY_KEYS:
        if not _nonempty_text(row.get(key)):
            errors.append(f"{key} is missing or malformed")
    return errors


def certificate_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Aggregate every repetition without selection or truthy coercion."""
    per_run: list[dict[str, Any]] = []
    for row in rows:
        snap = finite_number(row.get("dist_3d_m"))
        coordinate = finite_number(row.get("coordinate_dist_3d_m"))
        per_run.append(
            {
                "index": row.get("index"),
                "repetition": row.get("repetition"),
                "name": row.get("name"),
                "passed": row.get("passed") is True,
                "snap_dist_3d_m": snap,
                "coordinate_dist_3d_m": coordinate,
                "snap_minus_coordinate_m": (
                    abs(snap - coordinate)
                    if snap is not None and coordinate is not None
                    else None
                ),
                "requested_speedup": row.get("speedup"),
                "launch_measured_clock_rate": finite_number(
                    row.get("launch_measured_clock_rate")
                ),
                "measured_clock_rate": finite_number(
                    row.get("measured_clock_rate")
                ),
                "measured_clock_error": row.get("measured_clock_error") or "",
                "obs_source_gap_p99_ms": finite_number(
                    row.get("obs_source_gap_p99_ms")
                ),
                "cmd_source_gap_p99_ms": finite_number(
                    row.get("cmd_source_gap_p99_ms")
                ),
                "cmd_wall_gap_p99_ms": finite_number(
                    row.get("cmd_wall_gap_p99_ms")
                ),
                "cmd_fresh_count": finite_number(row.get("cmd_fresh_count")),
                "cmd_held_count": finite_number(row.get("cmd_held_count")),
                "cmd_age_p95_ms": finite_number(row.get("cmd_age_p95_ms")),
                "cmd_age_max_ms": finite_number(row.get("cmd_age_max_ms")),
                "invalid_reason": row.get("certificate_invalid_reason") or "",
                "identity_navpy_commit": row.get("identity_navpy_commit") or "",
                "identity_mission_sha": row.get("identity_mission_sha") or "",
                "identity_parameters_sha": row.get("identity_parameters_sha") or "",
                "identity_autopilot": row.get("identity_autopilot") or "",
                "error": row.get("error") or "",
                "evidence_errors": _row_evidence_errors(row),
            }
        )
    metrics: dict[str, Any] = {}
    for key in CERTIFICATE_METRIC_KEYS:
        stats = summarize([run[key] for run in per_run if run[key] is not None])
        metrics[key] = stats.as_dict() if stats is not None else None
    invalid_runs = [run for run in per_run if run["evidence_errors"]]
    consistency_errors = certificate_consistency_errors(rows)
    return {
        "runs": len(per_run),
        "passed_runs": sum(1 for run in per_run if run["passed"]),
        "failed_runs": sum(1 for run in per_run if not run["passed"]),
        "runs_without_snap": sum(
            1 for run in per_run if run["snap_dist_3d_m"] is None
        ),
        "runs_without_coordinate": sum(
            1 for run in per_run if run["coordinate_dist_3d_m"] is None
        ),
        "runs_without_measured_rate": sum(
            1 for run in per_run if run["measured_clock_rate"] is None
        ),
        "invalid_runs": len(invalid_runs),
        "invalid_reasons": [
            "; ".join(run["evidence_errors"])
            for run in invalid_runs
        ],
        "consistency_errors": consistency_errors,
        "valid": bool(
            per_run
            and not invalid_runs
            and not consistency_errors
            and all(run["passed"] for run in per_run)
        ),
        "metrics": metrics,
        "per_run": per_run,
    }


__all__ = [
    "CERTIFICATE_IDENTITY_KEYS",
    "CERTIFICATE_METRIC_KEYS",
    "CERTIFICATE_RATE_TOLERANCE",
    "certificate_consistency_errors",
    "certificate_identity_errors",
    "certificate_rate_error",
    "certificate_summary",
]
