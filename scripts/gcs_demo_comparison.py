"""Offline comparisons of recorded GCS evidence, never a flight certificate."""

from __future__ import annotations

import hashlib
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

from scripts.eval_gcs_demo_analysis import analyze_run
from scripts.eval_gcs_demo_audit import load_approval_records
from scripts.eval_gcs_demo_evidence import parse_final_approach_command_episodes
from scripts.eval_gcs_demo_models import GateLimits
from scripts.eval_gcs_demo_scenario import load_resolved_plan


LIMITATIONS = [
    "Recorded settings, resolved spots and command bounds are not all run inputs. "
    "Complete missions, EEPROM, firmware, dependencies, environment and fresh boot "
    "identity are not established by these artifacts.",
    "Issued TERMINAL_CMD records are sampled by the wall-cadence logger. Equal "
    "records do not establish equality of every executed command or trajectory.",
    "Per-run scoring limits are not cross-run tolerances. Numerical differences "
    "are reported without a tolerance waiver; their causes require investigation.",
    "Simulation approach evidence does not establish physical docking or receipt.",
]


def _json(path: Path) -> Any:
    def invalid_constant(token: str) -> None:
        raise ValueError(f"non-finite JSON value {token}")

    def number(token: str) -> float:
        value = float(token)
        if not math.isfinite(value):
            invalid_constant(token)
        return value

    def object_pairs(pairs: list[tuple[str, Any]]) -> dict:
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key {key}")
            result[key] = value
        return result

    return json.loads(path.read_text(encoding="utf-8"), parse_constant=invalid_constant,
                      parse_float=number, object_pairs_hook=object_pairs)


def _finite_json(value: Any) -> Any:
    """Retain failed scorer values explicitly without emitting invalid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": repr(value)}
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_finite_json(item) for item in value]
    return value


def first_difference(left: Any, right: Any, path: str = "$") -> dict | None:
    """Compare the whole ordered structure; no rounding, alignment or tail loss."""
    if type(left) is not type(right):
        return {"path": path, "left": left, "right": right}
    if isinstance(left, dict):
        if left.keys() != right.keys():
            return {"path": path + ".keys", "left": sorted(left), "right": sorted(right)}
        for key in sorted(left):
            difference = first_difference(left[key], right[key], f"{path}.{key}")
            if difference is not None:
                return difference
    elif isinstance(left, list):
        for index, (a, b) in enumerate(zip(left, right)):
            difference = first_difference(a, b, f"{path}[{index}]")
            if difference is not None:
                return difference
        if len(left) != len(right):
            return {"path": path + ".length", "left": len(left), "right": len(right)}
    elif left != right:
        return {"path": path, "left": left, "right": right}
    return None


def _inventory(run_dir: Path) -> dict[str, str]:
    """Bind every file in this evidence directory, including partial/failure files."""
    return {
        str(path.relative_to(run_dir)).replace("\\", "/"):
        hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(run_dir.rglob("*")) if path.is_file()
    }


def read_run(run_dir: Path) -> dict:
    run_dir = run_dir.resolve()
    if not run_dir.is_dir():
        return {"directory": str(run_dir), "errors": ["run directory is missing"], "sha256": {}}
    try:
        before = _inventory(run_dir)
    except OSError as error:
        return {"directory": str(run_dir), "errors": [f"inventory failed: {error}"], "sha256": {}}
    result: dict = {"directory": str(run_dir), "errors": [], "sha256": before}
    try:
        plan = load_resolved_plan(run_dir / "demo_mission_plan.json")
        approvals = load_approval_records(run_dir / "operator_approvals.json")
        report = analyze_run(run_dir, limits=GateLimits())
        result["report"] = _finite_json(asdict(report))
        result["analysis_assumptions"] = {
            "sim_speedup": report.sim_speedup,
            "navigation_speedup_override": report.navigation_speedup_override,
        }
        result["recorded_inputs"] = {
            "plan": asdict(plan),
            "settings": _json(run_dir / "gcs_settings.json"),
            "command_bounds": _json(run_dir / "demo_command_bounds.json"),
            "approval_policy": [
                [row["sys_id"], row["requested_delay_s"]] for row in approvals
            ],
            "observed_approval_delay_s": [
                [row["sys_id"], row["actual_delay_s"]] for row in approvals
            ],
        }
        # asdict retains tuples; normalize only JSON representation, never numbers.
        result["recorded_inputs"] = json.loads(json.dumps(result["recorded_inputs"]))
        result["outcome"] = {
            "assignments": {str(k): v for k, v in report.global_assignments.items()},
            "approvals": [
                {key: value for key, value in row.items() if key not in {
                    "round_uid", "request_observed_at_unix_s", "approved_at_unix_s",
                    "actual_delay_s",
                }} | {"round_sequence": row["round_uid"].split(":")[1]}
                for row in approvals
            ],
            "vehicle_passed": {str(row.sys_id): row.passed for row in report.vehicles},
        }
        result["approval_audit"] = approvals
        result["metrics"] = {
            str(row["sys_id"]): row["metrics"] for row in result["report"]["vehicles"]
        }
        result["sampled_commands"] = {}
        result["wall_timestamps"] = {}
        for vehicle in plan.vehicles:
            try:
                episodes = parse_final_approach_command_episodes(
                    run_dir / f"uav_{vehicle.sys_id}_navigation_debug.csv"
                )
                if not episodes:
                    raise ValueError("no complete command episode")
            except Exception as error:
                result["errors"].append(f"vehicle {vehicle.sys_id}: {type(error).__name__}: {error}")
                continue
            rows = [[asdict(command) for command in episode] for episode in episodes]
            result["wall_timestamps"][str(vehicle.sys_id)] = [
                [row["wall_s"] for row in episode] for episode in rows
            ]
            result["sampled_commands"][str(vehicle.sys_id)] = [
                [{key: value for key, value in row.items() if key != "wall_s"}
                 for row in episode] for episode in rows
            ]
    except Exception as error:
        result["errors"].append(f"{type(error).__name__}: {error}")
    try:
        after = _inventory(run_dir)
    except OSError as error:
        result["errors"].append(f"final inventory failed: {error}")
        return result
    if before != after:
        result["errors"].append("run evidence changed while being read")
        result["sha256_after"] = after
    return result


def _comparison(left: dict, right: dict, field: str) -> dict:
    if field not in left or field not in right or left["errors"] or right["errors"]:
        return {"status": "incomplete", "first_difference": None}
    difference = first_difference(left[field], right[field])
    return {"status": "equal" if difference is None else "different",
            "first_difference": difference}


def _metric_deltas(left: dict, right: dict) -> dict:
    left_rows = {row["sys_id"]: row["metrics"] for row in left.get("report", {}).get("vehicles", [])}
    right_rows = {row["sys_id"]: row["metrics"] for row in right.get("report", {}).get("vehicles", [])}
    deltas = {}
    for sys_id in sorted(left_rows.keys() & right_rows.keys()):
        values = {}
        for name, a in left_rows[sys_id].items():
            b = right_rows[sys_id].get(name)
            if (type(a) in (int, float) and type(b) in (int, float)
                    and math.isfinite(a) and math.isfinite(b)):
                values[name] = {"reference": a, "repeat": b, "delta": b - a}
            elif a != b:
                values[name] = {"reference": a, "repeat": b, "delta": None}
        deltas[str(sys_id)] = values
    return {
        "status": "compared" if left_rows and left_rows.keys() == right_rows.keys() else "not comparable",
        "unmatched_reference_sysids": sorted(left_rows.keys() - right_rows.keys()),
        "unmatched_repeat_sysids": sorted(right_rows.keys() - left_rows.keys()),
        "per_vehicle": deltas,
    }


def analysis_identity() -> dict:
    """Identify selected analysis sources, not the historical flight runtime."""
    root = Path(__file__).resolve().parents[1]
    paths = sorted((root / "scripts").glob("eval_gcs_demo_*.py"))
    paths.extend(root / path for path in (
        "scripts/gcs_demo_comparison.py", "scripts/compare_gcs_demo_runs.py",
        "src/navpy/modules/vision/vision_profiles.json",
        "src/gcs/backend/task_confirm_uid.py",
    ))
    return {
        "python": sys.executable, "python_version": sys.version,
        "scope": "selected comparator/analyzer source files; not a full runtime manifest",
        "sha256": {str(path.relative_to(root)).replace("\\", "/"):
                   hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
    }


def compare_runs(directories: list[Path]) -> dict:
    if len(directories) < 2:
        raise ValueError("at least two run directories are required")
    resolved = [path.resolve() for path in directories]
    if len(set(resolved)) != len(resolved):
        raise ValueError("the same run directory cannot count as a repeat")
    analysis_before = analysis_identity()
    runs = [read_run(path) for path in resolved]
    analysis_after = analysis_identity()
    comparisons = []
    for index, repeat in enumerate(runs[1:], start=1):
        comparisons.append({
            "reference_index": 0, "repeat_index": index,
            **{field: _comparison(runs[0], repeat, field) for field in (
                "recorded_inputs", "outcome", "sampled_commands", "approval_audit",
                "wall_timestamps", "metrics",
            )},
            "metric_deltas": _metric_deltas(runs[0], repeat),
            "cross_run_numeric_tolerance": "undefined; differences are not waived",
            "independence_warnings": [
                "Identical file inventories: copied evidence or same boot cannot be excluded."
            ] if runs[0]["sha256"] and runs[0]["sha256"] == repeat["sha256"] else [],
        })
    per_run_pass = analysis_before == analysis_after and all(not row["errors"] and row.get("report", {}).get("passed") is True
                       for row in runs)
    recorded_equal = per_run_pass and all(
        pair[field]["status"] == "equal" for pair in comparisons
        for field in ("recorded_inputs", "outcome", "sampled_commands", "metrics")
    )
    return {
        "schema_version": 1, "scope": "offline recorded-evidence comparison",
        "full_scenario_repeatability": "unverified",
        "recorded_evidence_equal": recorded_equal, "all_per_run_gates_pass": per_run_pass,
        "limits": asdict(GateLimits()), "limitations": LIMITATIONS,
        "analysis_identity": analysis_before, "analysis_identity_after": analysis_after,
        "analysis_source_stable": analysis_before == analysis_after,
        "declared_audit_fields": ["absolute wall timestamps", "approval boot nonce"],
        "runs": runs, "comparisons": comparisons,
    }
