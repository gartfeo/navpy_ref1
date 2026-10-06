"""Validate the exact NavPy process generation used by an ideal demo run."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from scripts.eval_gcs_demo_config import aas_params_for
from scripts.eval_gcs_demo_constants import SIM_SPEEDUP


def runtime_identities(
    log_dir: Path,
    sys_ids: tuple[int, int, int],
) -> dict[int, tuple[int, float]]:
    launch = json.loads(
        (log_dir / "stack_launch_evidence.json").read_text(encoding="utf-8")
    )
    _validate_stack_launch(launch, sys_ids)
    payload = json.loads(
        (log_dir / "mission_upload_evidence.json").read_text(encoding="utf-8")
    )
    if not isinstance(payload, dict):
        raise TypeError("mission-upload evidence must be an object")
    rows = payload.get("restart_instances")
    if not isinstance(rows, list):
        raise TypeError("mission-upload evidence has no restart_instances list")
    bindings = _index_rows(
        payload.get("nav_start_bindings"),
        label="nav-start binding",
    )
    statuses = _index_rows(
        payload.get("post_restart_status"),
        label="post-restart status",
    )
    identities: dict[int, tuple[int, float]] = {}
    wrapper_pids: list[int] = []
    owner_sys_id = sys_ids[0]
    for row in rows:
        sys_id, process_pid, scheduler_rate_hz, wrapper_pid = _runtime_identity(
            row,
            owner_sys_id=owner_sys_id,
            binding_by_sys_id=bindings,
            status_by_sys_id=statuses,
        )
        if sys_id in identities:
            raise ValueError(f"duplicate restart runtime sys_id {sys_id}")
        identities[sys_id] = process_pid, scheduler_rate_hz
        wrapper_pids.append(wrapper_pid)
    _validate_exact_runtime_set(
        identities,
        bindings,
        statuses,
        wrapper_pids,
        sys_ids,
    )
    return identities


def _validate_stack_launch(
    launch: object,
    sys_ids: tuple[int, int, int],
) -> None:
    if not isinstance(launch, dict):
        raise TypeError("stack-launch evidence must be an object")
    launch_pid = launch.get("sitl_pid")
    launch_pid_start = launch.get("sitl_pid_start")
    if (
        type(launch.get("launch_token")) is not str
        or not launch["launch_token"]
        or launch.get("sitl") is not True
        or launch.get("sitl_verified") is not True
        or launch.get("sitl_speedup") != SIM_SPEEDUP
        or type(launch_pid) is not int
        or launch_pid <= 0
        or isinstance(launch_pid_start, bool)
        or not isinstance(launch_pid_start, (int, float))
        or not math.isfinite(float(launch_pid_start))
        or float(launch_pid_start) <= 0.0
        or tuple(launch.get("sysids", ())) != sys_ids
    ):
        raise ValueError("stack-launch evidence is not a verified exact generation")


def _index_rows(raw_rows: object, *, label: str) -> dict[int, dict[str, Any]]:
    if not isinstance(raw_rows, list):
        raise TypeError(f"mission-upload evidence has no {label.replace(' ', '_')} list")
    indexed: dict[int, dict[str, Any]] = {}
    for row in raw_rows:
        if not isinstance(row, dict) or type(row.get("sys_id")) is not int:
            raise TypeError(f"{label} must contain an integer sys_id")
        sys_id = int(row["sys_id"])
        if sys_id in indexed:
            raise ValueError(f"duplicate {label} sys_id {sys_id}")
        indexed[sys_id] = row
    return indexed


def _runtime_identity(
    row: object,
    *,
    owner_sys_id: int,
    binding_by_sys_id: dict[int, dict[str, Any]],
    status_by_sys_id: dict[int, dict[str, Any]],
) -> tuple[int, int, float, int]:
    if not isinstance(row, dict):
        raise TypeError("restart instance must be an object")
    sys_id = row.get("sys_id")
    runtime = row.get("runtime")
    if type(sys_id) is not int or not isinstance(runtime, dict):
        raise TypeError("restart instance has invalid sys_id/runtime")
    process_pid = runtime.get("process_pid")
    scheduler_rate_hz = runtime.get("scheduler_rate_hz")
    old_pid = row.get("old_pid")
    wrapper_pid = row.get("pid")
    expected = aas_params_for(sys_id, owner_sys_id)
    binding = binding_by_sys_id.get(sys_id)
    post_status = status_by_sys_id.get(sys_id)
    if (
        binding is None
        or binding.get("nav_waypoint_ordinal") != expected["nav_last_wp"]
        or type(binding.get("mission_sequence")) is not int
        or int(binding["mission_sequence"]) <= 0
        or type(binding.get("mission_count")) is not int
        or int(binding["mission_count"]) <= 0
        or post_status is None
        or row.get("status") != "ready"
        or type(old_pid) is not int
        or old_pid <= 0
        or type(wrapper_pid) is not int
        or wrapper_pid <= 0
        or old_pid == wrapper_pid
        or type(process_pid) is not int
        or process_pid <= 0
        or isinstance(scheduler_rate_hz, bool)
        or not isinstance(scheduler_rate_hz, (int, float))
        or not math.isfinite(float(scheduler_rate_hz))
        or float(scheduler_rate_hz) <= 0.0
        or runtime.get("mission_items") != binding["mission_count"]
        or runtime.get("targ_wps") != expected["targ_wps"]
        or runtime.get("nav_last_wp") != expected["nav_last_wp"]
        or runtime.get("nav_min_wp_seq") != binding["mission_sequence"]
        or post_status.get("running") is not True
        or post_status.get("ready") is not True
        or post_status.get("exit_code") is not None
        or post_status.get("pid") != wrapper_pid
        or post_status.get("runtime") != runtime
    ):
        raise ValueError(
            f"restart instance {sys_id} has invalid process/rate identity"
        )
    return sys_id, process_pid, float(scheduler_rate_hz), wrapper_pid


def _validate_exact_runtime_set(
    identities: dict[int, tuple[int, float]],
    bindings: dict[int, dict[str, Any]],
    statuses: dict[int, dict[str, Any]],
    wrapper_pids: list[int],
    sys_ids: tuple[int, int, int],
) -> None:
    for label, actual in (
        ("restart runtime", set(identities)),
        ("nav-start binding", set(bindings)),
        ("post-restart status", set(statuses)),
    ):
        if actual != set(sys_ids):
            raise ValueError(
                f"{label} sysids {sorted(actual)} do not match {list(sys_ids)}"
            )
    process_pids = [identity[0] for identity in identities.values()]
    if len(set(process_pids)) != len(process_pids):
        raise ValueError(f"restart runtime process pids are not unique {process_pids}")
    if len(set(wrapper_pids)) != len(wrapper_pids):
        raise ValueError(
            f"restart runtime wrapper pids are not unique {wrapper_pids}"
        )


__all__ = ["runtime_identities"]
