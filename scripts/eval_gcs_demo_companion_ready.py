"""Exact configured-companion restart/readiness gate for demo evaluation."""

from __future__ import annotations

import math

from scripts.eval_gcs_demo_config import aas_params_for
from scripts.eval_gcs_demo_models import RegressionError, StackContext, ThreeUavIds
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import owner_sys_id_for


def _object(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise TypeError(f"{label} response must be an object")
    return value


def _list(value: JsonValue, label: str) -> list[JsonValue]:
    if not isinstance(value, list):
        raise TypeError(f"{label} response must be a list")
    return value


def restart_configured_companions_ready(
    context: StackContext,
    timeout_s: float,
    nav_start_bindings: list[dict[str, JsonValue]],
) -> list[dict[str, JsonValue]]:
    """Use the operator API and require three fresh, hydrated runtimes."""
    ids = ThreeUavIds.from_values(context.sys_ids)
    owner = owner_sys_id_for(ids)
    binding_by_sys_id = {
        int(binding["sys_id"]): binding for binding in nav_start_bindings
    }
    if set(binding_by_sys_id) != set(ids.values):
        raise RegressionError("nav-start binding sysids do not match companions")
    expected = {
        sys_id: {
            "targ_wps": int(aas_params_for(sys_id, owner)["targ_wps"]),
            "nav_last_wp": int(aas_params_for(sys_id, owner)["nav_last_wp"]),
            "nav_min_wp_seq": int(
                binding_by_sys_id[sys_id]["mission_sequence"]
            ),
            "mission_items": int(binding_by_sys_id[sys_id]["mission_count"]),
        }
        for sys_id in ids.values
    }
    status, raw = context.api.post_json(
        "/api/control/navpy-sim/restart-ready",
        {
            "instances": [
                {
                    "sys_id": sys_id,
                    "expected_params": {
                        "targ_wps": expected[sys_id]["targ_wps"],
                        "nav_last_wp": expected[sys_id]["nav_last_wp"],
                    },
                }
                for sys_id in ids.values
            ],
        },
        timeout=timeout_s,
    )
    body = _object(raw, "restart-ready")
    if status != 200 or body.get("status") != "ready":
        raise RegressionError(f"unexpected restart-ready response: {body}")
    instances: dict[int, dict[str, JsonValue]] = {}
    for raw_instance in _list(body.get("instances"), "restart-ready instances"):
        instance = _object(raw_instance, "restart-ready instance")
        sys_id = instance.get("sys_id")
        if type(sys_id) is not int or sys_id in instances:
            raise RegressionError("restart-ready returned invalid/duplicate sys_id")
        instances[sys_id] = instance
    if set(instances) != set(ids.values):
        raise RegressionError(
            f"restart-ready sysids {sorted(instances)} did not match {list(ids.values)}"
        )
    for sys_id in ids.values:
        _validate_ready_instance(instances[sys_id], expected[sys_id])
    process_pids = [
        int(_object(instances[sys_id]["runtime"], "restart-ready runtime")[
            "process_pid"
        ])
        for sys_id in ids.values
    ]
    if len(set(process_pids)) != len(process_pids):
        raise RegressionError(
            f"restart-ready returned duplicate runtime process pids {process_pids}"
        )
    return [instances[sys_id] for sys_id in ids.values]


def capture_post_restart_status(
    context: StackContext,
    restart_instances: list[dict[str, JsonValue]],
) -> list[dict[str, JsonValue]]:
    """Recheck that each exact ready generation is still live."""
    ids = ThreeUavIds.from_values(context.sys_ids)
    expected = {
        int(instance["sys_id"]): instance for instance in restart_instances
    }
    payload = _object(
        context.api.get_json("/api/control/navpy-sim/status"),
        "post-restart companion status",
    )
    observed: dict[int, dict[str, JsonValue]] = {}
    for raw in _list(payload.get("instances"), "post-restart instances"):
        instance = _object(raw, "post-restart instance")
        sys_id = instance.get("sys_id")
        if type(sys_id) is not int or sys_id in observed:
            raise RegressionError("post-restart status has invalid/duplicate sys_id")
        observed[sys_id] = instance
    if set(observed) != set(ids.values) or set(expected) != set(ids.values):
        raise RegressionError("post-restart status sysids do not match companions")
    captured: list[dict[str, JsonValue]] = []
    for sys_id in ids.values:
        actual = observed[sys_id]
        ready = expected[sys_id]
        actual_runtime = _object(
            actual.get("runtime"),
            "post-restart runtime",
        )
        ready_runtime = _object(ready.get("runtime"), "restart-ready runtime")
        if (
            actual.get("running") is not True
            or actual.get("ready") is not True
            or actual.get("exit_code") is not None
            or actual.get("pid") != ready.get("pid")
            or actual_runtime != ready_runtime
        ):
            raise RegressionError(
                f"NavPy sys_id={sys_id} changed after readiness: {actual}"
            )
        captured.append({
            "sys_id": sys_id,
            "pid": actual["pid"],
            "running": True,
            "ready": True,
            "exit_code": None,
            "runtime": actual_runtime,
        })
    return captured


def _validate_ready_instance(
    instance: dict[str, JsonValue],
    expected: dict[str, int],
) -> None:
    sys_id = instance["sys_id"]
    old_pid = instance.get("old_pid")
    pid = instance.get("pid")
    runtime = _object(instance.get("runtime"), "restart-ready runtime")
    process_pid = runtime.get("process_pid")
    scheduler_rate_hz = runtime.get("scheduler_rate_hz")
    if (
        instance.get("status") != "ready"
        or type(old_pid) is not int
        or old_pid <= 0
        or type(pid) is not int
        or pid <= 0
        or old_pid == pid
        or type(process_pid) is not int
        or process_pid <= 0
        or isinstance(scheduler_rate_hz, bool)
        or not isinstance(scheduler_rate_hz, (int, float))
        or not math.isfinite(float(scheduler_rate_hz))
        or float(scheduler_rate_hz) <= 0.0
        or type(runtime.get("mission_items")) is not int
        or runtime.get("mission_items") != expected["mission_items"]
        or runtime.get("targ_wps") != expected["targ_wps"]
        or runtime.get("nav_last_wp") != expected["nav_last_wp"]
        or runtime.get("nav_min_wp_seq") != expected["nav_min_wp_seq"]
    ):
        raise RegressionError(
            f"NavPy sys_id={sys_id} was not freshly hydrated: {instance}"
        )


__all__ = [
    "capture_post_restart_status",
    "restart_configured_companions_ready",
]
