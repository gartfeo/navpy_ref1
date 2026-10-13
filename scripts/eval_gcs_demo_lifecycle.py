"""One explicit lifecycle per run: connect, companions start once, acknowledged stop."""

from __future__ import annotations

from urllib.parse import urlencode

from scripts.eval_gcs_demo_execution import wait_until
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_ports import JsonValue

from gcs.backend.navpy_process_stop import STOP_ACKED


def _object(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise TypeError(f"{label} response must be an object")
    return value


def connect_vehicles(context: StackContext) -> None:
    """Connect each vehicle in manifest (ordinal) order, so the roster order is fixed."""
    if context.monitor_device is None:
        raise RegressionError("the owned stack has no GCS monitor device")
    for sys_id in context.sys_ids:
        query = urlencode({"device": context.monitor_device, "sys_id": sys_id})
        status, raw = context.api.post_json(f"/api/vehicles/connect?{query}")
        body = _object(raw, "vehicle connect")
        if status != 200 or body.get("status") != "connected" or body.get("sys_id") != sys_id:
            raise RegressionError(f"vehicle {sys_id} connect failed: {body}")


def companion_ids(context: StackContext) -> set[int]:
    """The sysids whose tracked companion is running."""
    return {sys_id for sys_id, running in _tracked(context).items() if running}


def _tracked(context: StackContext) -> dict[int, bool]:
    """Every tracked companion generation (running or exited) -> running."""
    payload = _object(
        context.api.get_json("/api/control/navpy-sim/status"),
        "companion status",
    )
    statuses = payload.get("instances")
    if not isinstance(statuses, list):
        raise TypeError("companion instances response must be a list")
    tracked: dict[int, bool] = {}
    for raw in statuses:
        status = _object(raw, "companion instance")
        sys_id = status.get("sys_id")
        if type(sys_id) is not int or sys_id <= 0:
            raise TypeError("companion sys_id must be a positive integer")
        if sys_id in tracked:
            raise ValueError(f"duplicate companion sys_id {sys_id}")
        tracked[sys_id] = status.get("running") is True
    return tracked


def wait_for_companions(context: StackContext, timeout_s: float) -> None:
    expected = set(context.sys_ids)
    wait_until(
        "three running NavPy companions",
        timeout_s,
        lambda: companion_ids(context) == expected,
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
        clock=context.clock,
    )


def start_companions(context: StackContext, timeout_s: float) -> None:
    """Start every companion once, after configuration and mission upload."""
    if companion_ids(context):
        raise RegressionError("a companion is already running before the single start")
    status, raw = context.api.post_json(
        "/api/control/navpy-sim/start-all",
        {"instances": [{"sys_id": sys_id} for sys_id in context.sys_ids]},
    )
    results = _object(_object(raw, "companion start").get("results"), "companion start results")
    failed = {
        key: value for key, value in results.items()
        if not isinstance(value, dict) or value.get("status") != "started"
    }
    if status != 200 or failed or len(results) != len(context.sys_ids):
        raise RegressionError(f"companion start failed: {raw}")
    wait_for_companions(context, timeout_s)


def stop_companions(context: StackContext) -> None:
    """Stop every tracked companion; anything but an ack for each is a host fault."""
    tracked = {str(sys_id) for sys_id in _tracked(context)}
    status, raw = context.api.post_json("/api/control/navpy-sim/stop-all", timeout=120.0)
    outcomes = _object(_object(raw, "companion stop").get("outcomes"), "companion stop outcomes")
    missed = {key: value for key, value in outcomes.items() if value != STOP_ACKED}
    missed.update({key: "no outcome" for key in tracked - set(outcomes)})
    if status != 200 or missed:
        raise RegressionError(
            f"host fault: companion stop not acknowledged: {missed or raw}"
        )


__all__ = [
    "companion_ids",
    "connect_vehicles",
    "start_companions",
    "stop_companions",
    "wait_for_companions",
]
