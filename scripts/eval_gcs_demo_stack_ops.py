"""Vehicle discovery, parameter, and companion operations for one stack."""

from __future__ import annotations

import math
from typing import Iterable

from scripts.eval_gcs_demo_command_bounds import (
    DemoCommandBounds,
    command_bounds_from_snapshot,
)
from scripts.eval_gcs_demo_config import aas_params_for, full_param_changes
from scripts.eval_gcs_demo_constants import TERMINAL_ROLL_LIMIT_DEG
from scripts.eval_gcs_demo_models import RegressionError, StackContext, ThreeUavIds
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_process import wait_until
from scripts.eval_gcs_demo_scenario import owner_sys_id_for


def _object(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise TypeError(f"{label} response must be an object")
    return value


def _list(value: JsonValue, label: str) -> list[JsonValue]:
    if not isinstance(value, list):
        raise TypeError(f"{label} response must be a list")
    return value


def wait_for_vehicles(
    context: StackContext,
    timeout_s: float,
) -> list[dict[str, JsonValue]]:
    expected = set(context.sys_ids)

    def probe() -> list[dict[str, JsonValue]] | None:
        raw = _list(context.api.get_json("/api/vehicles"), "vehicles")
        vehicles: dict[int, dict[str, JsonValue]] = {}
        for item in raw:
            vehicle = _object(item, "vehicle")
            sys_id = vehicle.get("sys_id")
            if type(sys_id) is not int or sys_id <= 0:
                raise TypeError("vehicle sys_id must be a positive integer")
            if sys_id in vehicles:
                raise ValueError(f"duplicate vehicle sys_id {sys_id}")
            vehicles[sys_id] = vehicle
        if set(vehicles) != expected:
            return None
        if not all(vehicles[sys_id].get("link_ok") is True for sys_id in context.sys_ids):
            return None
        return [vehicles[sys_id] for sys_id in context.sys_ids]

    return wait_until(
        "three linked GCS vehicles",
        timeout_s,
        probe,
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
    )


def _companion_ids(context: StackContext) -> set[int]:
    payload = _object(
        context.api.get_json("/api/control/navpy-sim/status"),
        "companion status",
    )
    statuses = _list(payload.get("instances"), "companion instances")
    running: set[int] = set()
    seen: set[int] = set()
    for raw in statuses:
        status = _object(raw, "companion instance")
        sys_id = status.get("sys_id")
        if type(sys_id) is not int or sys_id <= 0:
            raise TypeError("companion sys_id must be a positive integer")
        if sys_id in seen:
            raise ValueError(f"duplicate companion sys_id {sys_id}")
        seen.add(sys_id)
        if status.get("running") is True:
            running.add(sys_id)
    return running


def wait_for_companions(context: StackContext, timeout_s: float) -> None:
    expected = set(context.sys_ids)
    wait_until(
        "three running NavPy companions",
        timeout_s,
        lambda: _companion_ids(context) == expected,
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
    )


def wait_for_companions_stopped(context: StackContext, timeout_s: float) -> None:
    expected = set(context.sys_ids)
    wait_until(
        "NavPy companions stopped",
        timeout_s,
        lambda: not (_companion_ids(context) & expected),
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
    )


def restart_companions(context: StackContext, timeout_s: float) -> None:
    context.api.post_json("/api/control/navpy-sim/stop-all")
    wait_for_companions_stopped(context, timeout_s)
    context.api.post_json(
        "/api/control/navpy-sim/start-all",
        {"instances": [{"sys_id": sys_id} for sys_id in context.sys_ids]},
    )
    wait_for_companions(context, timeout_s)


def require_write_results(
    payload: JsonValue,
    names: Iterable[str],
    label: str,
) -> None:
    body = _object(payload, label)
    results = _object(body.get("results"), f"{label} results")
    failed: list[str] = []
    for name in names:
        result = results.get(name)
        ok = result is True or (
            isinstance(result, dict) and result.get("ok") is True
        )
        if not ok:
            failed.append(f"{name}={result!r}")
    if failed:
        raise RegressionError(f"{label} write failed: {', '.join(failed)}")


def configure_vehicle_params(
    context: StackContext,
    timeout_s: float,
) -> DemoCommandBounds:
    wait_for_vehicles(context, timeout_s)
    ids = ThreeUavIds.from_values(context.sys_ids)
    owner = owner_sys_id_for(ids)
    bounds = []
    for sys_id in ids.values:
        snapshot = context.api.get_json(
            f"/api/vehicles/{sys_id}/parameters?refresh=true",
            timeout=90.0,
        )
        bounds.append(command_bounds_from_snapshot(
            sys_id,
            snapshot,
            roll_limit_deg=TERMINAL_ROLL_LIMIT_DEG,
        ))
        full = context.api.put_json(
            f"/api/vehicles/{sys_id}/parameters",
            {"changes": full_param_changes()},
            timeout=60.0,
        )
        require_write_results(
            full,
            ("ROLL_LIMIT_DEG", "SIM_SPEEDUP"),
            f"vehicle {sys_id} full-param",
        )
        aas = aas_params_for(sys_id, owner)
        written = context.api.put_json(
            f"/api/vehicles/{sys_id}/params",
            {"params": aas},
            timeout=60.0,
        )
        require_write_results(written, aas, f"vehicle {sys_id} AAS")
    return DemoCommandBounds(1, tuple(bounds))  # type: ignore[arg-type]


def verify_aas_profile(context: StackContext) -> None:
    ids = ThreeUavIds.from_values(context.sys_ids)
    owner = owner_sys_id_for(ids)
    for sys_id in ids.values:
        payload = _object(
            context.api.get_json(f"/api/vehicles/{sys_id}/params", timeout=90.0),
            "vehicle AAS params",
        )
        params = _object(payload.get("params"), "vehicle AAS params values")
        mismatches: list[str] = []
        for name, wanted in aas_params_for(sys_id, owner).items():
            actual = params.get(name)
            if isinstance(wanted, bool):
                equal = actual is wanted
            elif isinstance(wanted, (int, float)):
                equal = (
                    not isinstance(actual, bool)
                    and isinstance(actual, (int, float))
                    and math.isclose(float(actual), float(wanted), abs_tol=1e-6)
                )
            else:
                equal = actual == wanted
            if not equal:
                mismatches.append(f"{name}: expected {wanted!r}, got {actual!r}")
        if mismatches:
            raise RegressionError(
                f"vehicle {sys_id} persisted profile mismatch: {'; '.join(mismatches)}"
            )


def start_esp32_simulator(context: StackContext) -> None:
    status = _object(
        context.api.get_json("/api/control/launch/esp32-sim/status"),
        "ESP32 simulator status",
    )
    if status.get("running") is True:
        return
    response_status, raw = context.api.post_json(
        "/api/control/launch/esp32-sim/start"
    )
    body = _object(raw, "ESP32 simulator start")
    if response_status != 200 or body.get("status") != "started":
        raise RegressionError(f"unexpected ESP32 simulator response: {body}")


def wait_for_missions(context: StackContext, timeout_s: float) -> None:
    def probe() -> bool:
        vehicles = wait_for_vehicles(context, min(timeout_s, 5.0))
        return all(
            type(vehicle.get("mission_total")) is int
            and int(vehicle["mission_total"]) > 0
            for vehicle in vehicles
        )

    wait_until(
        "three onboard missions",
        timeout_s,
        probe,
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
    )


__all__ = [
    "configure_vehicle_params",
    "require_write_results",
    "restart_companions",
    "start_esp32_simulator",
    "verify_aas_profile",
    "wait_for_companions",
    "wait_for_companions_stopped",
    "wait_for_missions",
    "wait_for_vehicles",
]
