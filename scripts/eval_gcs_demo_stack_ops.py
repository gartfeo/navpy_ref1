"""Vehicle discovery, parameter, and companion operations for one stack."""

from __future__ import annotations

import math
from typing import Iterable

from scripts.eval_gcs_demo_command_bounds import (
    DemoCommandBounds,
    command_bounds_from_snapshot,
)
from scripts.eval_gcs_demo_config import aas_params_for
from scripts.eval_gcs_demo_constants import (
    FINAL_APPROACH_ROLL_LIMIT_DEG,
    PARAM_HTTP_TIMEOUT_S,
    PARAM_REQUEST_BUDGET_S,
)
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_ports import JsonApiPort, JsonValue
from scripts.eval_gcs_demo_process import wait_until


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
        "every scenario vehicle linked to the GCS",
        timeout_s,
        probe,
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
        clock=context.clock,
    )


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


def _snapshot_value(snapshot: JsonValue, name: str) -> JsonValue:
    records = snapshot.get("params") if isinstance(snapshot, dict) else None
    for record in records if isinstance(records, list) else ():
        if isinstance(record, dict) and record.get("name") == name:
            return record.get("value")
    return None


def _ensure_roll_limit(context: StackContext, sys_id: int, snapshot: JsonValue) -> None:
    """Write ROLL_LIMIT_DEG only where boot did not already set it.

    The demo eval's pre-boot defaults set it, so it writes nothing; a run that
    boots the shared templates (the ideal eval) still gets the write.
    """
    value = _snapshot_value(snapshot, "ROLL_LIMIT_DEG")
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isclose(
        float(value), FINAL_APPROACH_ROLL_LIMIT_DEG, abs_tol=1e-6
    ):
        return
    change = {"name": "ROLL_LIMIT_DEG", "value": FINAL_APPROACH_ROLL_LIMIT_DEG}
    written = context.api.put_json(
        f"/api/vehicles/{sys_id}/parameters", {"changes": [change]}, timeout=60.0
    )
    require_write_results(written, ("ROLL_LIMIT_DEG",), f"vehicle {sys_id} full-param")


def refresh_parameter_snapshot(api: JsonApiPort, sys_id: int) -> JsonValue:
    """Wall mode's command-bound read: the MAVFTP full-parameter refresh."""
    return api.get_json(f"/api/vehicles/{sys_id}/parameters?refresh=true", timeout=90.0)


def configure_vehicle_params(
    context: StackContext,
    timeout_s: float,
) -> DemoCommandBounds:
    wait_for_vehicles(context, timeout_s)
    scenario = context.scenario
    ids = scenario.uav_ids(context.sys_ids)
    owner = scenario.owner_sys_id(ids)
    read_parameters = context.parameter_reader or refresh_parameter_snapshot
    bounds = []
    for sys_id in ids.values:
        snapshot = read_parameters(context.api, sys_id)
        bounds.append(command_bounds_from_snapshot(
            sys_id,
            snapshot,
            roll_limit_deg=FINAL_APPROACH_ROLL_LIMIT_DEG,
        ))
        _ensure_roll_limit(context, sys_id, snapshot)
        aas = aas_params_for(scenario, sys_id, owner)
        written = context.api.put_json(
            f"/api/vehicles/{sys_id}/params",
            {"params": aas, "timeout_s": PARAM_REQUEST_BUDGET_S},
            timeout=PARAM_HTTP_TIMEOUT_S,
        )
        require_write_results(written, aas, f"vehicle {sys_id} AAS")
    return DemoCommandBounds(1, tuple(bounds))


def verify_aas_profile(context: StackContext) -> None:
    scenario = context.scenario
    ids = scenario.uav_ids(context.sys_ids)
    owner = scenario.owner_sys_id(ids)
    for sys_id in ids.values:
        payload = _object(
            context.api.get_json(f"/api/vehicles/{sys_id}/params", timeout=90.0),
            "vehicle AAS params",
        )
        params = _object(payload.get("params"), "vehicle AAS params values")
        mismatches: list[str] = []
        for name, wanted in aas_params_for(scenario, sys_id, owner).items():
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
        "every scenario vehicle's onboard mission",
        timeout_s,
        probe,
        interval_s=1.0,
        transient_exceptions=(RegressionError,),
        clock=context.clock,
    )


__all__ = [
    "configure_vehicle_params",
    "refresh_parameter_snapshot",
    "require_write_results",
    "start_esp32_simulator",
    "verify_aas_profile",
    "wait_for_missions",
    "wait_for_vehicles",
]
