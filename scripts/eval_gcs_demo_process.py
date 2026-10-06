"""Scoped process ownership, strict polling, and aggregate teardown."""

from __future__ import annotations

import json
import os
import math
import secrets
import subprocess
from pathlib import Path
from types import TracebackType
from typing import Callable, Mapping, TypeVar

from scripts.eval_gcs_demo_api import FileTail, GcsApi
from scripts.eval_gcs_demo_config import launch_command, stop_command
from scripts.eval_gcs_demo_constants import (
    BACKEND_LOG,
    ROOT,
    SIM_SPEEDUP,
)
from scripts.eval_gcs_demo_execution import (
    SystemClock,
    exclusive_evaluator_lock,
    run_process,
    wait_until,
)
from scripts.eval_gcs_demo_models import (
    RegressionError,
    StackContext,
    ThreeUavIds,
)
from scripts.eval_gcs_demo_ports import JsonValue


T = TypeVar("T")


class PreexistingOwnedStackError(RegressionError):
    """The exact evaluator refuses to reuse another stack generation."""


def registry_path(env: Mapping[str, str]) -> Path:
    override = env.get("GCS_INSTANCE_REGISTRY", "").strip()
    return Path(override) if override else Path.home() / ".gcs" / "instances.json"


def owned_entry(env: Mapping[str, str]) -> dict[str, JsonValue] | None:
    path = registry_path(env)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    if not isinstance(payload, dict) or not isinstance(payload.get("instances"), dict):
        raise TypeError(f"invalid GCS registry schema in {path}")
    owner = f"dir:{os.path.normcase(os.path.abspath(str(ROOT)))}"
    candidates: list[dict[str, JsonValue]] = []
    for raw in payload["instances"].values():
        if not isinstance(raw, dict):
            raise TypeError(f"invalid GCS registry entry in {path}")
        if raw.get("owner") == owner and raw.get("label") == "gcs":
            candidates.append(raw)
    if len(candidates) > 1:
        raise ValueError("multiple owned GCS registry entries are active")
    return candidates[0] if candidates else None


def launch_stack(env: dict[str, str], timeout_s: float) -> StackContext:
    if owned_entry(env) is not None:
        raise PreexistingOwnedStackError(
            "an owned GCS stack already exists; exact evaluation requires "
            "a fresh generation"
        )
    launch_token = secrets.token_hex(16)
    backend_offset = BACKEND_LOG.stat().st_size if BACKEND_LOG.exists() else 0
    result = run_process(
        launch_command(launch_token),
        env,
        timeout_s=min(timeout_s, 60.0),
    )
    if result.returncode != 0:
        raise RegressionError(
            f"gcs_launch failed ({result.returncode}): "
            f"{result.stdout}\n{result.stderr}"
        )
    entry = wait_until(
        "verified owned SITL registry generation",
        min(timeout_s, 600.0),
        lambda: _verified_launch_entry(env, launch_token),
        transient_exceptions=(OSError,),
    )
    sys_ids = ThreeUavIds.from_values(entry.get("sysids", ())).values
    backend = entry.get("backend")
    if type(backend) is not int:
        raise TypeError("owned GCS registry backend port must be an integer")
    api = GcsApi(backend)
    try:
        def healthy() -> bool:
            payload = api.get_json("/health")
            if not isinstance(payload, dict):
                raise TypeError("GCS health response must be an object")
            return payload.get("status") == "ok"

        wait_until(
            "GCS backend health",
            30.0,
            healthy,
            transient_exceptions=(RegressionError,),
        )
        entry = _verified_launch_entry(env, launch_token)
        if entry is None:
            raise RegressionError(
                "verified SITL registry generation changed after backend health"
            )
    except BaseException as primary:
        try:
            api.close()
        except BaseException as cleanup:
            raise RegressionError(
                "backend health failed and API close also failed: "
                f"{_error_text(primary)}; {_error_text(cleanup)}"
            ) from primary
        raise
    evidence = {
        "launch_token": launch_token,
        "sitl": True,
        "sitl_verified": True,
        "sitl_speedup": entry["sitl_speedup"],
        "sitl_pid": entry["sitl_pid"],
        "sitl_pid_start": entry["sitl_pid_start"],
        "sysids": list(sys_ids),
        "backend": backend,
    }
    return StackContext(
        api,
        sys_ids,
        FileTail(BACKEND_LOG, backend_offset),
        evidence,
    )


def _verified_launch_entry(
    env: Mapping[str, str],
    launch_token: str,
) -> dict[str, JsonValue] | None:
    entry = owned_entry(env)
    if entry is None or entry.get("sitl_launch_token") != launch_token:
        return None
    verdict = entry.get("sitl_verified")
    if verdict is False:
        raise RegressionError(
            "SITL launch failed verification: "
            f"{entry.get('sitl_error') or 'unknown'}"
        )
    if verdict is not True:
        return None
    speedup = entry.get("sitl_speedup")
    pid = entry.get("sitl_pid")
    pid_start = entry.get("sitl_pid_start")
    if (
        entry.get("sitl") is not True
        or isinstance(speedup, bool)
        or not isinstance(speedup, (int, float))
        or not math.isfinite(float(speedup))
        or not math.isclose(float(speedup), SIM_SPEEDUP, abs_tol=1e-9)
        or type(pid) is not int
        or pid <= 0
        or isinstance(pid_start, bool)
        or not isinstance(pid_start, (int, float))
        or not math.isfinite(float(pid_start))
        or float(pid_start) <= 0.0
    ):
        raise RegressionError(f"invalid verified SITL registry entry: {entry}")
    ThreeUavIds.from_values(entry.get("sysids", ()))
    return entry


def stop_stack(env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    result = run_process(stop_command(), env, timeout_s=120.0)
    if result.returncode != 0:
        raise RegressionError(
            f"scoped gcs_stop failed ({result.returncode}): "
            f"{result.stdout} {result.stderr}"
        )
    return result


def _error_text(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"


def with_owned_stack(
    env: dict[str, str],
    timeout_s: float,
    action: Callable[[StackContext], T],
) -> T:
    """Attempt close, scoped stop, and tail; report every failure."""
    context: StackContext | None = None
    result: T | None = None
    errors: list[BaseException] = []
    try:
        context = launch_stack(env, timeout_s)
        result = action(context)
    except PreexistingOwnedStackError:
        raise
    except BaseException as error:
        errors.append(error)
    if context is not None:
        try:
            context.api.close()
        except BaseException as error:
            errors.append(error)
    try:
        stop_stack(env)
    except BaseException as error:
        errors.append(error)
    if context is not None:
        try:
            context.backend_tail.read_new()
        except BaseException as error:
            errors.append(error)
    if errors:
        raise RegressionError(
            "owned GCS stack failed: " + "; ".join(_error_text(error) for error in errors)
        ) from errors[0]
    return result  # type: ignore[return-value]


__all__ = [
    "RegressionError",
    "PreexistingOwnedStackError",
    "SystemClock",
    "exclusive_evaluator_lock",
    "launch_stack",
    "owned_entry",
    "registry_path",
    "run_process",
    "stop_stack",
    "wait_until",
    "with_owned_stack",
]
