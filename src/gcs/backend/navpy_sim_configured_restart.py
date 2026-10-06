"""Atomic configured restart of an exact NavPy companion set."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Mapping
from typing import Any, Protocol

from navpy.exception_groups import BaseExceptionGroup

from gcs.backend.settings_model import GcsSettings


NAVPY_RESTART_READY_TIMEOUT_S = 45.0
_configuration_restart_lock = threading.RLock()


class ConfiguredRestartManager(Protocol):
    def get_all_status(self) -> list[dict[str, object]]: ...

    def stop(self, sys_id: int) -> bool: ...

    def stop_generation(self, sys_id: int, pid: int) -> bool: ...

    def wait_ready(
        self,
        sys_id: int,
        pid: int,
        timeout_s: float,
    ) -> dict[str, object]: ...


StartInstance = Callable[[int, str, GcsSettings], Any]
ConnectionResolver = Callable[[int, GcsSettings], str]


def restart_configured_instances(
    expected_by_sys_id: Mapping[int, Mapping[str, int]],
    *,
    manager: ConfiguredRestartManager,
    settings: GcsSettings,
    managed_sysids: set[int],
    start_instance: StartInstance,
    connection_for: ConnectionResolver,
    timeout_s: float,
) -> list[dict[str, object]]:
    """Stop all requested generations, start all, then validate readiness."""
    requested = tuple(sorted(expected_by_sys_id))
    with _configuration_restart_lock:
        statuses = {
            int(status["sys_id"]): status
            for status in manager.get_all_status()
        }
        unavailable = [
            sys_id
            for sys_id in requested
            if not statuses.get(sys_id, {}).get("running")
        ]
        if unavailable:
            raise ValueError(
                f"Requested NavPy instances are not running: {unavailable}"
            )
        old_pids = {
            sys_id: int(statuses[sys_id]["pid"])
            for sys_id in requested
        }
        connections = {
            sys_id: statuses[sys_id].get("connection")
            or connection_for(sys_id, settings)
            for sys_id in requested
        }
        managed_before = set(requested) & managed_sysids
        managed_sysids.difference_update(requested)
        stopped: list[int] = []
        stop_errors: list[str] = []
        started: dict[int, Any] = {}
        start_errors: list[str] = []
        try:
            for sys_id in requested:
                try:
                    if not manager.stop(sys_id):
                        raise RuntimeError("instance disappeared before stop")
                    stopped.append(sys_id)
                except Exception as error:
                    stop_errors.append(f"sys_id={sys_id}: {error}")
            for sys_id in stopped:
                try:
                    started[sys_id] = start_instance(
                        sys_id,
                        str(connections[sys_id]),
                        settings,
                    )
                except Exception as error:
                    start_errors.append(f"sys_id={sys_id}: {error}")
            lifecycle_errors = [*stop_errors, *start_errors]
            if lifecycle_errors:
                raise RuntimeError(
                    "NavPy configured restart failed: "
                    + "; ".join(lifecycle_errors)
                )
            deadline = time.monotonic() + max(float(timeout_s), 0.0)
            results: list[dict[str, object]] = []
            for sys_id in requested:
                pid = int(started[sys_id].process.pid)
                if pid == old_pids[sys_id]:
                    raise RuntimeError(
                        f"NavPy sys_id={sys_id} did not start a new process generation"
                    )
                runtime_payload = manager.wait_ready(
                    sys_id,
                    pid,
                    max(deadline - time.monotonic(), 0.0),
                )
                _validate_runtime_expectation(
                    sys_id,
                    runtime_payload,
                    expected_by_sys_id[sys_id],
                )
                results.append({
                    "sys_id": sys_id,
                    "status": "ready",
                    "old_pid": old_pids[sys_id],
                    "pid": pid,
                    "runtime": runtime_payload,
                })
            return results
        except BaseException as failure:
            cleanup_errors = _cleanup_started(manager, started)
            if cleanup_errors:
                raise BaseExceptionGroup(
                    "NavPy configured restart and rollback failed",
                    (failure, *cleanup_errors),
                ) from failure
            raise
        finally:
            managed_sysids.update(managed_before)


def _cleanup_started(
    manager: ConfiguredRestartManager,
    started: Mapping[int, Any],
) -> list[BaseException]:
    errors: list[BaseException] = []
    for sys_id, instance in started.items():
        try:
            pid = int(instance.process.pid)
            if not manager.stop_generation(sys_id, pid):
                raise RuntimeError(
                    f"replacement sys_id={sys_id} pid={pid} changed before rollback"
                )
        except BaseException as error:
            errors.append(error)
    return errors


def _validate_runtime_expectation(
    sys_id: int,
    runtime_payload: dict[str, object],
    expected_params: Mapping[str, int],
) -> None:
    try:
        mission_items = int(runtime_payload["mission_items"])
        targ_wps = int(runtime_payload["targ_wps"])
        nav_last_wp = int(runtime_payload["nav_last_wp"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(
            f"NavPy sys_id={sys_id} returned an invalid readiness payload"
        ) from error
    if mission_items <= 0:
        raise RuntimeError(f"NavPy sys_id={sys_id} loaded an empty mission")
    if targ_wps != int(expected_params["targ_wps"]):
        raise RuntimeError(
            f"NavPy sys_id={sys_id} loaded targ_wps={targ_wps}, "
            f"expected {expected_params['targ_wps']}"
        )
    if nav_last_wp != int(expected_params["nav_last_wp"]):
        raise RuntimeError(
            f"NavPy sys_id={sys_id} loaded nav_last_wp={nav_last_wp}, "
            f"expected {expected_params['nav_last_wp']}"
        )


__all__ = ["NAVPY_RESTART_READY_TIMEOUT_S", "restart_configured_instances"]
