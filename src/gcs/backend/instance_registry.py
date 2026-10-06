"""Compatibility facade for the machine-wide GCS instance registry.

Storage, OS probing, allocation, entry liveness, and SITL verdict ownership are
implemented by focused sibling modules.  Existing callers keep importing this
module; the facade also preserves the historical private test seams.
"""

from __future__ import annotations

import os
import subprocess
from contextlib import AbstractContextManager
from typing import Any, Mapping

from gcs.backend import instance_registry_claims as claims
from gcs.backend import instance_registry_entries as entries
from gcs.backend import instance_registry_runtime as runtime
from gcs.backend import instance_registry_sitl as sitl_state
from gcs.backend import instance_registry_store as store
from gcs.backend.instance_registry_claims import (
    SitlSupervisorActive,
    SlotBusyError,
)


STARTUP_GRACE_S = entries.STARTUP_GRACE_S
_LOCK_TIMEOUT_S = store.LOCK_TIMEOUT_S
_LOCK_STALE_S = store.LOCK_STALE_S


registry_path = store.registry_path
owner_for = store.owner_for
_lock_path = store.lock_path
_lock_holder_alive = store.lock_holder_alive
_unlink_if_owner = store.unlink_if_owner


def _locked() -> AbstractContextManager[None]:
    store.LOCK_TIMEOUT_S = _LOCK_TIMEOUT_S
    store.LOCK_STALE_S = _LOCK_STALE_S
    return store.locked()


_sitl_launch_lock_path = store.sitl_launch_lock_path
sitl_launch_lock = store.sitl_launch_lock
_load = store.load
_save = store.save
_STORE_ACCESS = store.StoreAccess(
    locked=lambda: _locked(),
    load=lambda: _load(),
    save=lambda data: _save(data),
)
_port_bound = runtime.port_bound
_pid_alive = runtime.pid_alive
_pid_start_time = runtime.pid_start_time


def pid_matches(pid: object, expected_start: object = None) -> bool:
    if not _pid_alive(pid):
        return False
    if not expected_start:
        return True
    actual = _pid_start_time(pid)
    return actual is None or actual == expected_start


def _launch_active(entry: dict[str, Any]) -> bool:
    return entries.launch_active(
        entry,
        pid_check=pid_matches,
        port_check=_port_bound,
    )


def _gcs_stack_active(entry: dict[str, Any]) -> bool:
    return entries.gcs_stack_active(
        entry,
        launch_check=_launch_active,
        pid_check=pid_matches,
        port_check=_port_bound,
    )


_port_bindable = runtime.port_bindable
udp_port_held = runtime.udp_port_held


def slot_ports_free(chat: int) -> bool:
    from gcs.backend import instance_ports as ip

    return (
        _port_bindable(ip.frontend_port(chat), "tcp")
        and _port_bindable(ip.backend_port(chat), "tcp")
        and _port_bindable(ip.monitor_port(chat), "udp")
        and _port_bindable(ip.mission_planner_port(chat), "udp")
        and not any(
            udp_port_held(port) for port in ip.companion_ports_for_chat(chat)
        )
    )


listeners_on_port = runtime.listeners_on_port


def reap_port(
    port: int,
    kind: str = "tcp",
    exclude: set[int] | None = None,
) -> list[int]:
    return runtime.reap_port(
        port,
        kind=kind,
        exclude=exclude,
        listener_lookup=lambda value, mode: listeners_on_port(value, mode),
    )


def reap_companion_ports(
    chat: int,
    exclude: set[int] | None = None,
) -> list[int]:
    return runtime.reap_companion_ports(
        chat,
        exclude=exclude,
        port_reaper=lambda value, mode, protected: reap_port(
            value,
            kind=mode,
            exclude=protected,
        ),
    )


def registered_pids() -> set[int]:
    return entries.registered_pids(live_entries=live)


_now = store.now


def _is_alive(entry: dict[str, Any]) -> bool:
    return entries.is_alive(
        entry,
        port_check=_port_bound,
        pid_check=pid_matches,
        sitl_check=_sitl_alive,
    )


def _sitl_alive(entry: dict[str, Any]) -> bool:
    return entries.sitl_alive(
        entry,
        pid_check=pid_matches,
        udp_check=udp_port_held,
        port_check=_port_bound,
    )


def _prune(data: dict[str, Any]) -> dict[str, Any]:
    return entries.prune(data, alive_check=_is_alive)


def _new_entry(
    chat: int,
    label: str,
    clone: str,
    branch: str,
    owner: str,
    launcher_pid: int | None = None,
) -> dict[str, Any]:
    return claims.new_entry(
        chat,
        label,
        clone,
        branch,
        owner,
        launcher_pid,
        pid_start=_pid_start_time,
    )


def find_for_owner(owner: str, label: str | None = None) -> dict[str, Any] | None:
    return entries.find_for_owner(
        owner, label, alive_check=_is_alive, access=_STORE_ACCESS
    )


def claim(
    label: str = "",
    clone: str = "",
    branch: str = "",
    owner: str = "",
    prefer: int | None = None,
    lo: int = 0,
    hi: int | None = None,
    launcher_pid: int | None = None,
) -> dict[str, Any]:
    return claims.claim(
        label,
        clone,
        branch,
        owner,
        prefer,
        lo,
        hi,
        launcher_pid,
        port_check=slot_ports_free,
        pid_start=_pid_start_time,
        alive_check=_is_alive,
        launch_check=_launch_active,
        sitl_check=_sitl_alive,
        access=_STORE_ACCESS,
    )


def begin_sitl_launch(
    chat: int,
    *,
    supervisor_pid: int,
    launch_token: str | None = None,
) -> bool:
    return claims.begin_sitl_launch(
        chat,
        supervisor_pid=supervisor_pid,
        launch_token=launch_token,
        pid_check=pid_matches,
        pid_start=_pid_start_time,
        access=_STORE_ACCESS,
    )


def record_pids(
    chat: int,
    *,
    backend_pid: int | None = None,
    frontend_pid: int | None = None,
    sitl: bool | None = None,
    sitl_pid: int | None = None,
) -> None:
    sitl_state.record_pids(
        chat,
        backend_pid=backend_pid,
        frontend_pid=frontend_pid,
        sitl=sitl,
        sitl_pid=sitl_pid,
        pid_start=_pid_start_time,
        access=_STORE_ACCESS,
    )


def _normalized_rates(
    measured_rates: Mapping[object, object] | None,
) -> dict[str, float] | None:
    return sitl_state.normalized_rates(measured_rates)


def record_sitl_status(
    chat: int,
    speedup: object,
    verified: bool,
    *,
    sitl_pid: object,
    error: str | None = None,
    measured_rates: Mapping[object, object] | None = None,
) -> str:
    return sitl_state.record_sitl_status(
        chat,
        speedup,
        verified,
        sitl_pid=sitl_pid,
        error=error,
        measured_rates=measured_rates,
        access=_STORE_ACCESS,
    )


def retract_sitl(chat: int, *, sitl_pid: int) -> str:
    return sitl_state.retract_sitl(
        chat,
        sitl_pid=sitl_pid,
        gcs_active=_gcs_stack_active,
        access=_STORE_ACCESS,
    )


def release(chat: int) -> None:
    entries.release(chat, access=_STORE_ACCESS)


def get(chat: int) -> dict[str, Any] | None:
    return entries.get(chat, access=_STORE_ACCESS)


def live() -> list[dict[str, Any]]:
    return entries.live(alive_check=_is_alive, access=_STORE_ACCESS)
