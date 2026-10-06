"""Atomic slot allocation and SITL-supervisor ownership claims."""

from __future__ import annotations

from typing import Any, Callable

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry_entries as entries
from gcs.backend import instance_registry_runtime as runtime
from gcs.backend import instance_registry_store as store


class SlotBusyError(RuntimeError):
    """Raised when a directory's existing GCS launch remains active."""

    def __init__(self, entry: dict[str, Any]) -> None:
        self.entry = entry
        super().__init__(
            f"chat {entry.get('chat_index')} already has a live GCS launch "
            f"(backend_pid={entry.get('backend_pid')}, "
            f"launcher_pid={entry.get('launcher_pid')})"
        )


class SitlSupervisorActive(RuntimeError):
    """Raised when a chat already has a live swarm supervisor."""

    def __init__(self, entry: dict[str, Any]) -> None:
        self.entry = entry
        super().__init__(
            f"chat {entry.get('chat_index')} already has a live SITL supervisor "
            f"(pid={entry.get('sitl_pid')}); refusing to disrupt it"
        )


def new_entry(
    chat: int,
    label: str,
    clone: str,
    branch: str,
    owner: str,
    launcher_pid: int | None = None,
    *,
    pid_start: Callable[[object], int | None] = runtime.pid_start_time,
) -> dict[str, Any]:
    """Build the canonical persisted shape for a newly claimed chat."""
    return {
        "chat_index": chat,
        "frontend": ip.frontend_port(chat),
        "backend": ip.backend_port(chat),
        "monitor": ip.monitor_port(chat),
        "mission_planner": ip.mission_planner_port(chat),
        "sysids": ip.sysids_for_chat(chat),
        "backend_pid": None,
        "frontend_pid": None,
        "launcher_pid": launcher_pid,
        "launcher_pid_start": pid_start(launcher_pid),
        "sitl": False,
        "sitl_launch_token": None,
        "sitl_speedup": None,
        "sitl_measured_rates": None,
        "sitl_verified": None,
        "sitl_error": None,
        "clone": clone,
        "owner": owner,
        "branch": branch,
        "label": label,
        "started_at": store.now(),
    }


def claim(
    label: str = "",
    clone: str = "",
    branch: str = "",
    owner: str = "",
    prefer: int | None = None,
    lo: int = 0,
    hi: int | None = None,
    launcher_pid: int | None = None,
    *,
    port_check: Callable[[int], bool] = runtime.slot_ports_free,
    pid_start: Callable[[object], int | None] = runtime.pid_start_time,
    alive_check: Callable[[dict[str, Any]], bool] = entries.is_alive,
    launch_check: Callable[[dict[str, Any]], bool] = entries.launch_active,
    sitl_check: Callable[[dict[str, Any]], bool] = entries.sitl_alive,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> dict[str, Any]:
    """Atomically reuse or reserve a chat slot in the requested band."""
    upper = ip.MAX_CHAT_INDEX if hi is None else hi
    with access.locked():
        data = entries.prune(access.load(), alive_check=alive_check)
        reused = None
        if prefer is None:
            reused = _reuse_owned_entry(
                data,
                label=label,
                clone=clone,
                branch=branch,
                owner=owner,
                lo=lo,
                hi=upper,
                launcher_pid=launcher_pid,
                pid_start=pid_start,
                launch_check=launch_check,
                sitl_check=sitl_check,
            )
        if reused is not None:
            access.save(data)
            return reused
        chat = _select_chat(
            data,
            prefer=prefer,
            lo=lo,
            hi=upper,
            port_check=port_check,
        )
        entry = new_entry(
            chat,
            label,
            clone,
            branch,
            owner,
            launcher_pid=launcher_pid,
            pid_start=pid_start,
        )
        data["instances"][str(chat)] = entry
        access.save(data)
        return entry


def _reuse_owned_entry(
    data: dict[str, Any],
    *,
    label: str,
    clone: str,
    branch: str,
    owner: str,
    lo: int,
    hi: int,
    launcher_pid: int | None,
    pid_start: Callable[[object], int | None],
    launch_check: Callable[[dict[str, Any]], bool],
    sitl_check: Callable[[dict[str, Any]], bool],
) -> dict[str, Any] | None:
    if not owner:
        return None
    for key, entry in data["instances"].items():
        if entry.get("owner") != owner or not lo <= int(key) <= hi:
            continue
        if launcher_pid is None:
            return entry
        if launch_check(entry):
            raise SlotBusyError(entry)
        _take_over_entry(
            entry,
            label=label,
            clone=clone,
            branch=branch,
            launcher_pid=launcher_pid,
            pid_start=pid_start,
        )
        if not sitl_check(entry):
            entry.update(
                sitl=False,
                sitl_pid=None,
                sitl_pid_start=None,
                sitl_launch_token=None,
            )
        return entry
    return None


def _take_over_entry(
    entry: dict[str, Any],
    *,
    label: str,
    clone: str,
    branch: str,
    launcher_pid: int,
    pid_start: Callable[[object], int | None],
) -> None:
    entry.update(
        launcher_pid=launcher_pid,
        launcher_pid_start=pid_start(launcher_pid),
        backend_pid=None,
        backend_pid_start=None,
        frontend_pid=None,
        frontend_pid_start=None,
        clone=clone or entry.get("clone"),
        branch=branch or entry.get("branch"),
        label=label or entry.get("label"),
        started_at=store.now(),
        sitl_speedup=None,
        sitl_measured_rates=None,
        sitl_verified=None,
        sitl_error=None,
    )


def _select_chat(
    data: dict[str, Any],
    *,
    prefer: int | None,
    lo: int,
    hi: int,
    port_check: Callable[[int], bool],
) -> int:
    taken = {int(key) for key in data["instances"]}
    if prefer is not None:
        if prefer in taken:
            raise RuntimeError(
                f"chat {prefer} is already claimed by a running instance"
            )
        if not port_check(prefer):
            raise RuntimeError(f"chat {prefer} ports are busy")
        return prefer
    chat = lo
    while chat <= hi and (chat in taken or not port_check(chat)):
        chat += 1
    if chat > hi:
        raise RuntimeError(f"no free chat slot in {lo}..{hi}")
    return chat


def begin_sitl_launch(
    chat: int,
    *,
    supervisor_pid: int,
    launch_token: str | None = None,
    pid_check: Callable[[object, object], bool] = runtime.pid_matches,
    pid_start: Callable[[object], int | None] = runtime.pid_start_time,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> bool:
    """Claim SITL ownership and reset its verdict in one transaction."""
    with access.locked():
        data = access.load()
        entry = data["instances"].get(str(chat))
        if entry is None:
            return False
        if entry.get("sitl") and pid_check(
            entry.get("sitl_pid"), entry.get("sitl_pid_start")
        ):
            raise SitlSupervisorActive(entry)
        entry.update(
            sitl=True,
            sitl_pid=supervisor_pid,
            sitl_pid_start=pid_start(supervisor_pid),
            sitl_launch_token=launch_token,
            sitl_speedup=None,
            sitl_measured_rates=None,
            sitl_verified=None,
            sitl_error=None,
        )
        access.save(data)
        return True
