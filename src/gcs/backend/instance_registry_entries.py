"""Registry entry liveness, lookup, and basic lifecycle operations."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Callable

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry_runtime as runtime
from gcs.backend import instance_registry_store as store


STARTUP_GRACE_S = 45.0


def launch_active(
    entry: dict[str, Any],
    *,
    pid_check: Callable[[object, object], bool] = runtime.pid_matches,
    port_check: Callable[[int], bool] = runtime.port_bound,
) -> bool:
    """Return whether a GCS backend is running or still launching."""
    if pid_check(
        entry.get("launcher_pid"), entry.get("launcher_pid_start")
    ) or pid_check(
        entry.get("backend_pid"), entry.get("backend_pid_start")
    ):
        return True
    backend = entry.get("backend")
    return bool(backend) and port_check(int(backend))


def gcs_stack_active(
    entry: dict[str, Any],
    *,
    launch_check: Callable[[dict[str, Any]], bool] = launch_active,
    pid_check: Callable[[object, object], bool] = runtime.pid_matches,
    port_check: Callable[[int], bool] = runtime.port_bound,
) -> bool:
    """Return whether any GCS process or port still owns an entry."""
    if launch_check(entry) or pid_check(
        entry.get("frontend_pid"), entry.get("frontend_pid_start")
    ):
        return True
    frontend = entry.get("frontend")
    return bool(frontend) and port_check(int(frontend))


def is_alive(
    entry: dict[str, Any],
    *,
    port_check: Callable[[int], bool] = runtime.port_bound,
    pid_check: Callable[[object, object], bool] = runtime.pid_matches,
    sitl_check: Callable[[dict[str, Any]], bool] | None = None,
) -> bool:
    """Return whether an entry must remain discoverable and stoppable."""
    started = entry.get("started_at")
    if started:
        try:
            age = (
                datetime.now(timezone.utc) - datetime.fromisoformat(str(started))
            ).total_seconds()
            if age < STARTUP_GRACE_S:
                return True
        except (ValueError, TypeError):
            pass
    backend = entry.get("backend")
    if bool(backend) and port_check(int(backend)):
        return True
    if pid_check(
        entry.get("backend_pid"), entry.get("backend_pid_start")
    ):
        return True
    return (sitl_check or sitl_alive)(entry)


def sitl_alive(
    entry: dict[str, Any],
    *,
    pid_check: Callable[[object, object], bool] = runtime.pid_matches,
    udp_check: Callable[[int], bool] = runtime.udp_port_held,
    port_check: Callable[[int], bool] = runtime.port_bound,
) -> bool:
    """Return whether the entry's SITL swarm is still running."""
    if not entry.get("sitl"):
        return False
    if pid_check(
        entry.get("sitl_pid"), entry.get("sitl_pid_start")
    ):
        return True
    return any(
        udp_check(ip.companion_port(int(sysid)))
        or port_check(ip.companion_port(int(sysid)))
        for sysid in entry.get("sysids", ())
    )


def prune(
    data: dict[str, Any],
    *,
    alive_check: Callable[[dict[str, Any]], bool] = is_alive,
) -> dict[str, Any]:
    """Remove dead entries from a loaded registry document."""
    data["instances"] = {
        key: entry
        for key, entry in data["instances"].items()
        if alive_check(entry)
    }
    return data


def find_for_owner(
    owner: str,
    label: str | None = None,
    *,
    alive_check: Callable[[dict[str, Any]], bool] = is_alive,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> dict[str, Any] | None:
    """Return the live entry owned by a directory, optionally in one band."""
    if not owner:
        return None
    band = ip.band_for_label(label) if label is not None else None
    with access.locked():
        data = prune(access.load(), alive_check=alive_check)
        access.save(data)
        for key in sorted(data["instances"], key=int):
            entry = data["instances"][key]
            if entry.get("owner") != owner:
                continue
            if band is not None and not ip.in_band(int(key), band):
                continue
            return entry
    return None


def registered_pids(
    *,
    live_entries: Callable[[], list[dict[str, Any]]] | None = None,
) -> set[int]:
    """Return every process PID protected by a live registry entry."""
    pids: set[int] = set()
    for entry in (live_entries or live)():
        for key in ("backend_pid", "frontend_pid", "sitl_pid"):
            pid = entry.get(key)
            if pid:
                pids.add(int(pid))
    return pids


def release(
    chat: int,
    *,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> None:
    """Remove one registry entry atomically."""
    with access.locked():
        data = access.load()
        data["instances"].pop(str(chat), None)
        access.save(data)


def get(
    chat: int,
    *,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> dict[str, Any] | None:
    """Return one entry from the atomically replaced registry document."""
    return access.load()["instances"].get(str(chat))


def live(
    *,
    alive_check: Callable[[dict[str, Any]], bool] = is_alive,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> list[dict[str, Any]]:
    """Prune, persist, and return live entries ordered by chat index."""
    with access.locked():
        data = prune(access.load(), alive_check=alive_check)
        access.save(data)
        return [
            data["instances"][key]
            for key in sorted(data["instances"], key=int)
        ]
