"""SITL process identity and launch-verdict lifecycle operations."""

from __future__ import annotations

import math
from typing import Any, Callable, Literal, Mapping

from gcs.backend import instance_registry_entries as entries
from gcs.backend import instance_registry_runtime as runtime
from gcs.backend import instance_registry_store as store


SitlStatusOutcome = Literal["recorded", "missing", "not-ours"]
SitlRetractionOutcome = Literal["missing", "not-ours", "cleared", "released"]


def record_pids(
    chat: int,
    *,
    backend_pid: int | None = None,
    frontend_pid: int | None = None,
    sitl: bool | None = None,
    sitl_pid: int | None = None,
    pid_start: Callable[[object], int | None] = runtime.pid_start_time,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> None:
    """Persist process identities and reset evidence for a new SITL PID."""
    with access.locked():
        data = access.load()
        entry = data["instances"].get(str(chat))
        if entry is None:
            return
        if backend_pid is not None:
            entry["backend_pid"] = backend_pid
            entry["backend_pid_start"] = pid_start(backend_pid)
        if frontend_pid is not None:
            entry["frontend_pid"] = frontend_pid
            entry["frontend_pid_start"] = pid_start(frontend_pid)
        if sitl is not None:
            entry["sitl"] = sitl
        if sitl_pid is not None:
            _replace_sitl_generation(entry, sitl_pid=sitl_pid, pid_start=pid_start)
        access.save(data)


def _replace_sitl_generation(
    entry: dict[str, Any],
    *,
    sitl_pid: int,
    pid_start: Callable[[object], int | None],
) -> None:
    entry.update(
        sitl_pid=sitl_pid,
        sitl_pid_start=pid_start(sitl_pid),
        sitl_launch_token=None,
        sitl_speedup=None,
        sitl_measured_rates=None,
        sitl_verified=None,
        sitl_error=None,
    )


def normalized_rates(
    measured_rates: Mapping[object, object] | None,
) -> dict[str, float] | None:
    """Return finite measured rates in their JSON round-trip shape."""
    if not measured_rates:
        return None
    normalized: dict[str, float] = {}
    for sysid, rate in measured_rates.items():
        if isinstance(rate, bool) or not isinstance(rate, (int, float)):
            continue
        value = float(rate)
        if math.isfinite(value):
            normalized[str(sysid)] = value
    return normalized or None


def record_sitl_status(
    chat: int,
    speedup: object,
    verified: bool,
    *,
    sitl_pid: object,
    error: str | None = None,
    measured_rates: Mapping[object, object] | None = None,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> SitlStatusOutcome:
    """Persist a final launch verdict only for its owning supervisor."""
    with access.locked():
        data = access.load()
        entry = data["instances"].get(str(chat))
        if entry is None:
            return "missing"
        if not sitl_pid or entry.get("sitl_pid") != sitl_pid:
            return "not-ours"
        entry.update(
            sitl_speedup=speedup,
            sitl_measured_rates=normalized_rates(measured_rates),
            sitl_verified=verified,
            sitl_error=error,
        )
        access.save(data)
        return "recorded"


def retract_sitl(
    chat: int,
    *,
    sitl_pid: int,
    gcs_active: Callable[[dict[str, Any]], bool] = entries.gcs_stack_active,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> SitlRetractionOutcome:
    """Clear a failed launch without discarding a still-live GCS stack."""
    with access.locked():
        data = access.load()
        entry = data["instances"].get(str(chat))
        if entry is None:
            return "missing"
        if not sitl_pid or entry.get("sitl_pid") != sitl_pid:
            return "not-ours"
        entry.update(
            sitl=False,
            sitl_pid=None,
            sitl_pid_start=None,
            sitl_launch_token=None,
        )
        if gcs_active(entry):
            access.save(data)
            return "cleared"
        data["instances"].pop(str(chat), None)
        access.save(data)
        return "released"
