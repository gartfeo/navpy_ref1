"""Send one full-parameter write batch to a vehicle (see ``full_params``)."""
from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Awaitable, Callable, Optional, Sequence

from gcs.backend.full_param_values import (
    CellResult,
    ParamChange,
    coerce_to_stored,
    mav_param_type_for,
)
from gcs.backend.param_readonly import is_readonly
from navpy.plugins.vehicle.ardupilot.full_param_snapshot import FullParamSnapshot

if TYPE_CHECKING:
    from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

log = logging.getLogger("gcs.backend.full_params")


async def send_param_changes(
    vehicle: "VehicleMav",
    snapshot: FullParamSnapshot,
    changes: Sequence[ParamChange],
    *,
    per_param_timeout: float,
    progress_callback: Optional[Callable[[dict], Awaitable[None]]],
) -> tuple[dict[str, CellResult], list[str]]:
    """Each change's result, and the names that reached ``set_parameter``."""
    results: dict[str, CellResult] = {}
    total = len(changes)
    sent_names: list[str] = []
    for idx, change in enumerate(changes):
        if progress_callback is not None:
            # `idx` params already completed when starting this one.
            await progress_callback(
                {"written": idx, "total": total, "done": False})
        key = change.name.upper()
        if is_readonly(key):
            results[key] = CellResult(
                ok=False, error="read-only parameter")
            continue
        rec = snapshot.by_name.get(key)
        if rec is None:
            results[key] = CellResult(
                ok=False, error="unknown parameter")
            continue
        mav_type = mav_param_type_for(rec.ap_type)
        if mav_type is None:
            results[key] = CellResult(
                ok=False,
                error=f"unsupported AP type {rec.ap_type}",
            )
            continue
        # Honest-permissive: stage the value the autopilot would store
        # (truncate + clamp for ints), so a raw PUT of 2.1 to an int8
        # param writes 2 rather than failing with a misleading echo
        # error. Non-finite values can't be stored — reject precisely,
        # before send.
        stored = coerce_to_stored(rec.ap_type, change.value)
        if stored is None:
            results[key] = CellResult(
                ok=False,
                error=f"{change.value!r} is not a finite number",
            )
            continue
        sent_names.append(key)
        try:
            ok = await asyncio.to_thread(
                vehicle.set_parameter,
                key, stored,
                mav_param_type=mav_type,
                timeout=per_param_timeout,
            )
        except Exception as exc:
            log.exception(
                "full_params: set_parameter %s raised: %s", key, exc)
            results[key] = CellResult(
                ok=False, error=f"{type(exc).__name__}: {exc}")
            continue
        if ok:
            results[key] = CellResult(ok=True, value=stored)
        else:
            results[key] = CellResult(
                ok=False, error="echo did not match within timeout")
    if progress_callback is not None:
        await progress_callback(
            {"written": total, "total": total, "done": True})
    return results, sent_names


__all__ = ["send_param_changes"]
