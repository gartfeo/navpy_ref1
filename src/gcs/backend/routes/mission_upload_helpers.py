"""Mission geometry, geofence application and outcome projection."""
from __future__ import annotations

import asyncio
import logging
import math
from typing import Callable
from navpy.modules.vehicle.vehicle_control_interfaces import VehicleParameters
from gcs.backend.models import VehicleAssignment, FencePlan, FenceOutcome
from gcs.backend.vehicle_manager import VehicleEntry
from gcs.backend.full_params import FullParamCache
from gcs.backend.planner.fence_builder import MIN_FENCE_VERTICES, FenceUploadResult

log = logging.getLogger("gcs.backend.routes.missions")
# FENCE_TOTAL is written by the vehicle itself on upload; never set it here.
_FENCE_PARAM_KEYS = ["FENCE_TYPE", "FENCE_ACTION", "FENCE_AUTOENABLE", "FENCE_ENABLE"]

# Raw fence parameters mirrored back to the operator. The vehicle's own values
# are the only truth about its fence mode: FENCE_ENABLE=0 with
# FENCE_AUTOENABLE>0 is "arms the fence at takeoff", which is NOT "off".
_FENCE_READBACK_PARAMS = {
    "enable": "FENCE_ENABLE",
    "autoenable": "FENCE_AUTOENABLE",
    "type": "FENCE_TYPE",
    "action": "FENCE_ACTION",
}


def assignment_geometry(
    assignment: VehicleAssignment,
) -> tuple[list[dict], list[dict] | None, list[dict] | None, dict | None, dict | None]:
    """Convert the validated request's geometry to planner inputs."""
    track = [{"lat": wp.lat, "lon": wp.lon} for wp in assignment.waypoints]
    poly = [{"lat": p.lat, "lon": p.lon} for p in assignment.polygon] if assignment.polygon else None
    corr = [{"lat": p.lat, "lon": p.lon} for p in assignment.corridor_backbone] if assignment.corridor_backbone else None
    lp = {"lat": assignment.launch_point.lat, "lon": assignment.launch_point.lon} if assignment.launch_point else None
    if assignment.fallback_delivery_location:
        dt = {
            "lat": assignment.fallback_delivery_location.lat,
            "lon": assignment.fallback_delivery_location.lon,
        }
        if assignment.fallback_delivery_location.type:
            dt["type"] = assignment.fallback_delivery_location.type
    else:
        dt = None

    return track, poly, corr, lp, dt


def read_fence_params(vehicle: VehicleParameters) -> dict | None:
    """Read the vehicle's four raw fence params, fresh.

    Returns the raw values, or None when ANY of them could not be read. A
    partial answer is not a fence mode, and a missing value must never be
    filled in with 0 — downstream that reads as "fence off" on a vehicle whose
    fence may well be armed. Blocking MAVLink reads, so call it in an executor.
    """
    values: dict[str, float] = {}
    for key, name in _FENCE_READBACK_PARAMS.items():
        try:
            raw = vehicle.get_parameter_fresh(name)
        except Exception as exc:  # noqa: BLE001 - an unreadable param is "unknown"
            log.warning("Fence param %s readback failed: %s", name, exc)
            return None
        if raw is None:
            return None
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return None
        # NaN/Infinity are not a fence mode, and JSON cannot carry them: letting
        # one through turned the whole readback into a 500, so the operator saw
        # nothing rather than "unknown".
        if not math.isfinite(value):
            log.warning("Fence param %s read back non-finite (%r)", name, raw)
            return None
        values[key] = value
    return values


def _write_fence_params(vehicle: VehicleParameters, writes: list[tuple[str, float]]) -> list[str]:
    """Write each fence param and return the names the vehicle did not ack.

    Every write is attempted and its result checked: a rejected FENCE_ENABLE
    must not be reported as an applied fence just because the writes before it
    succeeded.
    """
    failed: list[str] = []
    for name, value in writes:
        try:
            ok = vehicle.set_parameter(name, value)
        except Exception as exc:  # noqa: BLE001 - reported, never raised on
            log.warning("Fence param %s write errored: %s", name, exc)
            ok = False
        if not ok:
            failed.append(name)
    return failed


def _outcome(action: str, *, applied: bool, enabled: bool = False,
             total: int | None = None, error: str | None = None,
             failed_params: list[str] | None = None) -> dict:
    return {
        # Kept for the existing upload_progress websocket payload.
        "ok": applied,
        "applied": applied,
        "action": action,
        "enabled": enabled,
        "total": total,
        "error": error,
        "failed_params": failed_params or [],
    }


async def apply_fence(
    loop: asyncio.AbstractEventLoop,
    entry: VehicleEntry,
    sys_id: int,
    fence: FencePlan | None,
    *,
    upload_fence: Callable[..., FenceUploadResult],
    cache: FullParamCache,
) -> dict | None:
    """Upload the shared geofence to one vehicle and set its fence params.

    Returns a structured outcome dict, or None when no fence was requested.
    Never raises — fence problems are reported but must not fail the mission
    upload. The caller reports the outcome separately from the mission result
    so a partially applied fence is visible even on a successful mission.
    """
    if fence is None:
        return None
    # `upload_fence` CLEARS the vehicle's stored fence before writing the new
    # one, so once it has been called the vehicle's geometry has changed
    # whatever the outcome — and FENCE_TOTAL with it. Any exit after that point
    # invalidates the cached fence values. This records an attempt, not a
    # rollback: nothing here restores the fence the vehicle held before.
    geometry_attempted = False

    def _invalidate_attempted_geometry() -> None:
        if geometry_attempted:
            cache.invalidate_keys(sys_id, _FENCE_PARAM_KEYS + ["FENCE_TOTAL"])

    try:
        if not fence.enabled:
            # Explicitly disable: ENABLE=0 and AUTOENABLE=0 only — otherwise
            # the fence re-enables on takeoff regardless of FENCE_ENABLE=0.
            # The stored ring, FENCE_TYPE and FENCE_ACTION are deliberately
            # left as the vehicle has them, so re-enabling later restores the
            # operator's own configuration rather than a guess.
            failed = _write_fence_params(
                entry.vehicle, [("FENCE_ENABLE", 0.0), ("FENCE_AUTOENABLE", 0.0)],
            )
            # The vehicle's values changed even when one write was rejected.
            cache.invalidate_keys(sys_id, ["FENCE_ENABLE", "FENCE_AUTOENABLE"])
            return _outcome(
                "disable", applied=not failed, enabled=False,
                error=f"parameter write failed: {', '.join(failed)}" if failed else None,
                failed_params=failed,
            )

        verts = [{"lat": v.lat, "lon": v.lon} for v in fence.vertices]
        if len(verts) < MIN_FENCE_VERTICES:
            # Reject, never downgrade to a disable: silently switching off a
            # fence the operator asked to switch ON is the dangerous direction.
            return _outcome(
                "rejected", applied=False,
                error=f"fence needs >= {MIN_FENCE_VERTICES} vertices",
            )

        exclusions = [
            [{"lat": v.lat, "lon": v.lon} for v in ring]
            for ring in (fence.exclusions or [])
        ]
        geometry_attempted = True
        res = await loop.run_in_executor(
            None, lambda: upload_fence(entry.vehicle, verts, exclusions),
        )
        if not res.success:
            log.warning("Fence upload to vehicle %d failed: %s", sys_id, res.error)
            _invalidate_attempted_geometry()
            return _outcome("enable", applied=False, error=res.error)

        # Enforce as an inclusion polygon with RTL on breach. FENCE_AUTOENABLE=1
        # (enable on takeoff) keeps the fence from blocking pre-arm on an
        # auto-launch UAV; validated in live SITL.
        failed = _write_fence_params(entry.vehicle, [
            ("FENCE_TYPE", float(fence.type)),
            ("FENCE_ACTION", float(fence.action)),
            ("FENCE_AUTOENABLE", 1.0),
            ("FENCE_ENABLE", 1.0),
        ])
        cache.invalidate_keys(sys_id, _FENCE_PARAM_KEYS + ["FENCE_TOTAL"])
        return _outcome(
            "enable", applied=not failed, enabled=not failed, total=res.vertex_count,
            error=f"parameter write failed: {', '.join(failed)}" if failed else None,
            failed_params=failed,
        )
    except Exception as e:  # noqa: BLE001 - fence must never fail the mission
        log.warning("Fence application to vehicle %d errored: %s", sys_id, e)
        try:
            _invalidate_attempted_geometry()
        except Exception as inval_exc:  # noqa: BLE001 - reported, never masks e
            log.warning("Fence cache invalidation for vehicle %d failed: %s",
                        sys_id, inval_exc)
        # The original failure is what the operator is told about; the
        # invalidation is bookkeeping and never replaces it.
        return _outcome("enable" if fence.enabled else "disable",
                        applied=False, error=str(e))


def fence_outcome_model(status: dict | None) -> FenceOutcome | None:
    """Project the helper's status dict onto the response model.

    The dict keeps its legacy ``ok`` key for the existing upload_progress
    websocket payload; the HTTP result carries the structured outcome.
    """
    if status is None:
        return None
    return FenceOutcome(
        applied=bool(status.get("applied", status.get("ok", False))),
        action=status.get("action", "enable"),
        enabled=bool(status.get("enabled", False)),
        total=status.get("total"),
        error=status.get("error"),
        failed_params=list(status.get("failed_params") or []),
    )
