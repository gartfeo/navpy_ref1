"""Point every RTL at the mission's default delivery hub (DDH).

After a successful mission upload the vehicle's rally table is made to match
the mission: one rally point at the DDH when the mission has one, an empty
table when it has none, so an RTL never flies to a hub from an older mission.
ArduPlane RTL goes to the nearest rally point and circles it; with no rally
point it goes to HOME (the arming point). See ``planner/rally_builder.py``.

The outcome is reported per vehicle, separately from the mission result, the
same way the geofence is: the mission itself is on the vehicle either way, but
the operator must see that an RTL would not go to the hub.
"""
from __future__ import annotations

import logging
from typing import Protocol

from navpy.core.kernel.runtime_time import LockPort
from navpy.core.ports.parameter_exchange import ExchangePolicy, ParameterExchange
from gcs.backend.exchange_budget import expired, held_within
from gcs.backend.full_params import FullParamCache
from gcs.backend.planner.rally_builder import RALLY_PARAM_WRITES, build_rally_items

log = logging.getLogger("gcs.backend.routes.missions")

#: Attempts at the rally transfer, as for the fence (``upload_fence_with_retry``).
RALLY_UPLOAD_ATTEMPTS = 3
#: RALLY_TOTAL is written by the vehicle itself on upload and on clear.
_RALLY_PARAM_KEYS = ["RALLY_TOTAL"] + [name for name, _ in RALLY_PARAM_WRITES]


class RallyVehicle(Protocol):
    """What the rally update needs from a vehicle (``VehicleMav`` provides it)."""

    target_system: int
    mission_lock: LockPort

    def write_parameter(self, name: str, value: float, policy: ExchangePolicy) -> ParameterExchange: ...

    def read_parameter(self, name: str, policy: ExchangePolicy) -> ParameterExchange: ...

    def upload_rally(self, items: list, policy: ExchangePolicy | None = None) -> bool: ...

    def clear_rally(self) -> None: ...


def _outcome(action: str, *, applied: bool, total: int | None = None,
             error: str | None = None, failed_params: list[str] | None = None) -> dict:
    return {
        "applied": applied,
        "action": action,
        "total": total,
        "error": error,
        "failed_params": failed_params or [],
    }


def _bounded(policy: ExchangePolicy | None) -> ExchangePolicy:
    """The request's policy for one parameter exchange; without one, the default."""
    return policy if policy is not None else ExchangePolicy()


def _write_rally_params(vehicle: RallyVehicle, policy: ExchangePolicy | None) -> list[str]:
    """Write the rally params; the names not confirmed by the vehicle.

    Each write is a policy-bounded exchange (``write_parameter``): its lock wait,
    its sends and its answer waits all end at the request's deadline, so a
    write that starts just before the deadline cannot retry past it (Codex
    round 1 on the rally change, finding 3).
    """
    failed: list[str] = []
    for name, value in RALLY_PARAM_WRITES:
        try:
            ok = vehicle.write_parameter(name, value, _bounded(policy)).confirmed
        except Exception as exc:  # noqa: BLE001 - reported, never raised on
            log.warning("Rally param %s write errored: %s", name, exc)
            ok = False
        if not ok:
            failed.append(name)
    return failed


def _set_hub_rally(vehicle: RallyVehicle, hub: dict, altitude_m: float,
                   policy: ExchangePolicy | None) -> dict:
    items = build_rally_items(hub, altitude_m, vehicle.target_system)
    for attempt in range(1, RALLY_UPLOAD_ATTEMPTS + 1):
        with held_within(vehicle.mission_lock, policy) as held:
            if not held:
                return _outcome("set", applied=False,
                                error="request deadline passed before the mission lock was free")
            ok = vehicle.upload_rally(items, policy=policy)
        if ok:
            break
        if expired(policy):
            return _outcome("set", applied=False, error="request deadline passed")
        log.warning("Rally upload attempt %d/%d got no ACK", attempt, RALLY_UPLOAD_ATTEMPTS)
    else:
        return _outcome("set", applied=False,
                        error=f"rally point not acknowledged after {RALLY_UPLOAD_ATTEMPTS} attempts")

    failed = _write_rally_params(vehicle, policy)
    return _outcome(
        "set", applied=not failed, total=len(items),
        error=f"parameter write failed: {', '.join(failed)}" if failed else None,
        failed_params=failed,
    )


def _clear_rally(vehicle: RallyVehicle, policy: ExchangePolicy | None) -> dict:
    with held_within(vehicle.mission_lock, policy) as held:
        if not held:
            return _outcome("clear", applied=False,
                            error="request deadline passed before the mission lock was free")
        vehicle.clear_rally()
    # MISSION_CLEAR_ALL is not awaited by the transport; the vehicle's own
    # RALLY_TOTAL, read after it on the same link, is the proof it was done.
    if expired(policy):
        return _outcome("clear", applied=False, error="request deadline passed")
    read = vehicle.read_parameter("RALLY_TOTAL", _bounded(policy))
    total = read.value if read.confirmed else None
    if total is None or total != total:  # unconfirmed, or NaN
        why = f" ({read.reason.value})" if read.reason is not None else ""
        return _outcome("clear", applied=False, error=f"RALLY_TOTAL could not be read back{why}")
    if total != 0:
        return _outcome("clear", applied=False, total=int(total),
                        error=f"vehicle still holds {int(total)} rally point(s)")
    return _outcome("clear", applied=True, total=0)


def apply_rally(vehicle: RallyVehicle, sys_id: int, hub: dict | None, altitude_m: float, *,
                cache: FullParamCache, policy: ExchangePolicy | None = None) -> dict:
    """Make one vehicle's rally table match its mission's DDH.

    ``hub`` is ``{"lat", "lon", ...}`` or None. Blocking MAVLink work: run it in
    an executor. Never raises; the returned dict is the operator-facing outcome.
    ``policy`` is the request's budget (todo 105): no vehicle work starts past it.
    """
    action = "set" if hub is not None else "clear"
    if expired(policy):
        return _outcome(action, applied=False, error="request deadline passed")
    try:
        if hub is not None:
            return _set_hub_rally(vehicle, hub, altitude_m, policy)
        return _clear_rally(vehicle, policy)
    except Exception as exc:  # noqa: BLE001 - reported per vehicle, never fails the mission
        log.warning("Rally update for vehicle %d errored: %s", sys_id, exc)
        return _outcome(action, applied=False, error=str(exc))
    finally:
        try:
            cache.invalidate_keys(sys_id, _RALLY_PARAM_KEYS)
        except Exception as exc:  # noqa: BLE001 - bookkeeping never masks the outcome
            log.warning("Rally cache invalidation for vehicle %d failed: %s", sys_id, exc)
