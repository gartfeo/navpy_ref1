"""Acknowledged parameter writes for the upload's follow-up vehicle work."""
from __future__ import annotations

import logging

from navpy.core.ports.parameter_exchange import ExchangePolicy
from navpy.core.ports.vehicle_control_interfaces import VehicleParameters
from gcs.backend.exchange_budget import expired

log = logging.getLogger("gcs.backend.routes.missions")


def write_checked_params(vehicle: VehicleParameters, writes: list[tuple[str, float]],
                         policy: ExchangePolicy | None = None) -> list[str]:
    """Write each param and return the names the vehicle did not ack.

    Every write is attempted and its result checked: a rejected FENCE_ENABLE
    must not be reported as an applied fence just because the writes before it
    succeeded. Past the request's deadline a write is not started and counts
    as failed.
    """
    failed: list[str] = []
    for name, value in writes:
        if expired(policy):
            failed.append(name)
            continue
        try:
            ok = vehicle.set_parameter(name, value)
        except Exception as exc:  # noqa: BLE001 - reported, never raised on
            log.warning("Param %s write errored: %s", name, exc)
            ok = False
        if not ok:
            failed.append(name)
    return failed
