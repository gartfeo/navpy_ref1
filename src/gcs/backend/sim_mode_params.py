"""Flight-mode params a GCS-launched SITL vehicle must carry (sim only).

SITL's built-in RC holds channel 5 at 1800 us (ArduPilot
``AP_RCProtocol_UDP.cpp``). The demo params use channel 5 as the flight-mode
switch, so 1800 us selects position 6. With ``FLTMODE6`` = AUTOTUNE, every
re-read of the switch put a sim UAV in AUTOTUNE (seen 2026-11-05, ArduPilot
log MODE 8, reason RC). Position 6 and the start-up mode are AUTO instead
(owner ruling 2026-11-05; ``docs/design/navpy_recovery.md``). The real
aircraft's switch layout is never written from here.

A PARAM_SET is saved to the SITL eeprom, so the next boot starts right; the
values are read back to confirm.
"""
from __future__ import annotations

import logging
from collections.abc import Mapping
from typing import Protocol

log = logging.getLogger(__name__)

PLANE_MODE_AUTO = 10.0

#: Mode params for this chat's launcher-owned SITL vehicles. Shared with the
#: eval SITL roots (``scripts/eval_gcs_demo_sitl_root.py``).
SIM_MODE_SWITCH_PARAMS: Mapping[str, float] = {
    "FLTMODE6": PLANE_MODE_AUTO,
    "INITIAL_MODE": PLANE_MODE_AUTO,
}


class SimModeParamVehicle(Protocol):
    def get_parameter_fresh(self, name: str, timeout: float = 2.0) -> float | None: ...

    def set_parameter(self, name: str, value: float) -> bool: ...


def ensure_sim_mode_params(vehicle: SimModeParamVehicle, sys_id: int) -> list[str]:
    """Write each differing mode param and confirm it; return names not confirmed.

    Never raises: a failure is logged and reported, and connecting goes on.
    """
    unconfirmed: list[str] = []
    for name, wanted in SIM_MODE_SWITCH_PARAMS.items():
        try:
            if vehicle.get_parameter_fresh(name) == wanted:
                continue
            vehicle.set_parameter(name, wanted)
            confirmed = vehicle.get_parameter_fresh(name) == wanted
        except Exception as exc:  # noqa: BLE001 - reported, never raised on
            log.warning("Vehicle %d: %s write errored: %s", sys_id, name, exc)
            confirmed = False
        if confirmed:
            log.info("Vehicle %d: %s set to %.0f (sim mode switch)", sys_id, name, wanted)
        else:
            unconfirmed.append(name)
    if unconfirmed:
        log.warning("Vehicle %d: sim mode params not confirmed: %s",
                    sys_id, ", ".join(unconfirmed))
    return unconfirmed


__all__ = ["PLANE_MODE_AUTO", "SIM_MODE_SWITCH_PARAMS", "ensure_sim_mode_params"]
