"""Launch telemetry, airborne confirmation and spacing predicates."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from gcs.backend.launch_state import VehicleState

if TYPE_CHECKING:
    from gcs.backend.launch_controller import LaunchController

log = logging.getLogger("gcs.backend.launch_controller")

def update_telemetry(controller: LaunchController, sys_id: int, alt_rel: float, armed: bool,
                     climb: float = 0.0, throttle: float = 0.0) -> None:
    prev_alt = controller._altitudes.get(sys_id)
    prev_armed = controller._armed.get(sys_id)
    controller._altitudes[sys_id] = alt_rel
    controller._armed[sys_id] = armed
    controller._climb_rates[sys_id] = climb
    controller._throttles[sys_id] = throttle
    # Track sustained climb for the airborne predicate. Only meaningful when
    # climb confirmation is enabled (min_climb_rate_ms > 0).
    if controller._min_climb_rate_ms > 0:
        if climb >= controller._min_climb_rate_ms:
            if controller._climb_ok_since.get(sys_id) is None:
                controller._climb_ok_since[sys_id] = time.monotonic()
        else:
            controller._climb_ok_since[sys_id] = None
    # Log significant telemetry changes during active launch
    if sys_id in controller._states and controller._states[sys_id] not in (
        VehicleState.idle, VehicleState.airborne, VehicleState.failed
    ):
        if prev_armed != armed:
            log.info("[launch] V%d telemetry: armed changed %s -> %s", sys_id, prev_armed, armed)
        if prev_alt is not None and abs(alt_rel - prev_alt) > 1.0:
            log.info("[launch] V%d telemetry: alt_rel=%.1f m", sys_id, alt_rel)

def _is_airborne(controller: LaunchController, sys_id: int) -> bool:
    """Predicate: is this vehicle considered airborne / successfully launched?

    Altitude above threshold, and — when ``require_armed`` is set — also
    armed. Sustained-climb confirmation is layered on in a later step.
    """
    if controller._altitudes.get(sys_id, 0.0) < controller._altitude_threshold:
        return False
    if controller._require_armed and not controller._armed.get(sys_id, False):
        return False
    if controller._require_throttle and controller._throttles.get(sys_id, 0.0) < controller._min_throttle_pct:
        # Throttle commanded confirms the powertrain is driving (motor/prop turning).
        return False
    if controller._min_climb_rate_ms > 0:
        # Require climb sustained >= min_climb_rate_ms for climb_confirm_s.
        # Missing/insufficient climb leaves climb_ok_since None → not airborne
        # (the altitude timeout is the fail-safe).
        since = controller._climb_ok_since.get(sys_id)
        if since is None:
            return False
        if (time.monotonic() - since) < controller._climb_confirm_s:
            return False
    return True

def _has_pending_vehicle(controller: LaunchController) -> bool:
    """True if any vehicle is still waiting to launch (idle/queued)."""
    return any(
        s in (VehicleState.idle, VehicleState.queued)
        for s in controller._states.values()
    )

def _post_vehicle_delay(controller: LaunchController, sys_id: int) -> float:
    """Delay to hold the launch lock after *sys_id* finishes, before the
    next queued vehicle starts.

    Returns the larger container gap at a container boundary — this
    vehicle's container has no more pending members and a later container is
    still waiting — otherwise the inter-vehicle stagger.
    """
    if not controller._container_of:
        return controller._stagger_s
    my_c = controller._container_of.get(sys_id)
    if my_c is None:
        return controller._stagger_s
    pending = (VehicleState.idle, VehicleState.queued)
    same_container_pending = False
    later_container_pending = False
    for sid, c in controller._container_of.items():
        if sid == sys_id or controller._states.get(sid) not in pending:
            continue
        if c == my_c:
            same_container_pending = True
        elif c > my_c:
            later_container_pending = True
    if same_container_pending:
        return controller._stagger_s
    if later_container_pending:
        return controller._container_gap_s
    return controller._stagger_s
