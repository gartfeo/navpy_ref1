"""Prepare a launch session and validate its trigger connection."""
from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Callable, Awaitable

from gcs.backend.launch_state import VehicleState

if TYPE_CHECKING:
    from gcs.backend.launch_controller import LaunchController

log = logging.getLogger("gcs.backend.launch_controller")

from gcs.backend.launch_state import _container_map


async def prepare(
    controller: LaunchController,
    sys_ids: list[int],
    channel_map: dict[int, int],
    esp32_host: str,
    esp32_port: int,
    altitude_threshold: float,
    arm_timeout: float,
    altitude_timeout: float,
    arm_func: Callable[[int], None],
    auto_func: Callable[[int], None],
    settle_s: float = 0.3,
    stagger_s: float = 0.0,
    container_gap_s: float = 0.0,
    uavs_per_container: int = 0,
    require_armed: bool = False,
    require_throttle: bool = False,
    min_throttle_pct: float = 20.0,
    min_climb_rate_ms: float = 0.0,
    climb_confirm_s: float = 0.0,
    *, broadcast: Callable[[dict], Awaitable[None]],
) -> None:
    """Initialize launch session: health check ESP32, set all to idle.

    Does NOT launch any vehicles. Call trigger_vehicle() individually.
    """
    log.info("[launch] prepare() called: sys_ids=%s, channel_map=%s, "
             "alt_thresh=%.1f, arm_timeout=%.1f, alt_timeout=%.1f, "
             "settle=%.2f, stagger=%.2f, require_armed=%s",
             sys_ids, channel_map, altitude_threshold, arm_timeout,
             altitude_timeout, settle_s, stagger_s, require_armed)
    if controller._prepared:
        log.warning("[launch] prepare() called but already prepared, ignoring")
        return
    controller._abort_event.clear()
    controller._abort_notified = False
    controller._states.clear()
    controller._errors.clear()
    controller._tasks.clear()

    for sid in sys_ids:
        controller._states[sid] = VehicleState.idle

    # Store session parameters
    controller._channel_map = channel_map
    controller._altitude_threshold = altitude_threshold
    controller._arm_timeout = arm_timeout
    controller._altitude_timeout = altitude_timeout
    controller._settle_s = settle_s
    controller._stagger_s = stagger_s
    controller._container_gap_s = container_gap_s
    # Map each sys_id to a container index (grouped by ascending sys_id) so
    # the container gap can be applied at container boundaries. Left empty
    # (no gaps) when disabled or the group size is unset.
    controller._container_of = (
        _container_map(sys_ids, uavs_per_container) if container_gap_s > 0 else {}
    )
    controller._require_armed = require_armed
    controller._require_throttle = require_throttle
    controller._min_throttle_pct = min_throttle_pct
    controller._min_climb_rate_ms = min_climb_rate_ms
    controller._climb_confirm_s = climb_confirm_s
    controller._climb_ok_since.clear()  # fresh climb-confirmation state per session
    controller._arm_func = arm_func
    controller._auto_func = auto_func

    # Create and health-check ESP32 client
    loop = asyncio.get_running_loop()
    from navpy.modules.comm.esp32_trigger import Esp32TriggerClient
    controller._esp32_client = Esp32TriggerClient(esp32_host, esp32_port)
    healthy = await loop.run_in_executor(None, controller._esp32_client.health_check)
    if not healthy:
        for sid in sys_ids:
            await controller._set_state(sid, VehicleState.failed, "ESP32 not reachable")
        await broadcast({"type": "launch_complete"})
        return

    controller._prepared = True
    log.info("Launch session prepared for vehicles: %s", sys_ids)
