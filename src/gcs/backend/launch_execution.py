"""Serialized per-vehicle launch execution and abort-aware waits."""
from __future__ import annotations

import asyncio
import logging
import time
from typing import TYPE_CHECKING, Callable

from gcs.backend.launch_state import VehicleState, _ALREADY_AIRBORNE_MIN_ALT_M

if TYPE_CHECKING:
    from gcs.backend.launch_controller import LaunchController

log = logging.getLogger("gcs.backend.launch_controller")

async def _run_vehicle(controller: LaunchController, sys_id: int) -> None:
    """Execute auto -> arm -> trigger -> wait altitude for one vehicle.

    Acquires _launch_lock so only one vehicle runs at a time.
    """
    queued_at = time.monotonic()
    if controller._launch_lock.locked():
        active = [sid for sid, st in controller._states.items()
                  if st in (VehicleState.arming, VehicleState.armed, VehicleState.launching)]
        log.info("[launch] V%d QUEUED — launch lock held (active=%s)", sys_id, active)
        await controller._set_state(sys_id, VehicleState.queued)
    log.info("[launch] V%d waiting for launch lock...", sys_id)
    try:
        async with controller._launch_lock:
            log.info("[launch] V%d ACQUIRED launch lock after %.1fs (all states=%s)",
                     sys_id, time.monotonic() - queued_at,
                     {sid: st.value for sid, st in controller._states.items()})
            if controller._abort_event.is_set():
                await controller._abort_vehicle(sys_id)
                return
            try:
                await controller._run_vehicle_locked(sys_id)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.exception("[launch] V%d unexpected launch error", sys_id)
                await controller._fail_session(sys_id, f"Launch error: {exc}")
            final = controller._states.get(sys_id)
            log.info("[launch] V%d RELEASING launch lock (final state=%s) — next queued vehicle may now start",
                     sys_id, final.value if final else "?")
    finally:
        controller._finish_aborted_session_if_quiescent()

async def _run_vehicle_locked(controller: LaunchController, sys_id: int) -> None:
    """Inner launch sequence — runs while holding _launch_lock."""
    loop = asyncio.get_running_loop()
    channel = controller._channel_map.get(sys_id, sys_id)

    log.info("[launch] V%d _run_vehicle START  channel=%d  alt=%.1f  armed=%s",
             sys_id, channel,
             controller._altitudes.get(sys_id, 0.0),
             controller._armed.get(sys_id, False))

    # Already flying — set AUTO, arm if needed, skip the ESP32 trigger.
    # Gated on a FIXED altitude floor (not the tunable confirmation
    # threshold, which may be 0/low) so a grounded vehicle never falsely
    # skips its launch.
    if controller._altitudes.get(sys_id, 0.0) >= _ALREADY_AIRBORNE_MIN_ALT_M:
        log.warning("[launch] V%d already flying (alt %.1f >= floor %.1f) — "
                    "skipping ESP32 trigger and marking airborne.",
                    sys_id, controller._altitudes.get(sys_id, 0.0), _ALREADY_AIRBORNE_MIN_ALT_M)
        controller._auto_func(sys_id)
        if not controller._armed.get(sys_id, False):
            await controller._set_state(sys_id, VehicleState.arming)
            controller._arm_func(sys_id)
            arm_ok = await controller._poll(
                lambda: controller._armed.get(sys_id, False),
                controller._arm_timeout,
            )
            if arm_ok is None:
                await controller._abort_vehicle(sys_id)
                return
            if not arm_ok:
                log.error("[launch] V%d ARM TIMEOUT (airborne path)", sys_id)
                await controller._fail_session(sys_id, "Arm timeout")
                return
        await controller._set_state(sys_id, VehicleState.airborne)
        log.info("[launch] V%d already airborne, set AUTO and armed", sys_id)
        await controller._check_all_done()
        return

    # Step 1: Set AUTO mode
    log.info("[launch] V%d step 1: setting AUTO mode", sys_id)
    controller._auto_func(sys_id)
    if controller._settle_s > 0:
        # Abort-aware so a tuned-up settle can't hold an abort.
        await controller._sleep_or_abort(controller._settle_s)
        if controller._abort_event.is_set():
            await controller._abort_vehicle(sys_id)
            return

    # Step 2: Arm
    if controller._armed.get(sys_id, False):
        log.info("[launch] V%d step 2: already armed, skipping arm", sys_id)
        await controller._set_state(sys_id, VehicleState.armed)
    else:
        log.info("[launch] V%d step 2: sending arm command (timeout=%.1fs)",
                 sys_id, controller._arm_timeout)
        await controller._set_state(sys_id, VehicleState.arming)
        controller._arm_func(sys_id)
        arm_ok = await controller._poll(
            lambda: controller._armed.get(sys_id, False),
            controller._arm_timeout,
        )
        if arm_ok is None:
            await controller._abort_vehicle(sys_id)
            return
        if not arm_ok:
            log.error("[launch] V%d ARM TIMEOUT after %.1fs", sys_id, controller._arm_timeout)
            await controller._fail_session(sys_id, "Arm timeout")
            return
        log.info("[launch] V%d armed successfully", sys_id)
        await controller._set_state(sys_id, VehicleState.armed)

    # Step 3: ESP32 trigger
    log.info("[launch] V%d step 3: triggering ESP32 channel %d", sys_id, channel)
    await controller._set_state(sys_id, VehicleState.launching)
    result = await loop.run_in_executor(
        None, controller._esp32_client.trigger_channel, channel,
    )
    if not result.success:
        log.error("[launch] V%d ESP32 trigger FAILED: %s", sys_id, result.message)
        await controller._fail_session(sys_id, f"Trigger failed: {result.message}")
        return
    log.info("[launch] V%d ESP32 trigger OK (duration=%dms)", sys_id, result.duration_ms)

    # Restrict sustained-climb confirmation to the POST-trigger window so the
    # required duration can't be pre-satisfied during prepare/arming.
    if controller._min_climb_rate_ms > 0:
        controller._climb_ok_since[sys_id] = None

    # Step 4: Wait for airborne confirmation (altitude + optional armed)
    log.info("[launch] V%d step 4: waiting for airborne (alt >= %.1fm, "
             "require_armed=%s, timeout=%.1fs)",
             sys_id, controller._altitude_threshold, controller._require_armed,
             controller._altitude_timeout)
    alt_ok = await controller._poll(
        lambda sid=sys_id: controller._is_airborne(sid),
        controller._altitude_timeout,
    )
    if alt_ok is None:
        await controller._abort_vehicle(sys_id)
        return
    if alt_ok:
        log.info("[launch] V%d AIRBORNE (alt=%.1f)", sys_id,
                 controller._altitudes.get(sys_id, 0.0))
        await controller._set_state(sys_id, VehicleState.airborne)
    else:
        log.error("[launch] V%d ALTITUDE TIMEOUT (alt=%.1f after %.1fs)",
                  sys_id, controller._altitudes.get(sys_id, 0.0), controller._altitude_timeout)
        await controller._fail_session(sys_id, "Altitude timeout")
        return

    # Post-launch spacing: hold the lock briefly after the launch trigger so the next
    # vehicle's ESP32 trigger is spaced out. Within a container this is the
    # small inter-vehicle stagger; at a container boundary it's the larger
    # container gap. Abort-aware so it never delays an abort, and skipped
    # when no vehicle is still waiting (don't delay launch_complete after
    # the last launch trigger).
    delay = controller._post_vehicle_delay(sys_id)
    if delay > 0 and controller._has_pending_vehicle():
        await controller._sleep_or_abort(delay)

    await controller._check_all_done()

async def _sleep_or_abort(controller: LaunchController, seconds: float) -> None:
    """Sleep up to *seconds*, returning early if abort is signalled."""
    try:
        await asyncio.wait_for(controller._abort_event.wait(), timeout=seconds)
    except asyncio.TimeoutError:
        pass

async def _poll(controller: LaunchController, condition: Callable[[], bool], timeout: float) -> bool | None:
    """Poll condition at 100ms intervals with abort support.

    Returns True if condition met, False on timeout, None on abort.
    """
    start = time.monotonic()
    while not condition():
        if controller._abort_event.is_set():
            return None
        if time.monotonic() - start > timeout:
            return False
        await asyncio.sleep(0.1)
    return True
