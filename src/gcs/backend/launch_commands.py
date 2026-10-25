"""Launch commands confirmed and resent until they hold (``radio_budget.md`` round 7, todo 117)."""
from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable
from enum import Enum, auto
from typing import TYPE_CHECKING, Callable

from navpy.core.kernel import runtime_time as rt

if TYPE_CHECKING:
    from gcs.backend.esp32_trigger import TriggerResult
    from gcs.backend.launch_controller import LaunchController

log = logging.getLogger("gcs.backend.launch_controller")

#: How long a launch command (AUTO, arm) waits before it is sent again: the
#: heartbeat period (1 Hz); sooner, the last command's effect cannot be seen yet.
LAUNCH_COMMAND_RESEND_S = 1.0


class CommandResult(Enum):
    DONE = auto()
    TIMEOUT = auto()
    ABORTED = auto()
    BLOCKED = auto()  # an attempt failed the session (EKF re-check)


async def set_auto(controller: LaunchController, sys_id: int) -> bool:
    """Send AUTO until a heartbeat newer than the first send reads AUTO; False: the run ended."""
    since: list[int] = []

    async def attempt() -> bool:
        count = controller._auto_func(sys_id)
        if not since and count is not None:  # read right before a send to a connected vehicle
            since.append(count)
        return True

    def confirmed() -> bool:
        mode, count = controller._modes.get(sys_id, (None, 0))
        return bool(since) and mode == "AUTO" and count > since[0]

    result = await command_until(controller, sys_id, attempt, confirmed, controller._arm_timeout)
    if result is CommandResult.TIMEOUT:
        log.error("[launch] V%d AUTO MODE TIMEOUT after %.1fs", sys_id, controller._arm_timeout)
        await controller._fail_session(sys_id, "AUTO mode timeout")
    elif result is CommandResult.ABORTED:
        await controller._abort_vehicle(sys_id)
    return result is CommandResult.DONE


async def arm(controller: LaunchController, sys_id: int) -> bool:
    """Send arm, the EKF re-checked right before each send, until armed; False: the run ended."""
    async def attempt() -> bool:
        if await ekf_blocks(controller, sys_id, "arm"):
            return False
        controller._arm_func(sys_id)
        return True

    result = await command_until(
        controller, sys_id, attempt, lambda: controller._armed.get(sys_id, False), controller._arm_timeout,
    )
    if result is CommandResult.TIMEOUT:
        log.error("[launch] V%d ARM TIMEOUT after %.1fs", sys_id, controller._arm_timeout)
        await controller._fail_session(sys_id, "Arm timeout")
    elif result is CommandResult.ABORTED:
        await controller._abort_vehicle(sys_id)
    return result is CommandResult.DONE


async def command_until(
    controller: LaunchController,
    sys_id: int,
    attempt: Callable[[], Awaitable[bool]],
    done: Callable[[], bool],
    timeout: float,
) -> CommandResult:
    """Send a launch command until it takes effect (``radio_budget.md`` round 7, todo 117).

    ``attempt`` sends it (False: it failed the session); it runs at once and
    again every ``LAUNCH_COMMAND_RESEND_S`` while the launch still owns the
    vehicle's commands (an operator command revokes that). ``done`` is polled
    every 0.1 s, like the arm wait was.
    """
    start = rt.monotonic()
    next_attempt = start
    while not done():
        if controller._abort_event.is_set():
            return CommandResult.ABORTED
        now = rt.monotonic()
        if now - start > timeout:
            return CommandResult.TIMEOUT
        if now >= next_attempt and sys_id not in controller._revoked:
            if not await attempt():
                return CommandResult.BLOCKED
            next_attempt = now + LAUNCH_COMMAND_RESEND_S
            continue  # the attempt may have taken effect at once
        await asyncio.sleep(0.1)
    return CommandResult.DONE


def release_refusal(controller: LaunchController, sys_id: int) -> str | None:
    """Why the release must not go now: an operator command, or the vehicle not AUTO and armed."""
    if sys_id in controller._revoked:
        return "Operator command during launch"
    mode, _count = controller._modes.get(sys_id, (None, 0))
    armed = controller._armed.get(sys_id, False)
    if mode == "AUTO" and armed:
        return None
    return f"Not AUTO and armed at release (mode {mode}, armed {armed})"


async def ekf_blocks(controller: LaunchController, sys_id: int, step: str) -> bool:
    """Fail the session when the EKF re-check blocks *sys_id*'s next *step*.

    Callers run it after their last await and immediately before dispatching
    the arm or release, so no wait (a settle, an arm poll, a state broadcast)
    sits between the check and the command.
    """
    issues = controller._ekf_recheck(sys_id)
    if not issues:
        return False
    log.error("[launch] V%d %s BLOCKED by EKF re-check: %s", sys_id, step, issues)
    await controller._fail_session(sys_id, "; ".join(issues))
    return True


async def trigger(controller: LaunchController, sys_id: int, channel: int) -> TriggerResult | None:
    """Release on the executor; :func:`release_refusal` is checked there, right before the call.

    So a release queued behind busy workers still stops for a change while it
    waited. None: the session failed.
    """
    refused: list[str] = []

    def release() -> TriggerResult | None:
        reason = release_refusal(controller, sys_id)
        if reason is not None:
            refused.append(reason)
            return None
        return controller._esp32_client.trigger_channel(channel)

    result = await asyncio.get_running_loop().run_in_executor(None, release)
    if refused:
        log.error("[launch] V%d release BLOCKED: %s", sys_id, refused[0])
        await controller._fail_session(sys_id, refused[0])
    return result
