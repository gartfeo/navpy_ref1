"""NAV recovery's view of one vehicle (core/flow/nav_recovery.py): its bus's command fence and its latest heartbeat,
and the main thread's escalation of a component failure through it (runtime_escalation.py)."""

from __future__ import annotations

from argparse import Namespace
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext

from navpy.app.runtime_escalation import RuntimeEscalation, supervised
from navpy.core.flow.nav_mode_request import MODE_SEND_LOCK_TIMEOUT_S
from navpy.core.flow.nav_ownership import ModeObservation
from navpy.core.ports.flight_mode import FlightMode
from navpy.core.kernel.logging.logger_api import ILogger
from navpy.core.flow.nav_recovery_ports import NavRecoveryLink
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence


def nav_recovery_link(fence: MavCommandFence, vehicle: IVehicle) -> NavRecoveryLink:
    """The heartbeat count is read first, so a newer count means a mode at least that new."""

    def observe_mode() -> ModeObservation:
        count = vehicle.heartbeat_count
        return ModeObservation(
            heartbeat_count=count,
            mode=vehicle.get_mode,
            armed=vehicle.is_armed,
            fresh=vehicle.link_ok,
        )

    def request_mode(mode: FlightMode) -> bool:
        # Bounded wait for the send lock: never a late write, never a stalled recovery step (step 6).
        return vehicle.set_mode(mode, lock_timeout_s=MODE_SEND_LOCK_TIMEOUT_S)

    return NavRecoveryLink(
        close_fence=fence.close, reopen_fence=fence.reopen, observe_mode=observe_mode,
        latch_fence=fence.latch_closed, formed_now=fence.formed_now, request_mode=request_mode,
    )


def command_formation(link: NavRecoveryLink | None) -> Callable[[], AbstractContextManager[None]]:
    """The command worker's formation context: without a link (no MAVLink bus) there is no fence."""
    if link is None:
        return nullcontext

    return link.formed_now


def runtime_escalation(args: Namespace, link: NavRecoveryLink | None, logger: ILogger) -> RuntimeEscalation | None:
    """None without a link (no MAVLink bus): a component failure then ends NavPy as before."""
    if link is None:
        return None
    return RuntimeEscalation(
        close_fence=link.latch_fence, escalation=link.escalation, supervised=supervised(args), logger=logger,
    )


def command_pass(link: NavRecoveryLink | None) -> Callable[[], AbstractContextManager[None]]:
    """One whole command-worker pass, watched for a hang (step 6): without a link there is no watchdog."""
    return nullcontext if link is None else link.command_progress.busy


def command_contexts(link: NavRecoveryLink | None) -> dict[str, Callable[[], AbstractContextManager[None]]]:
    """Navigation's command-worker contexts on this link: the fence generation and the watched pass."""
    return {"command_formation": command_formation(link), "command_pass": command_pass(link)}


__all__ = [
    "NavRecoveryLink", "command_contexts", "command_formation", "command_pass", "nav_recovery_link",
    "runtime_escalation",
]
