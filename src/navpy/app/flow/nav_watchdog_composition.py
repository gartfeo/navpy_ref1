"""NAV recovery and the in-process watchdog of one NavPy (core/flow/nav_recovery.py, nav_watchdog.py;
docs/design/navpy_recovery.md steps 4 and 6)."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.flow.nav_recovery import NavRecovery
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.flow.nav_recovery_ports import NavRecoveryLink, NavRecoveryPorts, RecoveryHold
from navpy.core.flow.nav_recovery_step import SharedRecoveryStep
from navpy.core.flow.nav_watchdog import NavWatchdog, NavWatchdogPorts
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.vehicle_contract import IVehicle


def nav_recovery(
    vehicle: IVehicle,
    hold: RecoveryHold,
    logger: ILogger,
    link: Optional[NavRecoveryLink],
) -> Optional[SharedRecoveryStep]:
    """None without a link (no MAVLink bus: tests, other vehicles): no fence to close and no recovery."""
    if link is None:
        return None
    return SharedRecoveryStep(NavRecovery(
        NavRecoveryPorts(
            close_fence=link.close_fence,
            reopen_fence=link.reopen_fence,
            observe_mode=link.observe_mode,
            request_mode=vehicle.set_mode if link.request_mode is None else link.request_mode,
            logger=logger,
            escalation=link.escalation,
            watchdog=link.watchdog,
        ),
        hold,
    ))


def recovery_period(
    recovery: Optional[SharedRecoveryStep], sample: Callable[[], TimeSample],
) -> Callable[[bool], None]:
    """The NAV loop's per-period hook: recovery's step."""
    if recovery is None:
        return lambda _cycle_ok: None
    return lambda cycle_ok: recovery.step(cycle_ok, sample()) and None


def nav_watchdog(
    link: Optional[NavRecoveryLink],
    nav_progress: ProgressBeacon,
    recovery: Optional[SharedRecoveryStep],
    sample: Callable[[], TimeSample],
    logger: ILogger,
) -> Optional[NavWatchdog]:
    """None without a link: there is no fence to close and no recovery."""
    if link is None:
        return None

    def report(message: str) -> None:
        logger.warning(message, key="nav", status=message, dest=LogStatusDest.DRONE)

    return NavWatchdog(NavWatchdogPorts(
        nav_progress=nav_progress,
        command_progress=link.command_progress,
        signal=link.watchdog,
        close_fence=link.close_fence,
        step_recovery=None if recovery is None else recovery.step,
        sample=sample,
        report=report,
    ))


__all__ = ["nav_recovery", "nav_watchdog", "recovery_period"]
