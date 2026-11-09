"""NAV recovery and the in-process watchdog of one NavPy (core/flow/nav_recovery.py, nav_watchdog.py;
docs/design/navpy_recovery.md steps 4 and 6)."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.flow.nav_mode_sender import ModeSender
from navpy.core.flow.nav_recovery import NavRecovery
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.flow.nav_recovery_ports import NavRecoveryLink, NavRecoveryPorts, RecoveryHold
from navpy.core.flow.nav_recovery_step import SharedRecoveryStep
from navpy.core.flow.nav_watchdog import NavWatchdog, NavWatchdogPorts
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
    report = _reporter(logger)
    sender = None if link.request_mode is None else ModeSender(link.request_mode, link.observe_mode, report)
    return SharedRecoveryStep(NavRecovery(
        NavRecoveryPorts(
            close_fence=link.close_fence,
            reopen_fence=link.reopen_fence,
            observe_mode=link.observe_mode,
            request_mode=vehicle.set_mode,
            logger=logger,
            escalation=link.escalation,
            watchdog=link.watchdog,
            mode_sender=sender,
        ),
        hold,
    ), sender)


def recovery_period(
    recovery: Optional[SharedRecoveryStep], sample: Callable[[], TimeSample],
) -> Callable[[bool], None]:
    """The NAV loop's per-period hook: recovery's step."""
    if recovery is None:
        return lambda _cycle_ok: None
    return lambda cycle_ok: recovery.step(cycle_ok, sample) and None


class RecoveryThreads:
    """Recovery's threads, started with the NAV application: the mode sender, then the watchdog; stopped the other
    way round, before the loop's shutdown join, which must not read as a hang (NavApplication)."""

    def __init__(self, sender: Optional[ModeSender], watchdog: NavWatchdog) -> None:
        self._sender = sender
        self._watchdog = watchdog

    def start(self) -> None:
        if self._sender is not None:
            self._sender.start()
        self._watchdog.start()

    def stop(self) -> None:
        self._watchdog.stop()
        if self._sender is not None:
            self._sender.stop()


def _close_when_held(link: NavRecoveryLink) -> Callable[[bool], object]:
    """A link without the fence's veto (tests): closing on every stuck check is all there is."""
    return lambda held: link.close_fence() if held else None


def _reporter(logger: ILogger) -> Callable[[str], None]:
    def report(message: str) -> None:
        logger.recovery(message, "watchdog")

    return report


def nav_watchdog(
    link: Optional[NavRecoveryLink],
    nav_progress: ProgressBeacon,
    recovery: Optional[SharedRecoveryStep],
    sample: Callable[[], TimeSample],
    logger: ILogger,
) -> Optional[RecoveryThreads]:
    """None without a link: there is no fence to close and no recovery."""
    if link is None:
        return None
    watchdog = NavWatchdog(NavWatchdogPorts(
        nav_progress=nav_progress,
        command_progress=link.command_progress,
        signal=link.watchdog,
        hold_fence=_close_when_held(link) if link.hold_fence is None else link.hold_fence,
        step_recovery=None if recovery is None else recovery.step,
        sample=sample,
        report=_reporter(logger),
    ))
    return RecoveryThreads(None if recovery is None else recovery.sender, watchdog)


__all__ = ["RecoveryThreads", "nav_recovery", "nav_watchdog", "recovery_period"]
