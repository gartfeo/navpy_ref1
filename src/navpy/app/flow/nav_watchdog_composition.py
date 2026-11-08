"""The in-process watchdog of one NavPy (core/flow/nav_watchdog.py, docs/design/navpy_recovery.md step 6)."""

from __future__ import annotations

from typing import Optional

from navpy.core.flow.nav_recovery_clock import AircraftTimeClock, attitude_boot_s
from navpy.core.flow.nav_recovery_ports import NavRecoveryLink
from navpy.core.flow.nav_watchdog import NavWatchdog, NavWatchdogPorts
from navpy.core.kernel.bounded_sender import BoundedSender
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.vehicle_contract import IVehicle


def nav_watchdog(
    vehicle: IVehicle,
    link: Optional[NavRecoveryLink],
    nav_progress: ProgressBeacon,
    logger: ILogger,
) -> Optional[NavWatchdog]:
    """None without a link (no MAVLink bus): there is no fence to close and no recovery."""
    if link is None:
        return None

    def report(message: str) -> None:
        logger.warning(message, key="nav", status=message, dest=LogStatusDest.DRONE)

    return NavWatchdog(NavWatchdogPorts(
        nav_progress=nav_progress,
        command_progress=link.command_progress,
        signal=link.watchdog,
        close_fence=link.close_fence,
        observe_mode=link.observe_mode,
        request_mode=BoundedSender(vehicle.set_mode, report),
        report=report,
        clock=AircraftTimeClock(lambda: attitude_boot_s(vehicle)),
    ))


__all__ = ["nav_watchdog"]
