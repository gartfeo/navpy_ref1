"""A one-shot sortie's return after a failed dock (docs/design/navpy_recovery.md, step 10, item 3).

Owner ruling 2026-11-09: a failed one-shot dock (DOCK_FAILED) is never disarmed, in simulation or on real hardware;
NavPy asks for RTL instead, so a failed dock never leaves the aircraft in GUIDED. The request goes through
recovery's ownership like every automatic mode request (nav_ownership.py): sent only while NavPy owns the mode and
the heartbeat is fresh, and repeated every MODE_RETRY_S until a heartbeat shows RTL. A mode NavPy does not own (the
operator's, a failsafe's) stands and ends the request; a stale link, or a GUIDED request of NavPy's still unsettled,
waits for a fresh heartbeat. It runs in recovery's step while recovery is NORMAL; in a hold recovery's stages own the
mode, and an RTL already shown is never turned into the hold's LOITER (nav_recovery.py).
"""

from __future__ import annotations

from navpy.core.flow.nav_mode_request import ModeRequest
from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership, Ownership
from navpy.core.flow.nav_recovery_ports import RecoveryHold
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.flight_mode import FlightMode


def return_after_failed_dock(
    hold: RecoveryHold, mode: ModeRequest, ownership: NavOwnership, observation: ModeObservation, now: float,
    logger: ILogger,
) -> None:
    if not hold.sortie_rtl:
        return
    if observation.fresh and observation.mode is FlightMode.RTL:
        hold.end_sortie_rtl()
        mode.clear()
        _report(logger, "One-shot dock failed: RTL shown")
    elif ownership.may_request_mode():
        if mode.pending is FlightMode.RTL:
            mode.retry(observation, now)
        else:
            mode.send(FlightMode.RTL, observation, now)
            _report(logger, "One-shot dock failed: RTL requested, no disarm")
    elif ownership.state is Ownership.NOT_OWNED and observation.fresh and not ownership.guided_pending:
        hold.end_sortie_rtl()
        shown = observation.mode.name if observation.mode is not None else "the mode shown"
        _report(logger, f"One-shot dock failed: {shown} is not NavPy's and stands, no RTL")


def _report(logger: ILogger, message: str) -> None:
    try:
        logger.info(message, key="nav")
    except Exception:  # noqa: BLE001 - reporting must not stop recovery's step
        pass


__all__ = ["return_after_failed_dock"]
