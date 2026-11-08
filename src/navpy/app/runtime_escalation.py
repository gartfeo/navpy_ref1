"""A latched component failure no longer ends NavPy by itself (docs/design/navpy_recovery.md, step 5).

The first failure the health check raises closes the command fence and asks NAV to escalate
(core/flow/nav_recovery.py: hand the UAV to the mission's AUTO, RTL if that is not confirmed). NavPy then exits,
for a fresh start, only when NAV released the UAV and a supervisor restarts it (``--supervised``: the GCS's
auto-managed sim companions). Otherwise it stays up, fenced and reporting, until the operator stops it.
A ``BaseException`` that is not an ``Exception`` (KeyboardInterrupt, SystemExit) still ends the run at once.
"""

from __future__ import annotations

from argparse import ArgumentParser, Namespace
from collections.abc import Callable

from navpy.core.flow.nav_recovery_ports import NavEscalation
from navpy.core.kernel.logging.logger_api import ILogger

SUPERVISED_FLAG = "--supervised"


def add_args(parser: ArgumentParser) -> None:
    parser.add_argument(
        SUPERVISED_FLAG,
        action="store_true",
        help="A supervisor restarts NavPy when it exits (GCS auto-managed sim): after a latched component "
             "failure NavPy hands the UAV back and exits for a fresh start instead of staying up fenced.",
    )


def supervised(args: Namespace) -> bool:
    return bool(getattr(args, "supervised", False))


class RuntimeEscalation:
    """Owned by the main (UI pump) thread."""

    def __init__(
        self,
        *,
        close_fence: Callable[[], object],
        escalation: NavEscalation,
        supervised: bool,
        logger: ILogger,
    ) -> None:
        self._close_fence = close_fence
        self._escalation = escalation
        self._supervised = supervised
        self._logger = logger
        self._failure: Exception | None = None
        self._waiting_reported = False

    def check(self, health_check: Callable[[], None]) -> None:
        """Run the health check; raise only when NavPy may exit."""
        if self._failure is None:
            try:
                health_check()
            except Exception as failure:  # noqa: BLE001 - escalated, then raised once NavPy may exit
                self._escalate(failure)
        if self._failure is None or not self._escalation.released:
            return
        if self._supervised:
            raise self._failure
        if not self._waiting_reported:
            self._waiting_reported = True
            self._logger.error("NavPy stays up fenced until stopped: no supervisor restarts it")

    def _escalate(self, failure: Exception) -> None:
        self._failure = failure
        # The NAV loop may be the part that failed: close the fence here too, it takes only the fence lock.
        self._close_fence()
        self._escalation.request()
        self._logger.error(f"NavPy component failed, commands fenced: {failure!r}", failure)


__all__ = ["RuntimeEscalation", "SUPERVISED_FLAG", "add_args", "supervised"]
