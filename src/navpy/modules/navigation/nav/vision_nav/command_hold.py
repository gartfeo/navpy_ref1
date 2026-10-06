"""Zero-order hold for primitive terminal actuator commands."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol

from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    TerminalAttitudeActuator,
)
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    TerminalCommandFreshness,
    TerminalCommandTiming,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    TerminalCommandFailureSink,
)
from navpy.modules.navigation.nav.vision_nav.source_time_ports import (
    TerminalCommandSourceTimeObserver,
)


class TerminalCommandHoldStore(Protocol):
    def remember(
        self,
        calc_data: CalcData,
        *,
        timing: TerminalCommandTiming,
    ) -> None: ...

    def clear(self) -> None: ...


class TerminalCommandHoldIssuer(Protocol):
    @property
    def available(self) -> bool: ...

    def issue(self) -> "TerminalHeldIssue | None": ...

    def record(self, issue: "TerminalHeldIssue") -> None: ...


@dataclass(frozen=True)
class TerminalHeldCommand:
    cmd_roll_deg: float
    cmd_pitch_deg: float
    throttle: float | None
    timing: TerminalCommandTiming

    def calc_data(self) -> CalcData:
        return CalcData(
            0.0,
            0.0,
            self.cmd_roll_deg,
            self.cmd_pitch_deg,
            self.throttle,
        )


@dataclass(frozen=True)
class TerminalHeldIssue:
    command: TerminalHeldCommand
    wall_start_s: float
    execution_ms: float


class TerminalCommandHold:
    """Reissue the last primitive output without re-running visual state."""

    def __init__(
        self,
        actuator: TerminalAttitudeActuator,
        source_time: TerminalCommandSourceTimeObserver,
        failures: TerminalCommandFailureSink,
        freshness: TerminalCommandFreshness,
        monotonic_s: Callable[[], float] = time.perf_counter,
    ) -> None:
        self._actuator = actuator
        self._source_time = source_time
        self._failures = failures
        self._freshness = freshness
        self._monotonic_s = monotonic_s
        self._lock = threading.RLock()
        self._command: TerminalHeldCommand | None = None

    @property
    def available(self) -> bool:
        with self._lock:
            return self._command is not None

    def remember(
        self,
        calc_data: CalcData,
        *,
        timing: TerminalCommandTiming,
    ) -> None:
        if calc_data.cmd_roll is None or calc_data.cmd_pitch is None:
            self.clear()
            return
        with self._lock:
            self._command = TerminalHeldCommand(
                float(calc_data.cmd_roll),
                float(calc_data.cmd_pitch),
                None if calc_data.cmd_thr is None else float(calc_data.cmd_thr),
                timing,
            )

    def clear(self) -> None:
        with self._lock:
            self._command = None

    def issue(self) -> TerminalHeldIssue | None:
        with self._lock:
            command = self._command
            if command is None:
                return None
            issued = False
            try:
                if not self._freshness.is_fresh(command.timing):
                    return None
                started_s = self._monotonic_s()
                self._actuator.issue(
                    command.cmd_roll_deg,
                    command.cmd_pitch_deg,
                    command.throttle,
                )
                issued = True
                return TerminalHeldIssue(
                    command,
                    started_s,
                    (self._monotonic_s() - started_s) * 1000.0,
                )
            finally:
                if not issued:
                    self._command = None
                    self._failures.mark_failed()

    def record(self, issue: TerminalHeldIssue) -> None:
        self._source_time.record_command(
            wall_start_s=issue.wall_start_s,
            source_timestamp_s=issue.command.timing.source_timestamp_s,
            source_now_s=issue.command.timing.source_now_s,
            execution_ms=issue.execution_ms,
            outcome="held",
        )


__all__ = [
    "TerminalCommandHold",
    "TerminalCommandHoldIssuer",
    "TerminalCommandHoldStore",
    "TerminalHeldCommand",
    "TerminalHeldIssue",
]
