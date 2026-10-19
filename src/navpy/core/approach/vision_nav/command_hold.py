"""Zero-order hold for primitive final-approach actuator commands."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Protocol

from navpy.core.approach.calc_data import CalcData
from navpy.core.approach.vision_nav.command_transaction import (
    FinalApproachAttitudeActuator,
)
from navpy.core.approach.vision_nav.command_freshness import (
    FinalApproachCommandFreshness,
    FinalApproachCommandTiming,
)
from navpy.core.approach.vision_nav.command_liveness import (
    FinalApproachCommandFailureSink,
)
from navpy.core.approach.vision_nav.source_time_ports import (
    FinalApproachCommandSourceTimeObserver,
)
from navpy.core.kernel import runtime_time as rt


# Aircraft-time period between re-sends of an unchanged (held) command.
# ArduPlane drops a GUIDED roll/pitch/throttle override 3.0 s after the last
# attitude target (hardcoded, mode_guided.cpp:35-87, Plane-4.6.3). Margin:
# 0.5 s refresh + <=0.5 s scheduling jitter + <=0.1 s transport (p95) = 1.1 s,
# well under 3.0 s. Pacing held re-sends by aircraft time, not by worker ticks,
# keeps the uplink within SITL's one-datagram-per-10-ms-of-sim input when SITL
# runs slower than its nominal speedup (design a2, final_approach_delivery_fix).
ATTITUDE_REFRESH_S = 0.5


class FinalApproachCommandHoldStore(Protocol):
    def remember(
        self,
        calc_data: CalcData,
        *,
        timing: FinalApproachCommandTiming,
    ) -> None: ...

    def clear(self) -> None: ...


class FinalApproachCommandHoldIssuer(Protocol):
    @property
    def available(self) -> bool: ...

    def issue(self) -> "FinalApproachHeldIssue | None": ...

    def record(self, issue: "FinalApproachHeldIssue") -> None: ...


@dataclass(frozen=True)
class FinalApproachHeldCommand:
    cmd_roll_deg: float
    cmd_pitch_deg: float
    throttle: float | None
    timing: FinalApproachCommandTiming

    def calc_data(self) -> CalcData:
        return CalcData(
            0.0,
            0.0,
            self.cmd_roll_deg,
            self.cmd_pitch_deg,
            self.throttle,
        )


@dataclass(frozen=True)
class FinalApproachHeldIssue:
    command: FinalApproachHeldCommand
    wall_start_s: float
    execution_ms: float


class FinalApproachCommandHold:
    """Refresh the newest primitive output once per aircraft-time period.

    Freshness is checked on every held tick; a send happens only when
    ``ATTITUDE_REFRESH_S`` of the command's source (aircraft) clock has passed
    since the last send of this command. Missed refreshes are never replayed.
    """

    def __init__(
        self,
        actuator: FinalApproachAttitudeActuator,
        source_time: FinalApproachCommandSourceTimeObserver,
        failures: FinalApproachCommandFailureSink,
        freshness: FinalApproachCommandFreshness,
        monotonic_s: Callable[[], float] = rt.perf_counter,
    ) -> None:
        self._actuator = actuator
        self._source_time = source_time
        self._failures = failures
        self._freshness = freshness
        self._monotonic_s = monotonic_s
        self._lock = rt.RLock()
        self._command: FinalApproachHeldCommand | None = None
        self._last_sent_source_s: float | None = None

    @property
    def available(self) -> bool:
        with self._lock:
            return self._command is not None

    def remember(
        self,
        calc_data: CalcData,
        *,
        timing: FinalApproachCommandTiming,
    ) -> None:
        if calc_data.cmd_roll is None or calc_data.cmd_pitch is None:
            self.clear()
            return
        with self._lock:
            self._command = FinalApproachHeldCommand(
                float(calc_data.cmd_roll),
                float(calc_data.cmd_pitch),
                None if calc_data.cmd_thr is None else float(calc_data.cmd_thr),
                timing,
            )
            # The new command was just sent; its refresh deadline starts now.
            self._last_sent_source_s = _source_now_s(timing)

    def clear(self) -> None:
        with self._lock:
            self._command = None
            self._last_sent_source_s = None

    def issue(self) -> FinalApproachHeldIssue | None:
        with self._lock:
            command = self._command
            if command is None:
                return None
            issued = False
            refresh_not_due = False
            try:
                if not self._freshness.is_fresh(command.timing):
                    return None
                source_now_s = _source_now_s(command.timing)
                if not self._refresh_due(source_now_s):
                    refresh_not_due = True
                    return None
                started_s = self._monotonic_s()
                self._actuator.issue(
                    command.cmd_roll_deg,
                    command.cmd_pitch_deg,
                    command.throttle,
                )
                self._last_sent_source_s = source_now_s
                issued = True
                return FinalApproachHeldIssue(
                    command,
                    started_s,
                    (self._monotonic_s() - started_s) * 1000.0,
                )
            finally:
                if not issued and not refresh_not_due:
                    self._command = None
                    self._last_sent_source_s = None
                    self._failures.mark_failed()

    def _refresh_due(self, source_now_s: float | None) -> bool:
        """Due once a refresh period passed; unknown or regressed time sends."""
        last_sent_s = self._last_sent_source_s
        if source_now_s is None or last_sent_s is None:
            return True
        elapsed_s = source_now_s - last_sent_s
        return not 0.0 <= elapsed_s < ATTITUDE_REFRESH_S

    def record(self, issue: FinalApproachHeldIssue) -> None:
        self._source_time.record_command(
            wall_start_s=issue.wall_start_s,
            source_timestamp_s=issue.command.timing.source_timestamp_s,
            source_now_s=issue.command.timing.source_now_s,
            execution_ms=issue.execution_ms,
            outcome="held",
        )


def _source_now_s(timing: FinalApproachCommandTiming) -> float | None:
    """Read the command's source (aircraft) clock; ``None`` when unusable."""
    provider = timing.source_now_s
    if provider is None:
        return None
    try:
        raw_now_s = provider()
    except Exception:  # noqa: BLE001 - unusable clock forces a refresh send
        return None
    if isinstance(raw_now_s, bool):
        return None
    try:
        now_s = float(raw_now_s)
    except (TypeError, ValueError):
        return None
    return now_s if math.isfinite(now_s) else None


__all__ = [
    "ATTITUDE_REFRESH_S",
    "FinalApproachCommandHold",
    "FinalApproachCommandHoldIssuer",
    "FinalApproachCommandHoldStore",
    "FinalApproachHeldCommand",
    "FinalApproachHeldIssue",
]
