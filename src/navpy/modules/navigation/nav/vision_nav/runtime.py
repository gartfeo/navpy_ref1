"""Small source-driven runtime for pure-vision final approach."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Iterable

from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_command_slot import (
    NavigationCommandLease,
    NavigationCommandSlot,
)
from navpy.modules.navigation.nav.vision_nav.command_executor import (
    TerminalCommandExecutor,
    TerminalCommandWork,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    TerminalCommandHoldIssuer,
    TerminalHeldIssue,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    TerminalCommandLiveness,
)
from navpy.modules.navigation.nav.vision_nav.command_reset import (
    TerminalCommandReset,
)
from navpy.modules.navigation.nav.vision_nav.ingress import TerminalIngress
from navpy.modules.navigation.nav.vision_nav.queued_frame import TerminalQueuedFrame
from navpy.modules.navigation.nav.vision_nav.runtime_state import TerminalRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassDetector
from navpy.modules.vision.models.detect_data import DetectedObject


class TerminalCommandWorkRuntime:
    """Own command admission, leases, execution, and zero-order hold work."""

    def __init__(
        self,
        slot: NavigationCommandSlot,
        ingress: TerminalIngress,
        executor: TerminalCommandExecutor,
        hold: TerminalCommandHoldIssuer,
    ) -> None:
        self._slot = slot
        self._ingress = ingress
        self._executor = executor
        self._hold = hold

    def nav(self, detect_data: DetectedObject) -> bool:
        return self._ingress.nav(detect_data)

    def has_command_pending_or_in_flight(self) -> bool:
        return self._slot.has_pending_or_in_flight() or self._hold.available

    def take_work(self) -> TerminalCommandWork | TerminalHeldCommandWork | None:
        lease = self._slot.take_or_else(
            lambda: _HELD_COMMAND if self._hold.available else None
        )
        if lease is None:
            return None
        if lease.payload is _HELD_COMMAND:
            return TerminalHeldCommandWork(lease)
        if not isinstance(lease.payload, TerminalQueuedFrame):
            self._slot.finish(lease)
            return None
        payload = lease.payload
        return TerminalCommandWork(payload.frame, payload.diagnostic_token, lease)

    def execute_work(
        self,
        work: TerminalCommandWork | TerminalHeldCommandWork,
    ) -> CalcData | None:
        if isinstance(work, TerminalHeldCommandWork):
            issue = self._slot.execute_if_current(
                work.lease,
                self._hold.issue,
            )
            if not isinstance(issue, TerminalHeldIssue):
                return None
            self._hold.record(issue)
            return issue.command.calc_data()
        return self._executor.execute(work)

    def finish_work(
        self,
        work: TerminalCommandWork | TerminalHeldCommandWork,
    ) -> None:
        self._slot.finish(work.lease)

    def postprocess_job(
        self,
        work: TerminalCommandWork | TerminalHeldCommandWork,
    ) -> Callable[[], None] | None:
        if isinstance(work, TerminalCommandWork):
            return self._executor.postprocess_job(work)
        return None


class TerminalSessionRuntime:
    """Own terminal lifecycle, source continuity, liveness, and status state."""

    def __init__(
        self,
        lock: threading.RLock,
        epochs: SourceEpochLedger,
        command_reset: TerminalCommandReset,
        visual_pass: VisualPassDetector,
        status: TerminalRuntimeStatus,
        liveness: TerminalCommandLiveness,
    ) -> None:
        self._lock = lock
        self._epochs = epochs
        self._command_reset = command_reset
        self._visual_pass = visual_pass
        self._status = status
        self._liveness = liveness

    def invalidate_commands(self) -> None:
        self._command_reset.invalidate_commands()

    def reset_phase(self) -> None:
        self._command_reset.reset_phase()

    def consume_command_liveness_failure(self) -> bool:
        return self._liveness.consume_failure()

    def clear_source_discontinuity_state(
        self,
        source_names: str | Iterable[str],
    ) -> None:
        names = (source_names,) if isinstance(source_names, str) else source_names
        with self._lock:
            changed = self._epochs.note_discontinuities(names)
            if changed:
                self._command_reset.invalidate_commands()

    def target_passed_override(self) -> bool:
        with self._lock:
            return self._visual_pass.passed

    def last_measured_lateral_bearing_deg(self) -> float | None:
        return self._status.last_measured_lateral_bearing_deg()


class VisionNavRuntime:
    """Expose the two focused terminal runtime capabilities."""

    def __init__(
        self,
        commands: TerminalCommandWorkRuntime,
        session: TerminalSessionRuntime,
    ) -> None:
        self._commands = commands
        self._session = session

    def nav(self, detect_data: DetectedObject) -> bool:
        return self._commands.nav(detect_data)

    def has_command_pending_or_in_flight(self) -> bool:
        return self._commands.has_command_pending_or_in_flight()

    def take_work(self) -> TerminalCommandWork | TerminalHeldCommandWork | None:
        return self._commands.take_work()

    def execute_work(
        self,
        work: TerminalCommandWork | TerminalHeldCommandWork,
    ) -> CalcData | None:
        return self._commands.execute_work(work)

    def finish_work(
        self,
        work: TerminalCommandWork | TerminalHeldCommandWork,
    ) -> None:
        self._commands.finish_work(work)

    def postprocess_job(
        self,
        work: TerminalCommandWork | TerminalHeldCommandWork,
    ) -> Callable[[], None] | None:
        return self._commands.postprocess_job(work)

    def invalidate_commands(self) -> None:
        self._session.invalidate_commands()

    def reset_phase(self) -> None:
        self._session.reset_phase()

    def consume_command_liveness_failure(self) -> bool:
        return self._session.consume_command_liveness_failure()

    def clear_source_discontinuity_state(
        self,
        source_names: str | Iterable[str],
    ) -> None:
        self._session.clear_source_discontinuity_state(source_names)

    def target_passed_override(self) -> bool:
        return self._session.target_passed_override()

    def last_measured_lateral_bearing_deg(self) -> float | None:
        return self._session.last_measured_lateral_bearing_deg()


@dataclass(frozen=True)
class TerminalHeldCommandWork:
    lease: NavigationCommandLease


_HELD_COMMAND = object()


__all__ = [
    "TerminalCommandWorkRuntime",
    "TerminalHeldCommandWork",
    "TerminalSessionRuntime",
    "VisionNavRuntime",
]
