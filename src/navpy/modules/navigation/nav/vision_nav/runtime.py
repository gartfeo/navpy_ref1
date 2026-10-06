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
    FinalApproachCommandExecutor,
    FinalApproachCommandWork,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    FinalApproachCommandHoldIssuer,
    FinalApproachHeldIssue,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    FinalApproachCommandLiveness,
)
from navpy.modules.navigation.nav.vision_nav.command_reset import (
    FinalApproachCommandReset,
)
from navpy.modules.navigation.nav.vision_nav.ingress import FinalApproachIngress
from navpy.modules.navigation.nav.vision_nav.queued_frame import FinalApproachQueuedFrame
from navpy.modules.navigation.nav.vision_nav.runtime_state import FinalApproachRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassDetector
from navpy.modules.vision.models.detect_data import DetectedObject


class FinalApproachCommandWorkRuntime:
    """Own command admission, leases, execution, and zero-order hold work."""

    def __init__(
        self,
        slot: NavigationCommandSlot,
        ingress: FinalApproachIngress,
        executor: FinalApproachCommandExecutor,
        hold: FinalApproachCommandHoldIssuer,
    ) -> None:
        self._slot = slot
        self._ingress = ingress
        self._executor = executor
        self._hold = hold

    def nav(self, detect_data: DetectedObject) -> bool:
        return self._ingress.nav(detect_data)

    def has_command_pending_or_in_flight(self) -> bool:
        return self._slot.has_pending_or_in_flight() or self._hold.available

    def take_work(self) -> FinalApproachCommandWork | FinalApproachHeldCommandWork | None:
        lease = self._slot.take_or_else(
            lambda: _HELD_COMMAND if self._hold.available else None
        )
        if lease is None:
            return None
        if lease.payload is _HELD_COMMAND:
            return FinalApproachHeldCommandWork(lease)
        if not isinstance(lease.payload, FinalApproachQueuedFrame):
            self._slot.finish(lease)
            return None
        payload = lease.payload
        return FinalApproachCommandWork(payload.frame, payload.diagnostic_token, lease)

    def execute_work(
        self,
        work: FinalApproachCommandWork | FinalApproachHeldCommandWork,
    ) -> CalcData | None:
        if isinstance(work, FinalApproachHeldCommandWork):
            issue = self._slot.execute_if_current(
                work.lease,
                self._hold.issue,
            )
            if not isinstance(issue, FinalApproachHeldIssue):
                return None
            self._hold.record(issue)
            return issue.command.calc_data()
        return self._executor.execute(work)

    def finish_work(
        self,
        work: FinalApproachCommandWork | FinalApproachHeldCommandWork,
    ) -> None:
        self._slot.finish(work.lease)

    def postprocess_job(
        self,
        work: FinalApproachCommandWork | FinalApproachHeldCommandWork,
    ) -> Callable[[], None] | None:
        if isinstance(work, FinalApproachCommandWork):
            return self._executor.postprocess_job(work)
        return None


class FinalApproachSessionRuntime:
    """Own final-approach lifecycle, source continuity, liveness, and status state."""

    def __init__(
        self,
        lock: threading.RLock,
        epochs: SourceEpochLedger,
        command_reset: FinalApproachCommandReset,
        visual_pass: VisualPassDetector,
        status: FinalApproachRuntimeStatus,
        liveness: FinalApproachCommandLiveness,
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

    def poi_passed_override(self) -> bool:
        with self._lock:
            return self._visual_pass.passed

    def last_measured_lateral_bearing_deg(self) -> float | None:
        return self._status.last_measured_lateral_bearing_deg()


class VisionNavRuntime:
    """Expose the two focused final-approach runtime capabilities."""

    def __init__(
        self,
        commands: FinalApproachCommandWorkRuntime,
        session: FinalApproachSessionRuntime,
    ) -> None:
        self._commands = commands
        self._session = session

    def nav(self, detect_data: DetectedObject) -> bool:
        return self._commands.nav(detect_data)

    def has_command_pending_or_in_flight(self) -> bool:
        return self._commands.has_command_pending_or_in_flight()

    def take_work(self) -> FinalApproachCommandWork | FinalApproachHeldCommandWork | None:
        return self._commands.take_work()

    def execute_work(
        self,
        work: FinalApproachCommandWork | FinalApproachHeldCommandWork,
    ) -> CalcData | None:
        return self._commands.execute_work(work)

    def finish_work(
        self,
        work: FinalApproachCommandWork | FinalApproachHeldCommandWork,
    ) -> None:
        self._commands.finish_work(work)

    def postprocess_job(
        self,
        work: FinalApproachCommandWork | FinalApproachHeldCommandWork,
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

    def poi_passed_override(self) -> bool:
        return self._session.poi_passed_override()

    def last_measured_lateral_bearing_deg(self) -> float | None:
        return self._session.last_measured_lateral_bearing_deg()


@dataclass(frozen=True)
class FinalApproachHeldCommandWork:
    lease: NavigationCommandLease


_HELD_COMMAND = object()


__all__ = [
    "FinalApproachCommandWorkRuntime",
    "FinalApproachHeldCommandWork",
    "FinalApproachSessionRuntime",
    "VisionNavRuntime",
]
