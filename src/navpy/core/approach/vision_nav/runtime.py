"""Small source-driven runtime for pure-vision final approach."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Iterable, Union

from navpy.core.approach.calc_data import CalcData
from navpy.core.command.navigation_command_slot import (
    NavigationCommandLease,
    NavigationCommandSlot,
)
from navpy.core.approach.vision_nav.command_executor import (
    FinalApproachCommandExecutor,
    FinalApproachCommandWork,
)
from navpy.core.approach.vision_nav.command_hold import (
    FinalApproachCommandHoldIssuer,
    FinalApproachHeldIssue,
)
from navpy.core.approach.vision_nav.command_liveness import (
    FinalApproachCommandLiveness,
)
from navpy.core.approach.vision_nav.command_reset import (
    FinalApproachCommandReset,
)
from navpy.core.approach.vision_nav.ingress import FinalApproachIngress
from navpy.core.approach.vision_nav.docked_evidence import (
    FinalApproachDockedEvidence,
    FinalApproachDockedEvidenceWork,
)
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeLatch
from navpy.core.approach.vision_nav.queued_frame import FinalApproachQueuedFrame
from navpy.core.approach.vision_nav.runtime_state import FinalApproachRuntimeStatus
from navpy.core.approach.vision_nav.source_epoch import SourceEpochLedger
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity


class FinalApproachCommandWorkRuntime:
    """Own command admission, leases, execution, and zero-order hold work."""

    def __init__(
        self,
        slot: NavigationCommandSlot,
        ingress: FinalApproachIngress,
        executor: FinalApproachCommandExecutor,
        hold: FinalApproachCommandHoldIssuer,
        docked_evidence: FinalApproachDockedEvidence,
    ) -> None:
        self._slot = slot
        self._ingress = ingress
        self._executor = executor
        self._hold = hold
        self._docked_evidence = docked_evidence

    def nav(self, detect_data: DetectedObject) -> bool:
        return self._ingress.nav(detect_data)

    def has_command_pending_or_in_flight(self) -> bool:
        return (
            self._slot.has_pending_or_in_flight()
            or self._hold.available
            or self._docked_evidence.pending
        )

    def take_work(self) -> _Work | None:
        # Staged DOCKED evidence first: its row must follow every earlier
        # command row and precede any newer frame's work.
        evidence = self._docked_evidence.take_work()
        if evidence is not None:
            return evidence
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

    def execute_work(self, work: _Work) -> CalcData | None:
        if isinstance(work, FinalApproachDockedEvidenceWork):
            return None
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

    def finish_work(self, work: _Work) -> None:
        if isinstance(work, FinalApproachDockedEvidenceWork):
            return
        self._slot.finish(work.lease)

    def postprocess_job(self, work: _Work) -> Callable[[], None] | None:
        if isinstance(work, FinalApproachDockedEvidenceWork):
            return work.postprocess_job()
        if isinstance(work, FinalApproachCommandWork):
            return self._executor.postprocess_job(work)
        return None


class FinalApproachSessionRuntime:
    """Own final-approach lifecycle, source continuity, liveness, and status state."""

    def __init__(
        self,
        lock: rt.RLock,
        epochs: SourceEpochLedger,
        command_reset: FinalApproachCommandReset,
        episode: FinalApproachEpisodeLatch,
        status: FinalApproachRuntimeStatus,
        liveness: FinalApproachCommandLiveness,
        docked_evidence: FinalApproachDockedEvidence,
    ) -> None:
        self._lock = lock
        self._epochs = epochs
        self._command_reset = command_reset
        self._episode = episode
        self._status = status
        self._liveness = liveness
        self._docked_evidence = docked_evidence

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
        """The approach completed: the episode docked (contact, not vision)."""
        with self._lock:
            return self._episode.docked

    def begin_dock_episode(self, identity: DockContactIdentity) -> None:
        with self._lock:
            self._episode.begin(identity)
        # NAV entry: no command row of this episode exists yet.
        self._docked_evidence.record_armed(identity)

    def latch_docked(self, event: DockContactEvent) -> bool:
        """Latch the contact, retire command work and stage DOCKED evidence.

        Runs on the contact callback's thread: it latches and fences only.
        The navigation loop thread picks the completion up in its next
        decision (poi_passed_override) and requests RESET there.
        """
        with self._lock:
            if not self._episode.latch_docked(event):
                return False
            # Terminal: nothing may actuate after docking (no held refresh,
            # no queued frame); the episode latch itself is kept.
            self._command_reset.retire_after_docking()
            self._docked_evidence.stage(event)
            return True

    def latch_failure(self) -> bool:
        """Admit a NAV_FAIL; False once the episode docked (first latch wins)."""
        with self._lock:
            return self._episode.latch_failure()

    def consume_missed_dock(self) -> bool:
        with self._lock:
            return self._episode.consume_missed()

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

    def take_work(self) -> _Work | None:
        return self._commands.take_work()

    def execute_work(
        self,
        work: _Work,
    ) -> CalcData | None:
        return self._commands.execute_work(work)

    def finish_work(
        self,
        work: _Work,
    ) -> None:
        self._commands.finish_work(work)

    def postprocess_job(
        self,
        work: _Work,
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

    @property
    def session(self) -> FinalApproachSessionRuntime:
        """The lifecycle/status capability (episode latch, completion)."""
        return self._session

    def last_measured_lateral_bearing_deg(self) -> float | None:
        return self._session.last_measured_lateral_bearing_deg()


@dataclass(frozen=True)
class FinalApproachHeldCommandWork:
    lease: NavigationCommandLease


_HELD_COMMAND = object()
_Work = Union[
    FinalApproachCommandWork,
    FinalApproachHeldCommandWork,
    FinalApproachDockedEvidenceWork,
]


__all__ = [
    "FinalApproachCommandWorkRuntime",
    "FinalApproachHeldCommandWork",
    "FinalApproachSessionRuntime",
    "VisionNavRuntime",
]
