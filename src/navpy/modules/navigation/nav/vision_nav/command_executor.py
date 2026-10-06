"""Lease-fenced execution and post-fence diagnostic cleanup."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from navpy.exception_groups import BaseExceptionGroup
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_command_slot import (
    NavigationCommandLease,
    NavigationCommandSlot,
)
from navpy.modules.common.resource_cleanup import CleanupStack
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    TerminalCommandOutcome,
    TerminalCommandResult,
    TerminalCommandTransaction,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    TerminalCommandHoldStore,
)
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    TerminalCommandFreshness,
    TerminalCommandTiming,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    TerminalCommandFailureSink,
)
from navpy.modules.navigation.navigation_postprocess_dispatcher import PostprocessJob
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    TerminalPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    TerminalDiagnosticEntry,
    TerminalDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.diagnostics import (
    TerminalCommandDiagnostics,
)
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.runtime_state import TerminalRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_time_ports import (
    TerminalCommandSourceTimeObserver,
)


@dataclass(frozen=True)
class TerminalDiagnosticCompletion:
    diagnostic: object | None
    evidence_errors: tuple[BaseException, ...]


@dataclass
class TerminalCommandWork:
    frame: TerminalVisionFrame
    diagnostic_token: int
    lease: NavigationCommandLease
    _postprocess_fence: TerminalPostprocessFence | None = field(
        default=None,
        init=False,
        repr=False,
        compare=False,
    )
    _completion: TerminalDiagnosticCompletion | None = field(
        default=None,
        init=False,
        repr=False,
        compare=False,
    )

    def begin_postprocess(self, fence: TerminalPostprocessFence) -> None:
        if self._postprocess_fence is not None:
            raise RuntimeError("terminal command work executed twice")
        self._postprocess_fence = fence
        fence.begin()

    def capture_completion(
        self,
        completion: TerminalDiagnosticCompletion,
    ) -> None:
        self._completion = completion

    @property
    def has_postprocess(self) -> bool:
        return self._postprocess_fence is not None

    def take_completion(
        self,
    ) -> tuple[TerminalPostprocessFence, TerminalDiagnosticCompletion | None] | None:
        fence = self._postprocess_fence
        if fence is None:
            return None
        completion = self._completion
        self._postprocess_fence = None
        self._completion = None
        return fence, completion


@dataclass(frozen=True)
class TerminalExecutorPorts:
    slot: NavigationCommandSlot
    transaction: TerminalCommandTransaction
    mailbox: TerminalDiagnosticMailbox
    diagnostics: TerminalCommandDiagnostics
    status: TerminalRuntimeStatus
    source_time: TerminalCommandSourceTimeObserver
    hold: TerminalCommandHoldStore
    failures: TerminalCommandFailureSink
    freshness: TerminalCommandFreshness
    postprocess_fence: TerminalPostprocessFence


class TerminalCommandExecutor:
    """Keep the lease fence limited to plan/issue/commit."""

    def __init__(self, ports: TerminalExecutorPorts) -> None:
        self._ports = ports

    def execute(self, work: TerminalCommandWork) -> CalcData | None:
        result: TerminalCommandResult | None = None
        entry: list[TerminalDiagnosticEntry | None] = [None]
        timing = self._ports.mailbox.timing(work.diagnostic_token)
        entry_s = time.perf_counter()
        work.begin_postprocess(self._ports.postprocess_fence)
        with CleanupStack() as cleanup:
            cleanup.push(
                lambda: self._capture_completion(
                    work,
                    result,
                    entry[0],
                )
            )
            cleanup.push(
                lambda: self._record_source_time(
                    work,
                    result,
                    entry[0],
                    entry_s,
                )
            )
            cleanup.push(
                lambda: entry.__setitem__(
                    0,
                    self._ports.mailbox.pop(work.diagnostic_token),
                )
            )
            result = self._ports.slot.execute_if_current(
                work.lease,
                lambda: self._execute_current(work, timing),
            )
        return None if result is None else result.calc_data

    def postprocess_job(self, work: TerminalCommandWork) -> PostprocessJob | None:
        if not work.has_postprocess:
            return None
        return PostprocessJob(
            execute=lambda: self.postprocess(work),
            abandon=lambda: self.abandon_postprocess(work),
        )

    @staticmethod
    def abandon_postprocess(work: TerminalCommandWork) -> None:
        owned = work.take_completion()
        if owned is None:
            return
        fence, _completion = owned
        fence.finish()

    def postprocess(self, work: TerminalCommandWork) -> None:
        owned = work.take_completion()
        if owned is None:
            return
        fence, completion = owned
        try:
            if completion is None:
                return
            if completion.evidence_errors:
                if len(completion.evidence_errors) == 1:
                    raise completion.evidence_errors[0]
                raise BaseExceptionGroup(
                    "terminal command evidence capture failed",
                    completion.evidence_errors,
                )
            if completion.diagnostic is not None:
                self._ports.diagnostics.record(completion.diagnostic)
        finally:
            fence.finish()

    def _capture_completion(
        self,
        work: TerminalCommandWork,
        result: TerminalCommandResult | None,
        entry: TerminalDiagnosticEntry | None,
    ) -> None:
        status_error: BaseException | None = None
        try:
            if result is not None:
                self._ports.status.record(work.frame)
        except BaseException as error:
            status_error = error

        evidence_errors: list[BaseException] = []
        diagnostic: object | None = None
        try:
            diagnostic = self._ports.diagnostics.capture(
                work.frame,
                None if entry is None else entry.target,
                result,
            )
        except BaseException as error:
            evidence_errors.append(error)
        work.capture_completion(TerminalDiagnosticCompletion(
            diagnostic=diagnostic,
            evidence_errors=tuple(evidence_errors),
        ))
        if status_error is not None:
            raise status_error

    def _execute_current(
        self,
        work: TerminalCommandWork,
        timing: TerminalCommandTiming | None,
    ) -> TerminalCommandResult | None:
        ports = self._ports
        ports.hold.clear()
        if timing is None or not ports.freshness.is_fresh(timing):
            ports.failures.mark_failed()
            return None
        try:
            result = ports.transaction.execute(work.frame)
        except Exception:
            ports.failures.mark_failed()
            raise
        if result.outcome is TerminalCommandOutcome.LAW_UNAVAILABLE:
            ports.failures.mark_failed()
        if result.issued and result.calc_data is not None:
            ports.hold.remember(
                result.calc_data,
                timing=timing,
            )
        return result

    def _record_source_time(
        self,
        work: TerminalCommandWork,
        result: TerminalCommandResult | None,
        entry: TerminalDiagnosticEntry | None,
        entry_s: float,
    ) -> None:
        if result is None or not result.issued:
            return
        self._ports.source_time.record_command(
            wall_start_s=entry_s,
            source_timestamp_s=work.frame.source_timestamp_s,
            source_now_s=(
                None if entry is None else entry.timing.source_now_s
            ),
            execution_ms=max(
                0.0,
                (time.perf_counter() - entry_s) * 1000.0,
            ),
            outcome="fresh",
        )


__all__ = [
    "TerminalCommandExecutor",
    "TerminalCommandWork",
    "TerminalDiagnosticCompletion",
    "TerminalExecutorPorts",
]
