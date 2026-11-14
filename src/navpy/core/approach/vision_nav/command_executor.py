"""Lease-fenced execution and post-fence diagnostic cleanup."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.exception_groups import BaseExceptionGroup
from navpy.core.approach.calc_data import CalcData
from navpy.core.command.navigation_command_slot import (
    NavigationCommandLease,
    NavigationCommandSlot,
)
from navpy.core.kernel.resource_cleanup import CleanupStack
from navpy.core.approach.vision_nav.command_transaction import (
    GuidanceCommandOutcome,
    GuidanceCommandResult,
    GuidanceCommandTransaction,
)
from navpy.core.approach.vision_nav.command_hold import (
    GuidanceCommandHoldStore,
)
from navpy.core.approach.vision_nav.command_freshness import (
    GuidanceCommandFreshness,
    GuidanceCommandTiming,
    GuidanceFreshnessCheck,
)
from navpy.core.approach.vision_nav.command_liveness import (
    GuidanceCommandFailureSink,
)
from navpy.core.approach.navigation_postprocess_dispatcher import PostprocessJob
from navpy.core.approach.vision_nav.command_postprocess import (
    GuidancePostprocessFence,
)
from navpy.core.approach.vision_nav.diagnostic_mailbox import (
    GuidanceDiagnosticEntry,
    GuidanceDiagnosticMailbox,
)
from navpy.core.approach.vision_nav.diagnostics import (
    GuidanceCommandDiagnostics,
)
from navpy.core.ports.frame import GuidanceVisionFrame
from navpy.core.approach.vision_nav.runtime_state import GuidanceRuntimeStatus
from navpy.core.approach.vision_nav.source_time_ports import (
    GuidanceCommandSourceTimeObserver,
)


@dataclass(frozen=True)
class GuidanceDiagnosticCompletion:
    diagnostic: object | None
    evidence_errors: tuple[BaseException, ...]


@dataclass
class GuidanceCommandWork:
    frame: GuidanceVisionFrame
    diagnostic_token: int
    lease: NavigationCommandLease
    _postprocess_fence: GuidancePostprocessFence | None = field(
        default=None,
        init=False,
        repr=False,
        compare=False,
    )
    _completion: GuidanceDiagnosticCompletion | None = field(
        default=None,
        init=False,
        repr=False,
        compare=False,
    )

    def begin_postprocess(self, fence: GuidancePostprocessFence) -> None:
        if self._postprocess_fence is not None:
            raise RuntimeError("guidance command work executed twice")
        self._postprocess_fence = fence
        fence.begin()

    def capture_completion(
        self,
        completion: GuidanceDiagnosticCompletion,
    ) -> None:
        self._completion = completion

    @property
    def has_postprocess(self) -> bool:
        return self._postprocess_fence is not None

    def take_completion(
        self,
    ) -> tuple[GuidancePostprocessFence, GuidanceDiagnosticCompletion | None] | None:
        fence = self._postprocess_fence
        if fence is None:
            return None
        completion = self._completion
        self._postprocess_fence = None
        self._completion = None
        return fence, completion


@dataclass(frozen=True)
class GuidanceExecutorPorts:
    slot: NavigationCommandSlot
    transaction: GuidanceCommandTransaction
    mailbox: GuidanceDiagnosticMailbox
    diagnostics: GuidanceCommandDiagnostics
    status: GuidanceRuntimeStatus
    source_time: GuidanceCommandSourceTimeObserver
    hold: GuidanceCommandHoldStore
    failures: GuidanceCommandFailureSink
    freshness: GuidanceCommandFreshness
    postprocess_fence: GuidancePostprocessFence
    report_stale: Callable[[GuidanceFreshnessCheck], None] = lambda check: None


class GuidanceCommandExecutor:
    """Keep the lease fence limited to plan/issue/commit."""

    def __init__(self, ports: GuidanceExecutorPorts) -> None:
        self._ports = ports

    def execute(self, work: GuidanceCommandWork) -> CalcData | None:
        result: GuidanceCommandResult | None = None
        entry: list[GuidanceDiagnosticEntry | None] = [None]
        timing = self._ports.mailbox.timing(work.diagnostic_token)
        entry_s = rt.perf_counter()
        entry_cpu_s = rt.thread_time()
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
                    entry_cpu_s,
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

    def postprocess_job(self, work: GuidanceCommandWork) -> PostprocessJob | None:
        if not work.has_postprocess:
            return None
        return PostprocessJob(
            execute=lambda: self.postprocess(work),
            abandon=lambda: self.abandon_postprocess(work),
        )

    @staticmethod
    def abandon_postprocess(work: GuidanceCommandWork) -> None:
        owned = work.take_completion()
        if owned is None:
            return
        fence, _completion = owned
        fence.finish()

    def postprocess(self, work: GuidanceCommandWork) -> None:
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
                    "guidance command evidence capture failed",
                    completion.evidence_errors,
                )
            if completion.diagnostic is not None:
                self._ports.diagnostics.record(completion.diagnostic)
        finally:
            fence.finish()

    def _capture_completion(
        self,
        work: GuidanceCommandWork,
        result: GuidanceCommandResult | None,
        entry: GuidanceDiagnosticEntry | None,
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
                None if entry is None else entry.poi,
                result,
            )
        except BaseException as error:
            evidence_errors.append(error)
        work.capture_completion(GuidanceDiagnosticCompletion(
            diagnostic=diagnostic,
            evidence_errors=tuple(evidence_errors),
        ))
        if status_error is not None:
            raise status_error

    def _execute_current(
        self,
        work: GuidanceCommandWork,
        timing: GuidanceCommandTiming | None,
    ) -> GuidanceCommandResult | None:
        ports = self._ports
        ports.hold.clear()
        result = ports.transaction.suppressed_result()
        if result is not None:
            return result
        if timing is None or not ports.freshness.is_fresh(timing, ports.report_stale):
            ports.failures.mark_failed()
            return None
        try:
            result = ports.transaction.execute(work.frame)
        except Exception:
            ports.failures.mark_failed()
            raise
        if result.outcome is GuidanceCommandOutcome.LAW_UNAVAILABLE:
            ports.failures.mark_failed()
        if result.issued and result.calc_data is not None:
            ports.hold.remember(
                result.calc_data,
                timing=timing,
            )
        return result

    def _record_source_time(
        self,
        work: GuidanceCommandWork,
        result: GuidanceCommandResult | None,
        entry: GuidanceDiagnosticEntry | None,
        entry_s: float,
        entry_cpu_s: float,
    ) -> None:
        frame = work.frame
        source_time = self._ports.source_time
        if result is None or not result.issued:
            # Not executed (superseded lease or stale timing) or not issued.
            source_time.record_frame_disposition(
                stage="worker",
                epoch=frame.source_generation,
                source_timestamp_s=frame.source_timestamp_s,
                disposition="stale" if result is None else "unissued",
            )
            return
        source_time.record_command(
            wall_start_s=entry_s,
            source_timestamp_s=frame.source_timestamp_s,
            source_now_s=(
                None if entry is None else entry.timing.source_now_s
            ),
            execution_ms=max(
                0.0,
                (rt.perf_counter() - entry_s) * 1000.0,
            ),
            outcome="fresh",
            # Same scope and thread as execution_ms.
            work_cpu_ms=max(0.0, (rt.thread_time() - entry_cpu_s) * 1000.0),
            epoch=frame.source_generation,
        )
        source_time.record_frame_disposition(
            stage="worker",
            epoch=frame.source_generation,
            source_timestamp_s=frame.source_timestamp_s,
            disposition="consumed",
        )


__all__ = [
    "GuidanceCommandExecutor",
    "GuidanceCommandWork",
    "GuidanceDiagnosticCompletion",
    "GuidanceExecutorPorts",
]
