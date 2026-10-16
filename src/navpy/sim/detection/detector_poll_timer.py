"""Per-iteration timing of the wall-paced detector poll loop (diagnostics only).

One row per poll iteration, empty reads included: iteration start, work time
and the same scope's thread CPU time, the requested and the actual wait, the
stage costs (read/pair, project, publish, logging), and the residual of the
previous iteration (its start-to-start time minus its work and actual wait:
wake-up, GIL reacquisition and this recorder's own row). CPU much below wall
means the thread was off-CPU (preemption, GIL/lock, I/O); CPU close to wall
means it was running (slow core, more work). ``thread_time`` is coarse on
Windows (15.625 ms), so read CPU summed over a window, not per row.

Nothing here feeds the loop's wait, the frame or any command. A failing row
sink is logged once and never stops detection.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.sim.detection.sim_runtime_ports import WarningSink

POLL_STAGES = ("read", "project", "publish", "log")
#: One poll row: start_wall_s, outcome, work_ms, work_cpu_ms,
#: requested_wait_ms, actual_wait_ms, read_ms, project_ms, publish_ms, log_ms,
#: residual_before_ms (empty on the first row).
PollRow = tuple
PollRowSink = Callable[[PollRow], None]


def _ms(seconds: float) -> float:
    return seconds * 1000.0


class _Stamp:
    """One instant on the wall, perf and thread-CPU clocks."""

    __slots__ = ("wall_s", "perf_s", "cpu_s")

    def __init__(self) -> None:
        self.wall_s = rt.time()
        self.cpu_s = rt.thread_time()
        self.perf_s = rt.perf_counter()


class DetectorPollTimer:
    """Stage clock of the detector poll thread; other threads' marks are ignored."""

    def __init__(self, sink: PollRowSink, warning: WarningSink) -> None:
        self._sink = sink
        self._warning = warning
        self._warned = False
        self._thread: Optional[int] = None
        self._start: Optional[_Stamp] = None
        self._work_end: Optional[_Stamp] = None
        self._mark_s = 0.0
        self._stages: dict[str, float] = {}
        self._projected = False
        self._in_frame = False
        # (start, work, actual wait) of the previous iteration, in seconds.
        self._previous: Optional[tuple[float, float, float]] = None

    def begin(self) -> None:
        self._thread = rt.get_ident()
        self._start = _Stamp()
        self._work_end = None
        self._mark_s = self._start.perf_s
        self._stages = dict.fromkeys(POLL_STAGES, 0.0)
        self._projected = False
        self._in_frame = False

    def frame(self) -> None:
        """A paired pose goes to detection: its stages follow."""
        if self._thread == rt.get_ident():
            self._in_frame = True

    def mark(self, stage: str) -> None:
        """Charge the time since the last mark to ``stage``.

        A frame that ends before projection (inactive, no timestamp) has no
        publish or log stage: its time stays with ``project``.
        """
        if self._thread != rt.get_ident() or stage not in self._stages:
            return
        if self._in_frame and not self._projected and stage in ("publish", "log"):
            stage = "project"
        if stage == "project":
            self._projected = True
        now_s = rt.perf_counter()
        self._stages[stage] += now_s - self._mark_s
        self._mark_s = now_s

    def end_work(self) -> None:
        self._work_end = _Stamp()

    def finish(self, outcome: str, requested_wait_s: float) -> None:
        """Close the iteration after its wait and hand its row to the sink."""
        end_s = rt.perf_counter()
        start, work_end = self._start, self._work_end
        if start is None or work_end is None:
            return
        work_s = work_end.perf_s - start.perf_s
        wait_s = end_s - work_end.perf_s
        residual = ""
        if self._previous is not None:
            previous_start, previous_work, previous_wait = self._previous
            residual = _ms(
                start.perf_s - previous_start - previous_work - previous_wait
            )
        self._previous = (start.perf_s, work_s, wait_s)
        stages = self._stages
        row = (
            start.wall_s,
            outcome,
            _ms(work_s),
            _ms(work_end.cpu_s - start.cpu_s),
            _ms(requested_wait_s),
            _ms(wait_s),
            *(_ms(stages[stage]) for stage in POLL_STAGES),
            residual,
        )
        try:
            self._sink(row)
        except Exception as error:  # noqa: BLE001 - diagnostics never stop detection
            self._warn_once(error)

    def _warn_once(self, error: Exception) -> None:
        if self._warned:
            return
        self._warned = True
        try:
            self._warning(
                "DetectorSim poll diagnostics failed: "
                f"{type(error).__name__}: {error}"
            )
        except Exception:  # noqa: BLE001 - nor does a failing warning
            pass


__all__ = ["DetectorPollTimer", "POLL_STAGES", "PollRow", "PollRowSink"]
