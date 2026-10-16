"""DetectorPollTimer rows: stage costs, CPU/wall scope, oversleep, residual."""

from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.core.kernel import runtime_time as rt
from navpy.sim.detection.detector_poll_timer import DetectorPollTimer


class _Clocks:
    def __init__(self) -> None:
        self.perf = 100.0
        self.cpu = 5.0
        self.thread = 1

    def advance(self, wall_s: float, cpu_s: float = 0.0) -> None:
        self.perf += wall_s
        self.cpu += cpu_s


@pytest.fixture
def clocks(monkeypatch: pytest.MonkeyPatch) -> _Clocks:
    state = _Clocks()
    monkeypatch.setattr(rt, "perf_counter", lambda: state.perf)
    monkeypatch.setattr(rt, "time", lambda: state.perf + 1_000.0)
    monkeypatch.setattr(rt, "thread_time", lambda: state.cpu)
    monkeypatch.setattr(rt, "get_ident", lambda: state.thread)
    return state


def _iteration(timer: DetectorPollTimer, clocks: _Clocks, *, wait_s: float,
               requested_s: float, outcome: str = "paired") -> None:
    timer.begin()
    clocks.advance(0.002, cpu_s=0.002)
    timer.mark("read")
    timer.frame()
    clocks.advance(0.010, cpu_s=0.004)  # off-CPU half: CPU < wall
    timer.mark("project")
    clocks.advance(0.003, cpu_s=0.003)
    timer.mark("publish")
    clocks.advance(0.001, cpu_s=0.001)  # the evidence-row write
    timer.mark("log")
    timer.end_work()
    clocks.advance(wait_s, cpu_s=0.001)  # the wait's CPU is not work CPU
    timer.finish(outcome, requested_s)


def test_row_has_stage_costs_cpu_in_work_scope_and_oversleep(clocks: _Clocks) -> None:
    rows: list[tuple] = []
    timer = DetectorPollTimer(rows.append, Mock())

    _iteration(timer, clocks, wait_s=0.030, requested_s=0.018)

    (row,) = rows
    assert row[0] == pytest.approx(1_100.0)
    assert row[1] == "paired"
    assert row[2] == pytest.approx(16.0)  # work_ms
    assert row[3] == pytest.approx(10.0)  # work_cpu_ms: same scope, no wait CPU
    assert row[4] == pytest.approx(18.0)  # requested wait
    assert row[5] == pytest.approx(30.0)  # actual wait: 12 ms oversleep
    assert row[6:10] == pytest.approx((2.0, 10.0, 3.0, 1.0))
    assert row[10] == ""  # no previous iteration


def test_residual_is_start_to_start_minus_work_and_wait(clocks: _Clocks) -> None:
    rows: list[tuple] = []
    timer = DetectorPollTimer(rows.append, Mock())

    _iteration(timer, clocks, wait_s=0.0, requested_s=0.0, outcome="empty")
    clocks.advance(0.004)  # wake-up, GIL and row cost before the next begin
    _iteration(timer, clocks, wait_s=0.0, requested_s=0.0)

    assert rows[0][4] == rows[0][5] == pytest.approx(0.0)  # zero wait
    assert rows[1][10] == pytest.approx(4.0)


def test_frame_ended_before_projection_is_charged_to_project(clocks: _Clocks) -> None:
    rows: list[tuple] = []
    timer = DetectorPollTimer(rows.append, Mock())

    timer.begin()
    timer.mark("read")
    timer.frame()
    clocks.advance(0.005)
    timer.mark("log")  # detection returned before projection (inactive)
    timer.end_work()
    timer.finish("paired", 0.0)

    assert rows[0][7] == pytest.approx(5.0) and rows[0][9] == 0.0


def test_unpaired_logging_is_charged_to_log(clocks: _Clocks) -> None:
    rows: list[tuple] = []
    timer = DetectorPollTimer(rows.append, Mock())

    timer.begin()
    timer.mark("read")
    clocks.advance(0.002)
    timer.mark("log")
    timer.end_work()
    timer.finish("unpaired", 0.0)

    assert rows[0][9] == pytest.approx(2.0) and rows[0][7] == 0.0


def test_marks_from_other_threads_are_ignored(clocks: _Clocks) -> None:
    rows: list[tuple] = []
    timer = DetectorPollTimer(rows.append, Mock())

    timer.begin()
    clocks.advance(0.005)
    clocks.thread = 2
    timer.mark("read")
    clocks.thread = 1
    timer.end_work()
    timer.finish("empty", 0.0)

    assert rows[0][6] == 0.0


def test_sink_failure_warns_once_and_does_not_raise(clocks: _Clocks) -> None:
    warning = Mock()
    timer = DetectorPollTimer(Mock(side_effect=OSError("disk full")), warning)

    _iteration(timer, clocks, wait_s=0.0, requested_s=0.0)
    _iteration(timer, clocks, wait_s=0.0, requested_s=0.0)

    warning.assert_called_once()
    assert "disk full" in warning.call_args.args[0]


def test_a_failing_warning_does_not_raise_either(clocks: _Clocks) -> None:
    timer = DetectorPollTimer(
        Mock(side_effect=OSError("disk full")), Mock(side_effect=RuntimeError("no log"))
    )

    _iteration(timer, clocks, wait_s=0.0, requested_s=0.0)
