"""Tracing changes no decision, through the REAL router.

Both arms run one controlled schedule through the real router, store and
registry (direct_pixel_router_bench), the real source and the real
NavigationCommandWorker. Only tracing differs, and the subscription it brings:
the traced arm subscribes the ATTITUDE ledger before the source starts, as
the child does. Every stamp and every receipt comes from the schedule, never
a clock, so the schedule alone decides each pairing. The traced arm has to
record NONZERO admissions, passes, lifecycle and subscription rows, and
the commands the worker executed as its command entries, each in the pass
that executed it, so an unwired or misnumbered recorder fails it, and to be
ELIGIBLE; its frame digests, command bytes, taken outcomes and metrics
have to equal the untraced arm's. The command bytes, and the pass each ran
in, are read from an independent sink both arms fill alike
(LANDING2 step-1 plan, plan review round 6,
R4).
"""

from __future__ import annotations

import contextlib
import itertools
import threading
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any
from unittest.mock import Mock

from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.modules.vehicle import inbound_router
from navpy.modules.vehicle.inbound_router import REJECT_STALE_BOOT
from navpy.modules.vision.sim import determinism_trace
from navpy.modules.vision.sim.determinism_eligibility import ELIGIBLE, judge
from navpy.modules.vision.sim.determinism_events import (
    EVENT_LIFECYCLE,
    EVENT_SUBSCRIPTION,
    LIFECYCLE_ACTIVATED,
    LIFECYCLE_CLOSED,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
)
from navpy.modules.vision.sim.determinism_evidence import sealed_capture
from navpy.modules.vision.sim.determinism_slots import (
    PAYLOAD_UNREADABLE,
    command_digest,
    frame_digest,
)
from navpy.modules.vision.sim.direct_pixel_trace import command_loop_observer
from navpy.modules.vision.sim.direct_poi_pixel_source import (
    DirectPoiPixelSource,
)
from tests.modules.vision import determinism_case_factory as factory
from tests.modules.vision.direct_pixel_router_bench import (
    POI,
    RouterVehicle,
    attitude_message,
    sim_state_message,
)

# The source test's neutrality schedule (test_direct_poi_pixel_source.py)
# with one stale ATTITUDE, ``a``, that the router rejects: one clean frame; a
# refused association; two renders with only one dispatch between; then a
# dispatch with nothing to take. T is a truth sample, A an ATTITUDE just after
# it, D one worker pass.
SCHEDULE = "T T A T D A a A T D A T A T D D"


class _RouterSchedule:
    """The schedule as MAVLink through the router. Truth advances one pose
    period on both the source and the receipt axis, each ATTITUDE is stamped
    just after the newest truth sample, so the next one brackets it, and each
    message is received at the time the schedule gives it."""

    FIRST_TRUTH_US = 12_475_000
    TRUTH_PERIOD_US = 25_000
    ATTITUDE_INTO_PERIOD_US = 10_000
    FIRST_TRUTH_RECEIPT_S = 100.0
    TRUTH_PERIOD_S = 0.025
    ATTITUDE_RECEIPT_LAG_S = 0.005
    # Under the last admitted ATTITUDE, so the router rejects it as stale.
    STALE_MS = 10

    def __init__(self, vehicle: RouterVehicle, receipts: SimpleNamespace):
        self._vehicle = vehicle
        self._receipts = receipts
        self._index = 0
        self._truth_us = self.FIRST_TRUTH_US
        self._truth_receipt_s = self.FIRST_TRUTH_RECEIPT_S
        self._admitted_ms: int | None = None

    def _ingest(self, message: Any, receipt_s: float) -> None:
        self._receipts.now = receipt_s
        self._vehicle.ingest(message)

    def truth(self) -> None:
        self._truth_us = (
            self.FIRST_TRUTH_US + self._index * self.TRUTH_PERIOD_US
        )
        self._truth_receipt_s = (
            self.FIRST_TRUTH_RECEIPT_S + self._index * self.TRUTH_PERIOD_S
        )
        self._index += 1
        self._ingest(sim_state_message(self._truth_us), self._truth_receipt_s)

    def attitude(self) -> None:
        self._admitted_ms = (
            self._truth_us + self.ATTITUDE_INTO_PERIOD_US
        ) // 1000
        self._ingest(
            attitude_message(self._admitted_ms),
            self._truth_receipt_s + self.ATTITUDE_RECEIPT_LAG_S,
        )

    def stale(self) -> None:
        self._ingest(
            attitude_message(self._admitted_ms - self.STALE_MS),
            self._truth_receipt_s + self.ATTITUDE_RECEIPT_LAG_S,
        )

    def run(self, script: str) -> None:
        steps = {"T": self.truth, "A": self.attitude, "a": self.stale}
        for step in script.split():
            steps[step]()


class _ProxyLaw:
    """A consumer with memory, as in the source test: each command is formed
    against the frame before it, so a different coalescing choice would show
    in what it emits. It also keeps the frames, for their digests."""

    def __init__(self, queue: list) -> None:
        self.commands: list[tuple[float, float, float]] = []
        self.frames: list[Any] = []
        self._queue = queue
        self._previous: tuple[float, float] | None = None

    def __call__(self, detection: Any) -> bool:
        pixel = detection.pixel
        current = (float(pixel.source_timestamp_s), float(pixel.u_px))
        previous = self._previous
        span = 0.0 if previous is None else current[0] - previous[0]
        formed = (current[0], span, current[1])
        self.frames.append(detection)
        self.commands.append(formed)
        # Queued, not issued: the worker takes work at the top of its NEXT
        # pass.
        self._queue.append(formed)
        self._previous = current
        return True


class _ExecutingRuntime:
    """Executes the queued work and returns a command. ``executed`` is the
    independent sink: filled alike whether tracing is on or off, so it is
    the control, not a second reading of the recorder. Each command goes in
    with the pass it ran in, ``pass_now``, counted from the arm's own
    dispatches: the worker executes a pass's work before that pass
    dispatches (NavigationCommandWorker.run)."""

    def __init__(
        self, queue: list, executed: list, pass_now: Callable[[], int]
    ) -> None:
        self._queue = queue
        self.executed = executed
        self._pass_now = pass_now

    def has_command_pending_or_in_flight(self) -> bool:
        return bool(self._queue)

    def take_work(self) -> Any:
        return self._queue.pop(0) if self._queue else None

    def execute_work(self, work: Any) -> CalcData:
        source_s, span_s, u_px = work
        command = CalcData(u_px, span_s, source_s, None, 0.5)
        self.executed.append((self._pass_now(), command))
        return command

    def finish_work(self, work: Any) -> None:
        return None

    def postprocess_job(self, work: Any) -> None:
        return None


def _arm(monkeypatch, *, tracing: bool) -> dict[str, Any]:
    """One arm: the schedule through the router and the real worker, one
    worker pass per D and one past the last, so the work the final dispatch
    queued is executed. The worker's clock is driven, never slept."""
    monkeypatch.setattr(determinism_trace, "ENABLED", tracing)
    receipts = SimpleNamespace(now=0.0)
    monkeypatch.setattr(
        inbound_router, "time", SimpleNamespace(time=lambda: receipts.now)
    )
    vehicle = RouterVehicle()
    queue: list = []
    executed: list = []
    law = _ProxyLaw(queue)
    source = DirectPoiPixelSource(
        vehicle,
        POI,
        SimpleNamespace(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=law,
        wall_now_s=lambda: 100.0,
    )
    trace = source.determinism_trace
    assert (trace is not None) is tracing
    subscription = (
        None if trace is None
        else vehicle.on_admission("ATTITUDE", trace.ledger.append)
    )
    source.start()
    source.activate()
    schedule = _RouterSchedule(vehicle, receipts)
    groups = SCHEDULE.split("D")[:-1]
    pending = iter(groups)
    stop_event = threading.Event()
    taken: list[bool] = []

    def source_dispatch() -> bool:
        group = next(pending, None)
        if group is not None:
            schedule.run(group)
        result = source.dispatch_available()
        taken.append(result)
        if len(taken) > len(groups):
            stop_event.set()
        return result

    runtime = _ExecutingRuntime(queue, executed, lambda: len(taken) + 1)

    @contextlib.contextmanager
    def runtime_session():
        yield runtime

    def never_sleep(seconds: float) -> None:
        raise AssertionError("the driven clock is always past the deadline")

    clock = itertools.count(0.0, 1.0)
    NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=threading.Event(),
            wall_period_s=lambda period_s: period_s,
            runtime_session=runtime_session,
            logger=Mock(),
            source_dispatch=source_dispatch,
            command_loop=command_loop_observer(trace),
            monotonic_s=lambda: next(clock),
            sleep_s=never_sleep,
        )
    ).run()
    metrics = source.metrics
    ending = source.close()
    capture = verdict = None
    if subscription is not None:
        subscription.cancel()
        capture = sealed_capture(trace)
        verdict = judge(factory.case_of(
            capture,
            admission=factory.admission(
                first=subscription.first,
                end=subscription.end,
                faults=subscription.faults,
                faulted=subscription.faulted,
            ),
            epoch=ending,
        ))
    return {
        "digests": tuple(frame_digest(frame) for frame in law.frames),
        "commands": tuple(law.commands),
        "executed": tuple(command_digest(command) for _, command in executed),
        "executed_in": tuple(number for number, _ in executed),
        "taken": tuple(taken),
        "metrics": metrics,
        "capture": capture,
        "verdict": verdict,
    }


def _outcomes(capture: Any, event: str) -> list[str]:
    return [row[2] for row in capture.rows if row[0] == event]


def test_tracing_through_the_router_changes_no_decision(monkeypatch) -> None:
    off = _arm(monkeypatch, tracing=False)
    on = _arm(monkeypatch, tracing=True)

    assert off["capture"] is None
    assert on["capture"] is not None
    # The frames the law received, bit for bit, and real ones: a digest
    # that failed to read is not None either.
    assert on["digests"] == off["digests"]
    assert len(on["digests"]) == 3
    assert all(
        digest is not None and digest != PAYLOAD_UNREADABLE
        for digest in on["digests"]
    )
    # The commands formed, each after the first against the frame before.
    assert on["commands"] == off["commands"]
    assert on["commands"][0][1] == 0.0
    assert all(command[1] > 0.0 for command in on["commands"][1:])
    # The commands the worker EXECUTED, from the independent sink.
    assert on["executed"] == off["executed"]
    assert len(on["executed"]) == 3
    assert all(digest is not None for digest in on["executed"])
    # And the pass each ran in, counted by the arm's own dispatches.
    assert on["executed_in"] == off["executed_in"] == (2, 3, 4)
    assert on["taken"] == off["taken"] == (True, True, True, False, False)
    assert on["metrics"] == off["metrics"]


def test_the_traced_arm_records_every_store_and_is_eligible(
    monkeypatch,
) -> None:
    """An unwired recorder fails here: every store holds what the schedule
    made, and D8 finds nothing wrong with it."""
    on = _arm(monkeypatch, tracing=True)
    capture = on["capture"]

    accepted = [entry[2] for entry in capture.ledger.entries]
    assert accepted.count(True) == SCHEDULE.split().count("A")
    assert [
        entry[3] for entry in capture.ledger.entries if not entry[2]
    ] == [REJECT_STALE_BOOT]
    assert len(capture.commands.passes) == len(on["taken"])
    # The command log's entries are the commands the worker executed, as
    # the independent sink holds them: an unwired command recorder fails
    # here (review round 1).
    assert [entry[1] for entry in capture.commands.entries] == list(
        on["executed"]
    )
    # Each in the pass that executed it, from the same sink: a recorder that
    # named a command's pass wrongly, and still named one, fails here
    # (review round 2).
    assert [entry[0] for entry in capture.commands.entries] == list(
        on["executed_in"]
    )
    assert _outcomes(capture, EVENT_LIFECYCLE) == [
        LIFECYCLE_ACTIVATED, LIFECYCLE_CLOSED
    ]
    assert _outcomes(capture, EVENT_SUBSCRIPTION) == [
        SUBSCRIPTION_OPENED, SUBSCRIPTION_CLOSED
    ]
    assert on["verdict"].status == ELIGIBLE, on["verdict"].reasons
