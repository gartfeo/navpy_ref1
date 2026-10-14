"""The participant loop's tick, in process: admission, input order, the end, faults."""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_sync import sleep
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.part_wire import Event, Lifecycle, PartWireError, Request, Tick
from navpy.sim.lockstep.participant import LockstepIdentity, Participant
from navpy.sim.lockstep.participant_hooks import Outbox, OutboxBuffer, ParticipantHooks
from navpy.sim.lockstep.participant_loop import ParticipantLoop

DEVICE = "lockstep:serial0.1"


class Link:
    def __init__(self, order: list) -> None:
        self._order = order

    def deliver(self, data: bytes) -> None:
        self._order.append(("bytes", data))

    def drain_egress(self) -> bytes:
        return b"out"


class Hooks(ParticipantHooks):
    def __init__(self, baton: BatonRuntime, *, fail: bool = False, bootstrap_s: float = 0.0) -> None:
        self.order: list = []
        self.links = {DEVICE: Link(self.order)}
        self.out = OutboxBuffer()
        self._baton = baton
        self._fail = fail
        self._bootstrap_s = bootstrap_s
        self.ended = False
        self.flushed_at: list[int] = []

    def construct(self) -> None:
        if self._fail:
            raise RuntimeError("construction failed")
        sleep(self._baton, self._bootstrap_s)
        self.out.events.append(Event("runtime_ready", None))

    def on_end(self) -> None:
        self.order.append(("end",))
        self.ended = True

    def on_lifecycle(self, entry) -> None:
        self.order.append(("lifecycle", entry.kind))

    def on_cancel(self, request_id) -> None:
        self.order.append(("cancel", request_id))

    def on_request(self, request) -> None:
        self.order.append(("request", request.id))

    def finished(self) -> bool:
        return self.ended

    def flushes(self):
        return [lambda: self.flushed_at.append(self._baton.now_us)]

    def drain_outbox(self) -> Outbox:
        return self.out.take()


def _loop(**kwargs) -> tuple[ParticipantLoop, Hooks]:
    baton = BatonRuntime()
    participant = Participant(
        LockstepRuntime.create(baton, epoch_us=0, seed=1, ordinal=1), LockstepIdentity(1, 1), ())
    hooks = Hooks(baton, **kwargs)
    return ParticipantLoop(participant, hooks, link=None), hooks


def test_inputs_arrive_in_tick_input_order():
    loop, hooks = _loop()
    done = loop.step(Tick(5, 100_000, ((DEVICE, b"ab"),), (Request((0, 2), 1), Request((0, 3), 2)),
                          (Lifecycle("stop", (0, 7), (2, 1), 1, 1),), ((0, 1),), end=True))
    assert hooks.order == [("end",), ("lifecycle", "stop"), ("cancel", (0, 1)),
                           ("request", (0, 2)), ("request", (0, 3)), ("bytes", b"ab")]
    assert done.k == 5 and done.ports == ((DEVICE, b"out"),)
    assert done.ended is True and hooks.flushed_at == [100_000]  # ended within the tick


def test_the_admission_tick_builds_the_product_and_readiness_comes_later():
    loop, _hooks = _loop(bootstrap_s=0.05)
    dones = [loop.step(Tick(k, k * 20_000)) for k in range(7, 11)]
    assert [bool(d.events) for d in dones] == [False, False, False, True]  # 0.06 s >= 0.05 s
    assert all(d.ended is None for d in dones)


def test_a_duplicate_or_a_missing_tick_is_refused():
    loop, _hooks = _loop()
    loop.step(Tick(3, 60_000))
    with pytest.raises(PartWireError, match="missing or duplicate"):
        loop.step(Tick(3, 60_000))
    with pytest.raises(PartWireError, match="missing or duplicate"):
        loop.step(Tick(5, 100_000))


def test_a_construction_failure_ends_with_a_fault_unflushed():
    loop, _hooks = _loop(fail=True)
    done = loop.step(Tick(1, 20_000))
    assert done.ended is False
    assert done.faults == ("RuntimeError:tick",)  # a stable kind; the text goes to stderr
