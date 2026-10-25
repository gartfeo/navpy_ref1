"""A process freeze and a CPU budget in the participant loop (``deterministic_dev_sim.md`` step 4).

The loop asks its handoff cap once per tick: work beyond it stays runnable, a
freeze (cap 0) runs no thread while its inputs still queue, and the timers
that came due fire on resume, after the work, in ``(due, park seq)`` order.
"""

from __future__ import annotations

from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import sleep, yield_now
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.part_wire import Request, Tick
from navpy.sim.lockstep.participant import LockstepIdentity, Participant
from navpy.sim.lockstep.participant_hooks import Outbox, OutboxBuffer, ParticipantHooks
from navpy.sim.lockstep.participant_loop import ParticipantLoop

DEVICE = "lockstep:serial0.1"
TICK_US = 20_000


class Link:
    """The reader's input queue and its egress."""

    def __init__(self, baton: BatonRuntime) -> None:
        self.inbound = BatonQueue(baton)
        self.egress = bytearray()

    def deliver(self, data: bytes) -> None:
        self.inbound.deliver(data)

    def drain_egress(self) -> bytes:
        data = bytes(self.egress)
        self.egress.clear()
        return data


class Product(ParticipantHooks):
    """A reader that yields per item (as the lockstep reader does per packet), a worker
    and a listener woken by requests, and four sleepers."""

    #: name -> sleep (s), from the admission tick at 20 ms: due 0.11, 0.09, 0.09, 0.15 s.
    SLEEPERS = (("late", 0.09), ("a", 0.07), ("b", 0.07), ("after", 0.13))

    def __init__(self, baton: BatonRuntime) -> None:
        self._baton = baton
        self.link = Link(baton)
        self.links = {DEVICE: self.link}
        self.kicks = BatonQueue(baton)
        self.calls = BatonQueue(baton)
        self.trace: list[tuple] = []
        self.out = OutboxBuffer()
        self.ended = False

    def now_ms(self) -> int:
        return self._baton.now_us // 1000

    def construct(self) -> None:
        self._baton.spawn(self.reader, name="reader")
        self._baton.spawn(self.worker, name="worker")
        self._baton.spawn(self.listener, name="listener")
        for name, seconds in self.SLEEPERS:
            self._baton.spawn(lambda name=name, seconds=seconds: self.sleeper(name, seconds), name=name)

    def reader(self) -> None:
        while True:
            data = self.link.inbound.get()
            self.trace.append(("read", data, self.now_ms()))
            self.link.egress += data
            yield_now(self._baton)

    def worker(self) -> None:
        while True:
            self.kicks.get()
            for step in range(2):
                self.trace.append(("work", step, self.now_ms()))
                yield_now(self._baton)

    def listener(self) -> None:
        while True:
            self.calls.get()
            self.trace.append(("call", self.now_ms()))

    def sleeper(self, name: str, seconds: float) -> None:
        sleep(self._baton, seconds)
        self.trace.append(("timer", name, self.now_ms()))
        self.out.log_lines.append(f"timer {name}")

    def on_request(self, request) -> None:
        (self.kicks if request.payload == "kick" else self.calls).deliver(request.payload)

    def on_end(self) -> None:
        self.ended = True

    def finished(self) -> bool:
        return self.ended

    def flushes(self):
        return [lambda: self.trace.append(("flush", self.now_ms()))]

    def drain_outbox(self) -> Outbox:
        return self.out.take()


class SleepyFlush(Product):
    """A flush that waits on its own timer (as a timed join does) before it is done."""

    def flushes(self):
        def flush() -> None:
            sleep(self._baton, 0)
            self.trace.append(("flush", self.now_ms()))
        return [flush]


def _loop(caps: dict[int, int], product_type: type[Product] = Product) -> tuple[ParticipantLoop, Product]:
    """``caps``: tick -> its handoff cap; every other tick has none."""
    baton = BatonRuntime()
    participant = Participant(
        LockstepRuntime.create(baton, epoch_us=0, seed=1, ordinal=1), LockstepIdentity(1, 1), ())
    product = product_type(baton)
    loop = ParticipantLoop(participant, product, link=None,
                           handoffs=lambda t_us: caps.get(t_us // TICK_US))
    return loop, product


def _tick(k: int, data: bytes = b"", requests: tuple = (), end: bool = False) -> Tick:
    return Tick(k, k * TICK_US, ((DEVICE, data),) if data else (), requests, end=end)


def test_a_freeze_holds_runnable_work_queues_inputs_and_fires_overdue_timers_on_resume():
    # Tick 3 has a CPU budget of one handoff, so work is still runnable when the
    # freeze (ticks 4-6) starts; sleepers a and b come due at 0.09 s, late at 0.11 s.
    loop, product = _loop({3: 1, 4: 0, 5: 0, 6: 0})
    dones = [loop.step(_tick(1)), loop.step(_tick(2)),
             loop.step(_tick(3, b"a", (Request((0, 1), "kick"),))),
             loop.step(_tick(4, b"b")),
             loop.step(_tick(5, requests=(Request((0, 2), "call"),))),
             loop.step(_tick(6))]
    assert product.trace == [("work", 0, 60)]  # the budget's one handoff, then nothing ran
    assert [(d.ports, d.log_lines) for d in dones[3:]] == [(((DEVICE, b""),), ())] * 3
    assert product.link.inbound.qsize() == 2 and product.calls.qsize() == 1  # inputs queued

    resumed = loop.step(_tick(7, b"d"))
    after = loop.step(_tick(8))
    assert product.trace == [
        ("work", 0, 60),
        # Work already runnable first, first in first out, then the work a frozen input woke.
        ("read", b"a", 140), ("work", 1, 140), ("call", 140), ("read", b"b", 140), ("read", b"d", 140),
        # Then every timer that came due, in (due, park seq) order, deadlines unchanged.
        ("timer", "a", 140), ("timer", "b", 140), ("timer", "late", 140),
        ("timer", "after", 160),
    ]
    assert resumed.ports == ((DEVICE, b"abd"),)
    assert resumed.log_lines == ("timer a", "timer b", "timer late") and after.log_lines == ("timer after",)


def test_the_end_shares_the_ticks_cap_and_waits_out_a_freeze():
    loop, product = _loop({3: 0, 4: 0})
    loop.step(_tick(1))
    loop.step(_tick(2))
    frozen = [loop.step(_tick(3, end=True)), loop.step(_tick(4))]
    assert [d.ended for d in frozen] == [None, None] and ("flush", 60) not in product.trace
    done = loop.step(_tick(5))
    # The end is work already runnable: it runs first on resume, then the due timers.
    assert done.ended is True and product.trace[-3:] == [("flush", 100), ("timer", "a", 100), ("timer", "b", 100)]


def test_the_ends_own_timers_share_what_is_left_of_the_cap():
    loop, product = _loop({3: 1}, SleepyFlush)
    loop.step(_tick(1))
    loop.step(_tick(2))
    # The tick's one handoff starts the flush; its timer, due now, waits for the next tick.
    assert loop.step(_tick(3, end=True)).ended is None and product.trace == []
    assert loop.step(_tick(4)).ended is True and product.trace == [("flush", 80)]


def test_a_budget_shared_by_inputs_and_timers_leaves_the_rest_for_the_next_tick():
    # Tick 5: two handoffs for the reader's input (it yields after each item),
    # one for the first due timer; b, popped with a, stays runnable.
    loop, product = _loop({5: 3})
    for k in range(1, 5):
        loop.step(_tick(k))
    loop.step(_tick(5, b"x"))
    assert product.trace == [("read", b"x", 100), ("timer", "a", 100)]
    loop.step(_tick(6))
    assert product.trace[2:] == [("timer", "b", 120), ("timer", "late", 120)]
