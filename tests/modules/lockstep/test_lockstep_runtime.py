"""The LockstepRuntime facets: clock, primitives, managed threads, entropy."""

from __future__ import annotations

import hashlib
import uuid
from datetime import datetime, timedelta, timezone
from queue import Empty, Full

import pytest

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import BatonRuntime, LockstepFault
from navpy.sim.lockstep.lockstep_runtime import (
    ENTROPY_DOMAIN,
    LockstepRuntime,
    module_port,
    participant_seed,
)

TICK_US = 20_000
EPOCH_US = 1_760_000_000_123_456  # a run input: 2025-10-09T08:53:20.123456Z


def make(epoch_us: int = EPOCH_US, seed: int = 7, ordinal: int = 1) -> LockstepRuntime:
    return LockstepRuntime.create(BatonRuntime(), epoch_us=epoch_us, seed=seed, ordinal=ordinal)


def run(runtime: LockstepRuntime, ticks: int) -> None:
    """Tick 0 starts every thread; then ``ticks`` ticks; then unwind what is parked."""
    baton = runtime.baton
    baton.run_until_quiescent()
    for k in range(1, ticks + 1):
        baton.tick(k * TICK_US)
    baton.abort()


def spawn(runtime: LockstepRuntime, body) -> None:
    runtime.baton.spawn(body, name=getattr(body, "__name__", "body"))


def tick_of(runtime: LockstepRuntime) -> int:
    return runtime.baton.now_us // TICK_US


# -- clock ----------------------------------------------------------------------

def test_the_clock_is_t_k_and_the_run_epoch():
    runtime = make()
    clock = runtime.clock
    runtime.baton.tick(3 * TICK_US)
    assert clock.monotonic() == clock.perf_counter() == 0.06
    assert clock.time() == (EPOCH_US + 60_000) / 1_000_000
    utc = datetime(1970, 1, 1, tzinfo=timezone.utc) + timedelta(microseconds=EPOCH_US + 60_000)
    assert clock.now(timezone.utc) == utc
    assert clock.now(timezone.utc).microsecond == 183_456
    assert clock.now() == utc.replace(tzinfo=None)  # naive UTC: no host time zone
    plus_four = timezone(timedelta(hours=4))
    assert clock.now(plus_four) == utc and clock.now(plus_four).utcoffset() == timedelta(hours=4)
    assert clock.since_start_us() == 60_000


def test_sleep_parks_to_the_first_tick_at_or_after_its_deadline():
    runtime = make()
    woke = []

    def sleeper() -> None:
        runtime.clock.sleep(0.04)  # exactly on tick 2
        woke.append(tick_of(runtime))
        runtime.clock.sleep(0.025)  # between ticks 3 and 4
        woke.append(tick_of(runtime))
        runtime.clock.sleep(-1.0)  # a negative wait is a zero wait
        woke.append(tick_of(runtime))

    spawn(runtime, sleeper)
    run(runtime, 5)
    assert woke == [2, 4, 4]


# -- primitives -------------------------------------------------------------------

def test_lock_rlock_condition_and_event_come_from_the_baton():
    runtime = make()
    threads = runtime.threads
    seen = []

    def body() -> None:
        lock, rlock = threads.Lock(), threads.RLock()
        with rlock, rlock:
            seen.append("reentrant")
        assert lock.acquire() and lock.locked()
        lock.release()
        condition = threads.Condition()  # default lock is reentrant, as threading's
        with condition, condition:
            seen.append(("condition", condition.wait(0.04), tick_of(runtime)))
        event = threads.Event()
        seen.append(("event", event.wait(0.02), tick_of(runtime)))

    spawn(runtime, body)
    run(runtime, 4)
    assert seen == ["reentrant", ("condition", False, 2), ("event", False, 3)]


def test_a_semaphore_hands_permits_over_first_in_first_out():
    runtime = make()
    semaphore = runtime.threads.Semaphore(2)
    order = []

    def worker(name: str, hold_s: float) -> None:
        with semaphore:
            order.append((name, tick_of(runtime)))
            runtime.clock.sleep(hold_s)

    for name, hold in (("a", 0.02), ("b", 0.04), ("c", 0.0), ("d", 0.0)):
        runtime.baton.spawn(lambda n=name, h=hold: worker(n, h), name=name)
    run(runtime, 3)
    assert order == [("a", 0), ("b", 0), ("c", 1), ("d", 1)]


def test_semaphore_timeouts_and_misuse_follow_threading():
    runtime = make()
    bounded = runtime.threads.BoundedSemaphore(1)
    outcome = {}

    def body() -> None:
        assert bounded.acquire()
        outcome["nonblocking"] = bounded.acquire(blocking=False)
        outcome["timed"] = (bounded.acquire(timeout=0.04), tick_of(runtime))  # on tick 2
        with pytest.raises(ValueError, match="non-blocking"):
            bounded.acquire(blocking=False, timeout=1.0)
        bounded.release()
        with pytest.raises(ValueError, match="too many"):
            bounded.release()
        plain = runtime.threads.Semaphore(0)
        plain.release(3)
        outcome["plain"] = [plain.acquire(blocking=False) for _ in range(4)]

    spawn(runtime, body)
    run(runtime, 3)
    assert outcome == {"nonblocking": False, "timed": (False, 2), "plain": [True, True, True, False]}


def test_a_bounded_queue_blocks_put_until_a_get():
    runtime = make()
    q = runtime.threads.Queue(1)
    trace = []

    def producer() -> None:
        q.put("one")
        trace.append(("full", q.full(), q.qsize()))
        with pytest.raises(Full):
            q.put_nowait("x")
        q.put("two")  # parks until the consumer takes "one"
        trace.append(("put two", tick_of(runtime)))

    def consumer() -> None:
        runtime.clock.sleep(0.04)
        trace.append(("got", q.get(), tick_of(runtime)))
        trace.append(("got", q.get(), tick_of(runtime)))
        trace.append(("empty", q.empty()))
        with pytest.raises(Empty):
            q.get_nowait()

    spawn(runtime, producer)
    spawn(runtime, consumer)
    run(runtime, 3)
    # The get wakes the parked put, which runs once the consumer parks again.
    assert trace == [
        ("full", True, 1), ("got", "one", 2), ("put two", 2), ("got", "two", 2), ("empty", True),
    ]


def test_queue_timeouts_land_on_their_tick_and_keep_float_deadlines():
    runtime = make()
    full = runtime.threads.Queue(1)
    empty = runtime.threads.Queue()
    outcome = {}

    def body() -> None:
        full.put("item")
        with pytest.raises(Full):
            full.put("more", timeout=0.04)  # exactly on tick 2
        outcome["put"] = tick_of(runtime)
        with pytest.raises(Empty):
            empty.get(timeout=0.04)
        outcome["get"] = tick_of(runtime)
        with pytest.raises(ValueError, match="non-negative"):
            empty.get(timeout=-1)

    def drift() -> None:
        runtime.clock.sleep(0.2)  # tick 10
        with pytest.raises(Empty):
            runtime.threads.Queue().get(timeout=0.1)  # 0.2 + 0.1 = 0.30000000000000004
        outcome["drift"] = tick_of(runtime)

    spawn(runtime, body)
    spawn(runtime, drift)
    run(runtime, 20)
    assert outcome == {"put": 2, "get": 4, "drift": 16}


def test_queue_join_waits_for_every_task_done():
    runtime = make()
    q = runtime.threads.Queue()
    trace = []

    def worker() -> None:
        while True:
            item = q.get()
            runtime.clock.sleep(0.02)
            trace.append((item, tick_of(runtime)))
            q.task_done()

    def owner() -> None:
        q.put("a")
        q.put("b")
        q.join()
        trace.append(("joined", tick_of(runtime)))
        with pytest.raises(ValueError, match="too many"):
            q.task_done()

    spawn(runtime, worker)
    spawn(runtime, owner)
    run(runtime, 3)
    assert trace == [("a", 1), ("b", 2), ("joined", 2)]


# -- managed threads ------------------------------------------------------------

def test_a_managed_thread_has_the_threading_surface():
    runtime = make()
    threads = runtime.threads
    seen = {}

    def child(value, *, key) -> None:
        me = threads.current_thread()
        seen["child"] = (value, key, me.name, me.ident == threads.get_ident(), me.daemon)
        runtime.clock.sleep(0.06)

    def parent() -> None:
        thread = threads.Thread(target=child, args=(1,), kwargs={"key": "k"}, name="worker", daemon=True)
        seen["before"] = (thread.ident, thread.is_alive())
        with pytest.raises(RuntimeError, match="before it is started"):
            thread.join()
        thread.start()
        with pytest.raises(RuntimeError, match="active thread"):
            thread.daemon = False
        with pytest.raises(RuntimeError, match="only be started once"):
            thread.start()
        thread.join(timeout=0.04)  # times out exactly on tick 2
        seen["timed out"] = (tick_of(runtime), thread.is_alive())
        thread.join()
        seen["joined"] = (tick_of(runtime), thread.is_alive(), thread.ident)
        seen["default name"] = threads.Thread(target=child).name

    spawn(runtime, parent)
    run(runtime, 4)
    assert seen == {
        "before": (None, False),
        "child": (1, "k", "worker", True, True),
        "timed out": (2, True),
        "joined": (3, False, 2),  # ident: the creation sequence (parent is 1)
        "default name": "Thread-1 (child)",
    }


def test_current_thread_is_stable_and_only_on_the_baton():
    runtime = make()
    threads = runtime.threads
    seen = []

    def body() -> None:  # spawned by the runtime directly, not through rt.Thread
        first = threads.current_thread()
        seen.append((first is threads.current_thread(), first.name, first.ident))

    spawn(runtime, body)
    run(runtime, 1)
    assert seen == [(True, "body", 1)]
    with pytest.raises(LockstepFault, match="off the baton"):
        make().threads.current_thread()


def test_a_default_child_inherits_its_creators_daemon_state():
    """threading's rule; reading ``current_thread()`` first must not change it."""
    runtime = make()
    threads = runtime.threads
    seen = {}

    def report(tag: str) -> None:
        seen[tag] = threads.current_thread().daemon

    def body() -> None:  # spawned by the runtime directly: not a daemon
        seen["before handle"] = threads.Thread(target=lambda: None).daemon
        seen["body"] = threads.current_thread().daemon
        seen["after handle"] = threads.Thread(target=lambda: None).daemon
        threads.Thread(target=daemon_parent, daemon=True).start()

    def daemon_parent() -> None:
        threads.Thread(target=report, args=("daemon child",)).start()
        timer = threads.Timer(0.0, lambda: None)
        seen["daemon timer"] = timer.daemon

    spawn(runtime, body)
    run(runtime, 1)
    assert seen == {
        "before handle": False, "body": False, "after handle": False,
        "daemon timer": True, "daemon child": True,
    }


def test_a_timer_fires_at_its_tick_and_a_cancelled_one_never_does():
    runtime = make()
    threads = runtime.threads
    fired = []

    def body() -> None:
        on_tick = threads.Timer(0.04, lambda tag: fired.append((tag, tick_of(runtime))), args=("on",))
        between = threads.Timer(0.05, lambda **kw: fired.append((kw["tag"], tick_of(runtime))),
                                kwargs={"tag": "between"})
        cancelled = threads.Timer(0.02, lambda: fired.append(("cancelled", tick_of(runtime))))
        between.daemon = True
        for timer in (on_tick, between, cancelled):
            timer.start()
        cancelled.cancel()
        on_tick.join()
        fired.append(("alive", on_tick.is_alive(), between.is_alive()))

    spawn(runtime, body)
    run(runtime, 4)
    assert fired == [("on", 2), ("alive", False, True), ("between", 3)]


# -- entropy --------------------------------------------------------------------

def _draws(runtime: LockstepRuntime) -> tuple:
    entropy = runtime.entropy
    return (
        [entropy.rng().random() for _ in range(3)],
        entropy.uuid4(),
        entropy.token_urlsafe(),
        entropy.token_urlsafe(16),
    )


def test_entropy_is_one_stream_per_seed_and_ordinal():
    reference = _draws(make(seed=7, ordinal=1))
    assert _draws(make(seed=7, ordinal=1)) == reference
    assert _draws(make(seed=7, ordinal=2)) != reference
    assert _draws(make(seed=8, ordinal=1)) != reference
    _floats, uid, token, short = reference
    assert uid.version == 4 and uid.variant == uuid.RFC_4122
    assert len(token) == 43 and len(short) == 22  # 32 and 16 bytes, unpadded base64


def test_the_participant_seed_derivation_is_pinned():
    material = f"{ENTROPY_DOMAIN}:7:1".encode("ascii")
    assert participant_seed(7, 1) == int.from_bytes(hashlib.sha256(material).digest(), "big")
    assert ENTROPY_DOMAIN == "navpy.lockstep.entropy.v1"


# -- the module port ------------------------------------------------------------

def test_module_port_serves_every_rt_function():
    port = module_port(make())
    not_served = {
        "ClockPort", "EntropyPort", "LockPort", "QueuePort", "RuntimeBinding", "RuntimeTime",
        "ThreadsPort", "WallClock", "WallEntropy", "WallThreads", "bind", "wall_runtime",
    }
    assert set(vars(port)) == set(rt.__all__) - not_served
    assert port.Empty is Empty
