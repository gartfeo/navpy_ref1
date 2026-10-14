"""Cancellation scopes: every baton wait is a checkpoint; a cancel ends its unit only.

``lockstep_slice.md``, "Backend participant", Cancel: a scope is a context
variable that managed threads and executor jobs inherit; a cancel wakes a
parked waiter, which undoes its own state (a handed-over lock or permit is
passed on) and raises ``Cancelled``; shielded waits complete; a consumed
``Cancelled`` retires its thread or cancels its future, anything else is a
fault.
"""

from __future__ import annotations

import asyncio
import contextvars

import pytest

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import (
    BatonCondition,
    BatonEvent,
    BatonLock,
    BatonSemaphore,
    join,
    sleep,
)
from navpy.sim.lockstep.cancel_scope import CancelScope, Cancelled

TICK_US = 20_000


def _in_scope(scope: CancelScope, body):
    def run() -> None:
        with cancel_scope.entered(scope):
            body()
    return run


def _tick(runtime: BatonRuntime, k: int, deliver=None) -> None:
    runtime.tick(k * TICK_US, deliver)


def test_a_cancelled_lock_waiter_leaves_and_the_lock_serves_the_next_thread():
    runtime = BatonRuntime()
    lock, log = BatonLock(runtime), []
    scope = CancelScope("request 0.1")

    def holder() -> None:
        lock.acquire()
        sleep(runtime, 0.05)
        lock.release()

    def cancelled() -> None:
        lock.acquire()
        log.append("cancelled waiter got the lock")  # never: a cancelled scope does not return

    def later() -> None:
        sleep(runtime, 0.03)
        with lock:
            log.append(("later", runtime.now_us))

    runtime.spawn(holder, name="holder")
    runtime.spawn(_in_scope(scope, cancelled), name="cancelled")
    runtime.spawn(later, name="later")
    runtime.run_until_quiescent()
    _tick(runtime, 1, lambda: scope.cancel())
    for k in range(2, 5):
        _tick(runtime, k)
    assert log == [("later", 60_000)]  # the holder's release went straight to "later"
    assert runtime.threads.alive() == [] and not runtime.threads.errors


def test_a_cancel_racing_a_handoff_passes_the_lock_on_and_makes_no_progress():
    """Release hands the lock to A; A's cancel lands in the same tick before A runs."""
    runtime = BatonRuntime()
    lock, progress = BatonLock(runtime), []
    scope = CancelScope("upload")
    released = BatonEvent(runtime)
    handed = []

    def holder() -> None:
        lock.acquire()
        released.wait()
        lock.release()  # hands to A (the first waiter); A has not run yet
        handed.append(lock._owner.name)
        scope.cancel()  # the cancel lands between the handoff and A's run

    def waiter(name: str):
        def body() -> None:
            with lock:
                progress.append((name, runtime.now_us))
        return body

    runtime.spawn(holder, name="holder")
    runtime.spawn(_in_scope(scope, waiter("A")), name="A")
    runtime.spawn(waiter("B"), name="B")
    runtime.run_until_quiescent()
    _tick(runtime, 1, released.deliver_set)
    assert handed == ["A"]
    assert progress == [("B", TICK_US)]  # A passed it on and made no progress
    assert not lock.locked() and runtime.threads.alive() == [] and not runtime.threads.errors


def test_a_wait_in_a_cancelled_scope_raises_before_parking():
    runtime = BatonRuntime()
    scope = CancelScope("probe")
    scope.cancel()
    seen = []

    def body() -> None:
        try:
            sleep(runtime, 1.0)
        except Cancelled as error:
            seen.append(error.scope is scope)
            raise

    runtime.spawn(_in_scope(scope, body), name="probe")
    runtime.run_until_quiescent()
    assert seen == [True] and runtime.threads.alive() == [] and not runtime.threads.errors


@pytest.mark.parametrize("primitive", ["event", "queue", "semaphore", "join", "sleep", "condition"])
def test_every_wait_is_a_checkpoint_and_undoes_its_waiter_state(primitive):
    runtime = BatonRuntime()
    scope = CancelScope("job")
    event, queue, semaphore = BatonEvent(runtime), BatonQueue(runtime), BatonSemaphore(runtime, 0)
    condition = BatonCondition(runtime)
    blocker = runtime.spawn(lambda: event.wait(), name="blocker")

    def condition_wait() -> None:
        with condition:
            condition.wait()

    waits = {
        "event": lambda: event.wait(),
        "queue": lambda: queue.get(),
        "semaphore": lambda: semaphore.acquire(),
        "join": lambda: join(runtime, blocker),
        "sleep": lambda: sleep(runtime, 5.0),
        "condition": condition_wait,
    }
    after = []

    def body() -> None:
        waits[primitive]()
        after.append("returned")

    runtime.spawn(_in_scope(scope, body), name="cancelled")
    runtime.run_until_quiescent()
    _tick(runtime, 1, lambda: scope.cancel())
    assert after == []
    assert [t.name for t in runtime.threads.alive()] == ["blocker"]
    # The primitive is clean: another thread uses it normally.
    used = []

    def other() -> None:
        if primitive == "queue":
            queue.put(1)
            used.append(queue.get(timeout=0.1))
        elif primitive == "semaphore":
            semaphore.release()
            used.append(semaphore.acquire(timeout=0.1))
        elif primitive == "condition":
            with condition:  # the cancelled waiter reacquired (shielded), then left
                condition.notify_all()
                used.append(True)
        else:
            used.append(True)
        event.set()

    runtime.spawn(other, name="other")
    _tick(runtime, 2)
    assert used and runtime.threads.alive() == [] and not runtime.threads.errors


def test_a_permit_handed_to_a_cancelled_waiter_is_passed_on():
    runtime = BatonRuntime()
    semaphore, scope, got = BatonSemaphore(runtime, 0), CancelScope("a"), []
    release = BatonEvent(runtime)

    def releaser() -> None:
        release.wait()
        semaphore.release()

    runtime.spawn(releaser, name="releaser")
    runtime.spawn(_in_scope(scope, lambda: semaphore.acquire()), name="a")
    runtime.spawn(lambda: got.append(semaphore.acquire()), name="b")
    runtime.run_until_quiescent()
    _tick(runtime, 1, lambda: (release.deliver_set(), scope.cancel()))
    assert got == [True] and runtime.threads.alive() == []


def test_shielded_waits_complete_in_a_cancelled_scope():
    runtime = BatonRuntime()
    scope, log = CancelScope("rollback"), []

    def body() -> None:
        try:
            sleep(runtime, 1.0)
        finally:
            with cancel_scope.shielded():
                sleep(runtime, 0.04)  # cleanup still parks and returns
                log.append(runtime.now_us)

    runtime.spawn(_in_scope(scope, body), name="unit")
    runtime.run_until_quiescent()
    _tick(runtime, 1, lambda: scope.cancel())
    for k in range(2, 5):
        _tick(runtime, k)
    assert log == [60_000] and runtime.threads.alive() == [] and not runtime.threads.errors


def test_children_inherit_the_scope_and_an_unconsumed_cancel_is_a_fault():
    runtime = BatonRuntime()
    scope, children = CancelScope("route"), []

    def parent() -> None:
        children.append(cancel_scope.current())
        runtime.spawn(lambda: children.append(cancel_scope.current()), name="child")

    runtime.spawn(_in_scope(scope, parent), name="parent")
    runtime.run_until_quiescent()
    assert children == [scope, scope]

    stray = CancelScope("nobody cancelled it")

    def bad() -> None:
        raise Cancelled(stray)

    runtime.spawn(bad, name="bad")
    with pytest.raises(Cancelled):
        runtime.run_until_quiescent()


def test_a_cancelled_executor_job_ends_its_future_cancelled_not_failed():
    runtime = BatonRuntime()
    scope, box = CancelScope("request 0.4"), {}

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        box["loop"] = loop

        async def main() -> None:
            future = loop.run_in_executor(None, sleep, runtime, 5.0)
            box["future"] = future
            try:
                await future
            except asyncio.CancelledError:
                box["outcome"] = "cancelled"
            await loop.shutdown_default_executor()

        context = cancel_scope.bind(contextvars.copy_context(), scope)
        try:
            loop.run_until_complete(loop.create_task(main(), context=context))
        finally:
            loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    _tick(runtime, 1)
    _tick(runtime, 2, lambda: scope.cancel())
    assert box["outcome"] == "cancelled" and box["future"].cancelled()
    assert runtime.threads.alive() == [] and not runtime.threads.errors


def test_a_closed_scope_ignores_a_late_cancel():
    scope = CancelScope("answered")
    scope.close()
    assert scope.cancel() is False and not scope.cancelled


def test_the_wall_runtime_shield_is_a_no_op():
    with rt.wall_runtime().threads.shielded() as shield:
        assert shield is None


@pytest.mark.parametrize("primitive", [
    "free lock", "set event", "free permit", "queue item", "queue room", "finished join",
    "sleep 0", "wait_for true", "wait_for expired",
])
def test_a_wait_that_would_return_at_once_takes_nothing_in_a_cancelled_scope(primitive):
    runtime = BatonRuntime()
    scope, raised, took = CancelScope("job"), [], []
    lock, event, semaphore = BatonLock(runtime), BatonEvent(runtime), BatonSemaphore(runtime, 1)
    queue, condition = BatonQueue(runtime), BatonCondition(runtime)
    finished = runtime.spawn(lambda: None, name="finished")

    def wait_for(predicate, timeout=None):
        with cancel_scope.shielded():
            condition.__enter__()  # held before the wait, as a caller's ``with`` holds it
        try:
            return condition.wait_for(predicate, timeout)
        finally:
            condition.__exit__()
    runtime.run_until_quiescent()

    def ready() -> None:
        event.set()
        queue.put("item")

    runtime.spawn(ready, name="ready")
    runtime.run_until_quiescent()
    waits = {
        "free lock": lambda: lock.acquire(),
        "set event": lambda: event.wait(),
        "free permit": lambda: semaphore.acquire(),
        "queue item": lambda: queue.get(),
        "queue room": lambda: queue.put("more"),
        "finished join": lambda: join(runtime, finished),
        "sleep 0": lambda: sleep(runtime, 0),
        "wait_for true": lambda: wait_for(lambda: True),
        "wait_for expired": lambda: wait_for(lambda: False, 0),
    }
    scope.cancel()

    def body() -> None:
        try:
            took.append(waits[primitive]())
        except Cancelled:
            raised.append(primitive)
            raise

    runtime.spawn(_in_scope(scope, body), name="cancelled")
    runtime.run_until_quiescent()
    assert raised == [primitive] and took == []
    assert not lock.locked() and semaphore._value == 1 and queue.qsize() == 1
    assert not condition._lock.locked()
    assert runtime.threads.alive() == [] and not runtime.threads.errors


def test_an_already_cancelled_job_cannot_take_a_free_mission_lock_or_submit_progress():
    """A request cancelled before its executor job ran: the job's first wait
    (the free mission lock) raises, so no progress is made or broadcast."""
    runtime = BatonRuntime()
    scope, box, progress = CancelScope("request 0.7"), {}, []
    mission_lock = BatonLock(runtime, reentrant=True)

    def download() -> str:
        with mission_lock:
            progress.append((1, 4))
            return "downloaded"

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)

        async def main() -> None:
            scope.cancel()
            try:
                box["result"] = await loop.run_in_executor(None, download)
            except asyncio.CancelledError:
                box["result"] = "cancelled"
            await loop.shutdown_default_executor()

        context = cancel_scope.bind(contextvars.copy_context(), scope)
        try:
            loop.run_until_complete(context.run(loop.create_task, main()))
        finally:
            loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    assert box["result"] == "cancelled" and progress == [] and not mission_lock.locked()
    assert runtime.threads.alive() == [] and not runtime.threads.errors
