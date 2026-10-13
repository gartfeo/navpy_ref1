"""The ASGI contract: an asyncio loop and a FastAPI app under the baton."""

from __future__ import annotations

import asyncio
import random
import time as host_time
from contextlib import asynccontextmanager

import pytest
from fastapi import FastAPI

from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop, asgi_request
from navpy.sim.lockstep.baton_sync import sleep

TICK_US = 20_000


def _app(runtime: BatonRuntime, events: list) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_app):
        events.append(("startup", runtime.now_us))
        yield
        events.append(("shutdown", runtime.now_us))

    app = FastAPI(lifespan=lifespan)

    @app.get("/slow")
    async def slow() -> dict:
        await asyncio.sleep(0.05)  # asyncio timer: due at 50 ms -> tick 3
        loop = asyncio.get_running_loop()
        # Blocking work goes to the managed executor, never the loop thread.
        await loop.run_in_executor(None, sleep, runtime, 0.02)
        return {"done_us": runtime.now_us}

    return app


def run(jitter: bool) -> tuple[list, tuple[int, bytes]]:
    runtime = BatonRuntime()
    events: list = []
    result = {}

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        app = _app(runtime, events)

        async def main() -> None:
            async with app.router.lifespan_context(app):
                result["response"] = await asgi_request(app, "GET", "/slow")
                result["tick"] = runtime.now_us // TICK_US

        try:
            loop.run_until_complete(main())
        finally:
            loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    for k in range(1, 20):
        if jitter:
            host_time.sleep(random.random() / 2000)
        runtime.tick(k * TICK_US)
        if "tick" in result:
            break
    runtime.run_until_quiescent()
    return events + [("response_tick", result["tick"])], result["response"]


def test_a_request_spanning_ticks_is_deterministic_with_lifespan_and_executor():
    events, (status, body) = run(jitter=False)
    # sleep(0.05) is due at tick 3 (60 ms); the executor job then sleeps
    # 20 ms, so the response completes at tick 4.
    assert status == 200 and body == b'{"done_us":80000}'
    assert events == [("startup", 0), ("shutdown", 80000), ("response_tick", 4)]
    assert run(jitter=True) == (events, (status, body))


def test_the_coordinator_can_deliver_a_callback_to_the_loop():
    runtime = BatonRuntime()
    loop_box, seen = {}, []

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        loop_box["loop"] = loop
        stop = asyncio.Event()

        async def main() -> None:
            loop_box["stop"] = stop
            await stop.wait()

        loop.run_until_complete(main())
        loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    loop = loop_box["loop"]
    runtime.tick(TICK_US, lambda: loop.deliver(lambda: (seen.append(runtime.now_us), loop_box["stop"].set())))
    assert seen == [TICK_US]
    assert runtime.threads.alive() == []


def _run_loop(runtime: BatonRuntime, main_factory, ticks: int, deliver_at=None):
    box = {}

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        box["loop"] = loop
        try:
            box["result"] = loop.run_until_complete(main_factory(loop))
            loop.run_until_complete(loop.shutdown_default_executor())
        finally:
            loop.close()  # also on abort

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    for k in range(1, ticks + 1):
        callback = (deliver_at or {}).get(k)
        runtime.tick(k * TICK_US, (lambda cb=callback: box["loop"].deliver(cb)) if callback else None)
    return box


def test_an_input_at_an_asyncio_timer_tick_runs_first_and_drift_waits_a_tick():
    runtime = BatonRuntime()
    order = []

    async def main(loop):
        loop.call_later(0.04, lambda: order.append(("timer", runtime.now_us)))
        drift = asyncio.get_running_loop().time() + 0.2 + 0.1  # 0.30000000000000004
        loop.call_at(drift, lambda: order.append(("drift", runtime.now_us)))
        await asyncio.sleep(0.4)

    _run_loop(runtime, main, 20, {2: lambda: order.append(("input", runtime.now_us))})
    assert order == [("input", 40000), ("timer", 40000), ("drift", 320000)]


def test_a_fault_raised_inside_an_asyncio_callback_still_fails_the_tick():
    from navpy.sim.lockstep.baton import LockstepFault
    from navpy.sim.lockstep.baton_sync import BatonLock

    runtime = BatonRuntime()
    lock = BatonLock(runtime)

    def misuse() -> None:
        lock.acquire()
        lock.acquire()  # asyncio logs and swallows this callback's exception

    async def main(loop):
        loop.call_soon(misuse)
        await asyncio.sleep(0.04)

    with pytest.raises(LockstepFault, match="re-acquired"):
        _run_loop(runtime, main, 3)
    runtime.abort()


def test_executor_shutdown_waits_for_jobs_and_refuses_new_ones():
    runtime = BatonRuntime()

    async def main(loop):
        future = loop.run_in_executor(None, sleep, runtime, 0.04)
        await loop.shutdown_default_executor()
        assert future.done()
        with pytest.raises(RuntimeError, match="after shutdown"):
            loop.run_in_executor(None, print)
        return runtime.now_us

    box = _run_loop(runtime, main, 4)
    assert box["result"] == 40000
    assert runtime.threads.alive() == []


def test_shutdown_refuses_a_job_submitted_while_it_waits():
    runtime = BatonRuntime()

    async def main(loop):
        loop.run_in_executor(None, sleep, runtime, 0.04)

        async def late_submit():
            await asyncio.sleep(0.02)
            with pytest.raises(RuntimeError, match="after shutdown"):
                loop.run_in_executor(None, print)
            return "refused"

        late = asyncio.ensure_future(late_submit())
        await loop.shutdown_default_executor()
        return await late

    assert _run_loop(runtime, main, 4)["result"] == "refused"


def test_an_asyncio_zero_timer_loop_is_a_fault_not_a_hang():
    from navpy.sim.lockstep.baton import LockstepFault

    runtime = BatonRuntime()

    async def main(loop):
        def again() -> None:
            loop.call_later(0, again)

        loop.call_later(0.02, again)
        await asyncio.sleep(1.0)

    with pytest.raises(LockstepFault, match="asyncio timers still due"):
        _run_loop(runtime, main, 2)
    runtime.abort()


def test_a_timer_loop_with_ready_callbacks_is_still_bounded():
    from navpy.sim.lockstep.baton import LockstepFault

    runtime = BatonRuntime()

    async def main(loop):
        def again() -> None:
            loop.call_soon(lambda: None)
            loop.call_later(0, again)

        loop.call_later(0.02, again)
        await asyncio.sleep(1.0)

    with pytest.raises(LockstepFault, match="asyncio timers still due"):
        _run_loop(runtime, main, 2)
    runtime.abort()
