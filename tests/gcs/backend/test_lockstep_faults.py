"""Unobserved failures latch a participant fault (``lockstep_slice.md``, "Cancel").

A request or producer task, or a retired executor job, that ends with an
unexpected exception or with a ``Cancelled`` of a scope nobody cancelled is
not an outcome: the fault lands in the ``DONE``'s ``faults`` even when the
raise is swallowed. So does a producer's executor submission after the event
sink closed.
"""

from __future__ import annotations

import asyncio
import gc
import logging
import re

import pytest

from gcs.backend.lockstep.event_sink import EventSink
from gcs.backend.lockstep.faults import FaultLatch
from gcs.backend.lockstep.log_capture import capture_backend_logs
from gcs.backend.lockstep.requests import RequestRunner
from gcs.backend.lockstep.scopes import LockstepProducerScopes
from gcs.backend.lockstep.shutdown import ShutdownFault, _observed, _owner_only
from gcs.backend.producer_inventory import ProducerInventory
from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.baton_sync import sleep
from navpy.sim.lockstep.cancel_scope import CancelScope, Cancelled
from navpy.sim.lockstep.part_wire import Request
from navpy.sim.lockstep.participant_hooks import OutboxBuffer

REQUEST = Request((0, 1), {"kind": "http", "method": "GET", "target": "/x"})


#: An absolute host path in a log line: a drive, or a POSIX root directory.
HOST_PATH = re.compile(r"[A-Za-z]:[\\/]|(^|[\s'\"(])/(home|Users|tmp|usr|var|opt|mnt)/")


def _serve(app, request: Request = REQUEST) -> OutboxBuffer:
    """Start the request on a real loop and never await its task (as the
    backend does not); collect garbage after, as an unretrieved task would
    then be reported; every line the backend logged goes to ``log_lines``."""
    runtime, out = BatonRuntime(), OutboxBuffer()
    handler = capture_backend_logs(out)

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        runner = RequestRunner(loop, app, EventSink(out), out, ProducerInventory())

        async def main() -> None:
            runner.start(request)
            while runner._pending:
                await asyncio.sleep(0)
            await loop.shutdown_default_executor()

        try:
            loop.run_until_complete(main())
        finally:
            loop.close()
        gc.collect()

    try:
        runtime.spawn(loop_thread, name="backend-loop")
        runtime.run_until_quiescent()
    finally:
        logging.getLogger().removeHandler(handler)
    assert not [line for line in out.log_lines if "never retrieved" in line]
    assert not [line for line in out.log_lines if HOST_PATH.search(line)]
    return out


def test_a_request_that_raises_unexpectedly_latches_a_fault_and_is_unanswered():
    async def app(scope, receive, send):
        raise RuntimeError("route bug")

    out = _serve(app)
    assert out.faults == ["RuntimeError:request 0.1"] and out.responses == []


def test_a_cancelled_of_a_scope_nobody_cancelled_is_a_fault_not_a_cancel():
    stray = CancelScope("someone else's")

    async def app(scope, receive, send):
        raise Cancelled(stray)

    out = _serve(app)
    assert out.faults == ["Cancelled:request 0.1"] and out.responses == []


def test_an_unknown_request_kind_latches_a_fault():
    out = _serve(None, Request((0, 2), {"kind": "bogus"}))
    assert out.faults == ["ValueError:request 0.2"]


def test_a_producer_task_that_fails_latches_a_fault():
    out = OutboxBuffer()
    scopes = LockstepProducerScopes(FaultLatch(out))

    async def main() -> None:
        async def fails() -> None:
            raise KeyError("lost")

        async def consumed() -> None:
            scope = CancelScope("producer launch_tasks")
            scope.cancel()
            raise Cancelled(scope)

        for name, coroutine in (("autoconnect", fails()), ("launch_tasks", consumed())):
            task = asyncio.get_running_loop().create_task(coroutine)
            task.add_done_callback(lambda t, n=name: scopes.observe(n, t))
            await asyncio.gather(task, return_exceptions=True)

    asyncio.run(main())
    assert out.faults == ["KeyError:producer autoconnect"]


def test_a_retired_job_that_failed_is_a_fault_and_a_consumed_cancel_is_not():
    out = OutboxBuffer()

    class Parts:
        faults = FaultLatch(out)

    cancelled = CancelScope("request 0.3")
    cancelled.cancel()
    _observed(Parts, [None, Cancelled(cancelled), asyncio.CancelledError()])
    assert out.faults == []
    with pytest.raises(ShutdownFault):
        _observed(Parts, [OSError("disk")])
    assert out.faults == ["OSError:retired work"]


def test_a_producer_submission_after_the_sink_closed_is_latched_even_if_swallowed():
    out, owner = OutboxBuffer(), CancelScope("shutdown owner")
    faults = FaultLatch(out)
    with cancel_scope.entered(CancelScope("producer telemetry_loop")):
        try:
            _owner_only(owner, faults)
        except ShutdownFault:
            pass  # a producer's ``except Exception`` swallows the raise
    with cancel_scope.entered(owner):
        _owner_only(owner, faults)  # the owner's own submissions pass
    assert out.faults == ["ShutdownFault:executor after the sink closed"]


def test_a_logged_traceback_reaches_log_lines_without_host_paths():
    out = OutboxBuffer()
    handler = capture_backend_logs(out)
    try:
        try:
            raise OSError("disk full")
        except OSError:
            logging.getLogger("gcs.backend.test").exception("write failed")
    finally:
        logging.getLogger().removeHandler(handler)
    [line] = out.log_lines
    assert line.endswith("write failed\nOSError: disk full") and not HOST_PATH.search(line)


def _on_loop(runtime: BatonRuntime, main, ticks: int = 0) -> None:
    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        try:
            loop.run_until_complete(main(loop))
        finally:
            loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    for k in range(1, ticks + 1):
        runtime.tick(k * 20_000)
    assert runtime.threads.alive() == []


def test_a_broadcast_that_fails_latches_when_it_completes_not_at_retirement():
    runtime, out = BatonRuntime(), OutboxBuffer()
    inventory = ProducerInventory()
    inventory.scopes = LockstepProducerScopes(FaultLatch(out))
    seen = {}

    async def failing_broadcast() -> None:
        raise RuntimeError("send failed")

    async def main(loop) -> None:
        future = inventory.broadcast("route_broadcasts", failing_broadcast(), loop)
        while not future.done():
            await asyncio.sleep(0)
        seen["done"], seen["faults"] = future.done(), list(out.faults)
        assert inventory.futures() == []  # retirement would no longer see it

    _on_loop(runtime, main)
    assert seen == {"done": True, "faults": ["RuntimeError:producer route_broadcasts"]}


def test_an_executor_job_that_fails_after_its_awaiter_left_latches_when_it_completes():
    runtime, out = BatonRuntime(), OutboxBuffer()
    faults = FaultLatch(out)
    seen = {}

    def job() -> None:
        sleep(runtime, 0.05)
        raise OSError("disk")

    def observed_job() -> None:
        raise OSError("handled by its route")

    async def main(loop) -> None:
        loop.job_failures.report = faults.job_failure
        with pytest.raises(OSError):  # an awaited failure is its awaiter's: no fault
            await loop.run_in_executor(None, observed_job)
        waiter = asyncio.ensure_future(loop.run_in_executor(None, job))
        await asyncio.sleep(0.02)  # the job has started (a pending one would just be cancelled)
        waiter.cancel()  # the awaiting request is gone; the job runs on
        await asyncio.sleep(0.1)
        seen["faults"] = list(out.faults)  # before any shutdown or retirement

    _on_loop(runtime, main, ticks=8)
    assert seen == {"faults": ["OSError:executor job"]}


def test_a_failure_arriving_as_the_deadline_cancels_its_awaiter_is_a_fault():
    """Codex's case: the job fails at 0.049 s, the request's deadline is 0.05 s,
    ticks are 0.02 s. Both land at t = 0.06 s, in one loop iteration: the
    failure is copied to the job's future (not cancelled), then the deadline
    cancels the awaiting task, which wakes with a cancel and never takes it."""
    runtime, out = BatonRuntime(), OutboxBuffer()
    faults = FaultLatch(out)

    def job() -> None:
        sleep(runtime, 0.049)
        raise OSError("disk")

    async def app(scope, receive, send):
        await asyncio.get_running_loop().run_in_executor(None, job)

    request = Request((0, 9), {"kind": "http", "method": "GET", "target": "/x", "deadline_s": 0.05})

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)
        loop.job_failures.report = faults.job_failure
        runner = RequestRunner(loop, app, EventSink(out), out, ProducerInventory())

        async def main() -> None:
            runner.start(request)
            while runner._pending:
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.02)  # the decision runs two iterations after the arrival

        try:
            loop.run_until_complete(main())
        finally:
            loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    for k in range(1, 8):
        runtime.tick(k * 20_000)
    assert [r.payload for r in out.responses] == [{"deadline": True}]
    assert out.faults == ["OSError:executor job"]
    assert runtime.threads.alive() == [] and not runtime.threads.errors


def test_log_lines_carry_no_run_root_or_other_absolute_path(tmp_path):
    """The run's directories become ``<run>`` and ``<root>`` (with a POSIX
    remainder), any other absolute path ``<abs>``; a URL route stays."""
    run_dir = tmp_path / "run" / "host"
    run_dir.mkdir(parents=True)
    out = OutboxBuffer()
    handler = capture_backend_logs(out, run_dir)
    log = logging.getLogger("gcs.backend.main")
    try:
        log.warning("Could not set up backend file logging: [Errno 13] Permission denied: %r",
                    str(run_dir / "gcs" / "backend.log"))
        log.warning("settings at %s, root %s", (run_dir / "gcs_settings.json").as_posix(), run_dir.parent)
        log.warning(r"calibration D:\data\board\cal.npz missing; UNC \\server\share\f.txt; POST /api/vehicles/upload")
    finally:
        logging.getLogger().removeHandler(handler)
    first, second, third = out.log_lines
    assert first.endswith("Permission denied: '<run>/gcs/backend.log'")
    assert second.endswith("settings at <run>/gcs_settings.json, root <root>")
    assert third.endswith("calibration <abs> missing; UNC <abs>; POST /api/vehicles/upload")
    for line in out.log_lines:
        assert str(tmp_path) not in line and tmp_path.as_posix() not in line and not HOST_PATH.search(line)
