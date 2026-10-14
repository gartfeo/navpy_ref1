"""The backend product inside its participant: built at the admission tick.

Construction (the managed bootstrap thread, under the baton):

- the run inputs: the settings document is written run-locally and the
  settings store points at it; logs go to the run-relative ``gcs`` directory
  (so the startup log line names a run-relative path);
- AnyIO's worker threads (sync routes, ``run_in_threadpool``) are bound to
  the loop's ``BatonExecutor``;
- the module-level vehicle manager opens ``lockstep:serial1`` through the
  link (``VehicleManager.set_bus_opener``); the ESP32 launch controller is
  the in-process port (``esp32_port``);
- every broadcast is journaled by the event sink; the producer inventory
  runs each background task and thread in its own cancellation scope.

The loop thread (``BatonEventLoop``: ``loop.time()`` is the runtime clock,
the default executor is the ``BatonExecutor``) runs lifespan startup, then
reports readiness (``runtime_ready``, in that tick's ``DONE``); requests run
until the drain ``end``, then the strict shutdown, and the loop closes.
"""

from __future__ import annotations

import asyncio
import contextvars
import json
import os
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any

import anyio.to_thread

from gcs.backend.lockstep.log_capture import capture_backend_logs

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.cancel_scope import CancelScope
from navpy.sim.lockstep.coordinator_tick import READY_EVENT
from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.part_wire import Event, Lifecycle, Request, RequestId
from navpy.sim.lockstep.participant_hooks import OutboxBuffer, ParticipantContext

if TYPE_CHECKING:
    from gcs.backend.lockstep.launcher_port import LockstepLauncher

#: Run-relative paths of the backend's run inputs and logs.
SETTINGS_FILE = "gcs_settings.json"
LOG_DIR = "gcs"


def configure_run_inputs(config: Mapping[str, Any]) -> None:
    """The settings document and the log directory, before the product imports."""
    settings = config.get("settings")
    if not isinstance(settings, dict):
        raise ValueError("the backend role config needs a settings document")
    Path(SETTINGS_FILE).write_text(json.dumps(settings, sort_keys=True), encoding="utf-8")
    os.environ["GCS_SETTINGS_PATH"] = SETTINGS_FILE
    os.environ["GCS_LOG_DIR"] = LOG_DIR


async def _run_sync_on_executor(func: Callable[..., Any], *args: Any, **_options: Any) -> Any:
    """``anyio.to_thread.run_sync`` on the loop's managed executor.

    The job runs in a copy of the calling task's context, as AnyIO's does.
    """
    return await asyncio.get_running_loop().run_in_executor(None, lambda: func(*args))


def bind_anyio_to_executor() -> None:
    anyio.to_thread.run_sync = _run_sync_on_executor


class BackendHost:
    def __init__(self, context: ParticipantContext, link: LockstepLink, out: OutboxBuffer) -> None:
        self._participant = context.participant
        self._baton = context.participant.runtime.baton
        self._out = out
        configure_run_inputs(context.config)
        bind_anyio_to_executor()
        from gcs.backend import main
        from gcs.backend.broadcast import manager as ws_manager
        from gcs.backend.esp32_access import esp32
        from gcs.backend.lockstep.esp32_port import InProcessEsp32
        from gcs.backend.lockstep.event_sink import EventSink
        from gcs.backend.lockstep.faults import FaultLatch
        from gcs.backend.lockstep.scopes import LockstepProducerScopes
        from gcs.backend.producer_inventory import producers

        self._main = main
        capture_backend_logs(out, Path.cwd())  # the host dir; after main's ``basicConfig``
        self._link = link
        main.vehicle_mgr.set_bus_opener(link.open)
        esp32.port = InProcessEsp32()
        self._launcher = self._launcher_port(context, out)
        self.sink = EventSink(out)
        ws_manager.observer = self.sink.observe
        self.faults = FaultLatch(out)
        self.scopes = LockstepProducerScopes(self.faults)
        producers.scopes = self.scopes
        self._inventory = producers
        self._loop: BatonEventLoop | None = None
        self._runner = None
        self._end: asyncio.Event | None = None
        self.finished = False

    @staticmethod
    def _launcher_port(context: ParticipantContext, out: OutboxBuffer) -> LockstepLauncher:
        """Always the launcher port: with no configured companions it refuses
        every start. The subprocess manager is refused outright (fail closed)."""
        from gcs.backend import navpy_sim_runtime
        from gcs.backend.lockstep.launcher_port import LockstepLauncher
        from gcs.backend.navpy_process_manager import NavpyProcessManager

        NavpyProcessManager.refused = "lockstep companions are coordinator participants"
        launcher = LockstepLauncher(context.config.get("companions") or {}, out, context.ids)
        navpy_sim_runtime.set_manager_factory(lambda: launcher)
        return launcher

    def start(self) -> None:
        rt.Thread(target=self._run_loop, name="backend-loop").start()

    # -- coordinator side (the tick's input phase) -------------------------------
    def lifecycle(self, entry: Lifecycle) -> None:
        self._launcher.deliver(entry)

    def cancel(self, request_id: RequestId) -> None:
        self._runner.cancel(request_id)

    def request(self, request: Request) -> None:
        self._loop.deliver(lambda: self._runner.start(request))

    def end(self) -> None:
        self._loop.deliver(self._end.set)

    # -- the loop thread -----------------------------------------------------------
    def _run_loop(self) -> None:
        from gcs.backend.lockstep.requests import RequestRunner

        loop = BatonEventLoop(self._baton)
        loop.job_failures.report = self.faults.job_failure
        self._loop = loop
        self._runner = RequestRunner(loop, self._main.app, self.sink, self._out, self._inventory)
        owner = CancelScope("shutdown owner")
        try:
            context = cancel_scope.bind(contextvars.copy_context(), owner)
            loop.run_until_complete(context.run(loop.create_task, self._lifetime(owner)))
        finally:
            loop.close()
        self.finished = True

    async def _lifetime(self, owner: CancelScope) -> None:
        from gcs.backend.lockstep.shutdown import ShutdownParts, strict_shutdown

        self._end = asyncio.Event()
        await self._main.startup()
        self._out.events.append(Event(READY_EVENT, {"role": "backend", **self._participant.identity()}))
        await self._end.wait()
        await strict_shutdown(ShutdownParts(
            self._loop, self._baton, self._inventory, self.scopes, self._runner, self.sink,
            (self._link,), self.faults,
        ), owner)


__all__ = ["BackendHost", "LOG_DIR", "SETTINGS_FILE", "bind_anyio_to_executor", "configure_run_inputs"]
