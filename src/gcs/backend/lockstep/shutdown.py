"""The backend's strict shutdown in drain (``lockstep_slice.md``, "Shutdown in drain").

It runs on the loop as the **shutdown owner** (its own scope, never
cancelled), after the first drain ``TICK`` (``end``):

1. **Fence** every producer of the inventory (:mod:`gcs.backend.lockstep.fence`).
   Admission closes; work admitted before it runs to completion or its
   cancellation and is journaled.
2. **Cancel** every pending request's scope and every producer scope.
3. **Retire work:** await the cancelled tasks, the executor jobs, the probe
   threads and the tracked broadcast futures, until none is left. A job does
   not stop because its awaiting coroutine was cancelled; its scope stops it.
   Then the event sink closes (a later broadcast is a fault) and only the
   shutdown owner may submit executor work.
4. **Companions:** stop them through the port; anything but an ack is a fault.
5. **Vehicles:** close each strictly; a close failure is a fault. Then every
   link's bus closes (one a scan opened has no vehicle), and the ESP32
   simulator port stops.
6. **Recording:** the pose-cadence flusher stops with a strict flush, the
   diagnostics session finalizes strictly (a record, ``session_finished``
   included, that failed to reach the timeline is a fault), and a backend
   file-log line that failed to reach its file is a fault.
7. **Executors:** shut down; every managed thread but the loop's has ended.

Any failure raises: the participant then ends unflushed (no clean end).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass

from gcs.backend import navpy_sim_runtime
from gcs.backend.diagnostics import diagnostic_session
from gcs.backend.lockstep.event_sink import EventSink
from gcs.backend.lockstep.faults import FaultLatch, unexpected
from gcs.backend.lockstep.fence import fence
from gcs.backend.lockstep.requests import RequestRunner
from gcs.backend.lockstep.scopes import LockstepProducerScopes
from gcs.backend.navpy_process_stop import STOP_ACKED
from gcs.backend.producer_inventory import ProducerInventory
from gcs.backend.routes.control import stop_esp32_sim_if_running
from gcs.backend.vehicle_manager import vehicle_mgr
from navpy.core.kernel.logging import pose_cadence_debug
from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.cancel_scope import CancelScope
from navpy.sim.lockstep.lockstep_link import LockstepLink


class ShutdownFault(RuntimeError):
    """A strict shutdown step failed: the backend has no clean end."""


@dataclass(frozen=True)
class ShutdownParts:
    loop: BatonEventLoop
    baton: BatonRuntime
    inventory: ProducerInventory
    scopes: LockstepProducerScopes
    runner: RequestRunner
    sink: EventSink
    links: tuple[LockstepLink, ...]
    faults: FaultLatch


async def strict_shutdown(parts: ShutdownParts, owner: CancelScope) -> None:
    """Steps 1-7, run in the shutdown owner's scope ``owner``."""
    tasks = await fence(parts.inventory)
    tasks += parts.runner.cancel_all()
    parts.scopes.cancel_all()
    await _retire(parts, tasks)
    parts.sink.close()
    parts.loop.baton_executor.gate = lambda: _owner_only(owner, parts.faults)
    outcomes = await asyncio.to_thread(navpy_sim_runtime.stop_all_instances)
    missed = {sys_id: outcome for sys_id, outcome in outcomes.items() if outcome != STOP_ACKED}
    if missed:
        raise ShutdownFault(f"companion stop not acknowledged: {missed}")
    for sys_id in sorted(vehicle_mgr.vehicles):
        await asyncio.to_thread(_close_strictly, sys_id)
    for link in parts.links:
        await asyncio.to_thread(link.close)
        if link.state not in ("unopened", "closed"):
            raise ShutdownFault(f"{link.device} did not close: {link.state}")
    stop_esp32_sim_if_running()
    await asyncio.to_thread(_flush_recording)
    await parts.loop.shutdown_default_executor()
    loop_thread = parts.baton.current()
    alive = [t.name for t in parts.baton.threads.alive() if t is not loop_thread]
    if alive:
        raise ShutdownFault(f"managed threads outlived the shutdown: {alive}")


async def _retire(parts: ShutdownParts, tasks: list[asyncio.Task]) -> None:
    """Await every unit of admitted work until nothing is left running."""
    _observed(parts, await asyncio.gather(*tasks, return_exceptions=True))
    while True:
        futures = parts.loop.baton_executor.pending() + parts.inventory.futures()
        threads = [t for t in parts.inventory.threads() if t.is_alive()]
        if not futures and not threads:
            parts.loop.job_failures.retire()  # failures nobody awaited: retrieved, or a fault
            return
        _observed(parts, await asyncio.gather(*(asyncio.wrap_future(f) for f in futures),
                                              return_exceptions=True))
        for thread in threads:
            await asyncio.to_thread(thread.join)


def _observed(parts: ShutdownParts, results: list[object]) -> None:
    """Every retired unit's end is observed: an unexpected one is a fault."""
    failed = [r for r in results if isinstance(r, BaseException) and unexpected(r)]
    for error in failed:
        parts.faults.latch("retired work", error)
    if failed:
        raise ShutdownFault(f"admitted work failed during the shutdown: {failed!r}")


def _owner_only(owner: CancelScope, faults: FaultLatch) -> None:
    """The post-sink gate: a producer's submission is latched, not only raised."""
    if cancel_scope.current() is not owner:
        faults.latch("executor after the sink closed", "ShutdownFault")
        raise ShutdownFault("executor work submitted after the fence by a producer")


def _close_strictly(sys_id: int) -> None:
    entry = vehicle_mgr.get_vehicle(sys_id)
    if entry is not None:
        entry.vehicle.close()  # raises on a failed close; removal then closes nothing
    vehicle_mgr.remove_vehicle(sys_id)


def _flush_recording() -> None:
    from gcs.backend.main import check_file_logging

    pose_cadence_debug.stop_flusher()
    pose_cadence_debug.dump_all(strict=True)
    diagnostic_session.stop_strictly()
    check_file_logging()


__all__ = ["ShutdownFault", "ShutdownParts", "strict_shutdown"]
