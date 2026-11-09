"""One mission-upload transaction per vehicle at a time.

A vehicle's upload is several transfers: the mission, then RTL_ALTITUDE, the
rally table that points every RTL at the mission's DDH, and the geofence. The
mission lock covers each transfer alone, so without this lock two overlapping
requests could interleave: B installs mission B and hub B while A waits
between its mission and its rally step, then A writes hub A over B's, and both
report success (Codex round 1 on the rally change, finding 1). Holding this
lock across the whole per-vehicle update keeps each vehicle's mission and its
hub from the same request.
"""
from __future__ import annotations

import asyncio
import math
import weakref
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from navpy.core.ports.parameter_exchange import ExchangePolicy
from gcs.backend.exchange_budget import expired, remaining_s


#: A request that could not start its update before its deadline: the vehicle
#: was still busy with another request's update. Nothing was sent to it.
UPLOAD_BUSY_ERROR = "Another upload to this vehicle was still running at the deadline"


class VehicleUploadLocks:
    """Per-vehicle asyncio locks, one set per event loop (an asyncio lock
    belongs to the loop it first waits on)."""

    def __init__(self) -> None:
        self._by_loop: weakref.WeakKeyDictionary[
            asyncio.AbstractEventLoop, dict[int, asyncio.Lock]] = weakref.WeakKeyDictionary()

    def _lock(self, sys_id: int) -> asyncio.Lock:
        locks = self._by_loop.setdefault(asyncio.get_running_loop(), {})
        lock = locks.get(sys_id)
        if lock is None:
            lock = locks[sys_id] = asyncio.Lock()
        return lock

    @asynccontextmanager
    async def held(self, sys_id: int, policy: ExchangePolicy | None) -> AsyncIterator[bool]:
        """Hold ``sys_id``'s upload lock; yields whether it is held.

        A free lock is taken at once, whatever the deadline: the update's own
        transfers then apply the deadline and report each step that could not
        start. Only the wait for another request's update is bounded: it ends
        at the request's deadline, and then nothing is held.
        """
        lock = self._lock(sys_id)
        acquired = False
        if not lock.locked():
            acquired = await lock.acquire()
        elif not expired(policy):
            wait_s = remaining_s(policy)
            try:
                if math.isinf(wait_s):
                    acquired = await lock.acquire()
                else:
                    acquired = await asyncio.wait_for(lock.acquire(), timeout=wait_s)
            except asyncio.TimeoutError:
                acquired = False
        try:
            yield acquired
        finally:
            if acquired:
                lock.release()


#: The backend's one registry: every upload request serializes through it.
vehicle_upload_locks = VehicleUploadLocks()
