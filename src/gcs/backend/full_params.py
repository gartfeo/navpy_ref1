"""Backend coordinator for full-parameter snapshots and writes.

Owns:
    - per-vehicle ParamPck snapshot cache
    - per-MavBus async lock so MAVFTP transfers across multiple vehicles on
      the same physical link serialise (D8)
    - per-vehicle write-batch async lock so the same vehicle never has two
      batches in flight
    - per-vehicle short-lived "armed write" tokens so a long-press in the UI
      is required before the backend accepts a write while the vehicle is
      armed (O7)
    - cache invalidation on full-param writes, AAS writes, mission uploads,
      observed PARAM_VALUE messages whose value differs from cached, and
      vehicle removal/reconnect (D6, D7)

Threading model:
    - FastAPI event loop calls async methods.
    - The MavBus reader thread invokes `_observed_param_value` on the
      vehicle's PARAM_VALUE callback channel. State dicts are guarded by a
      runtime RLock (`full_param_store.FullParamStore`).
    - Async sequencing (`_sid_fetch_locks`, `_bus_sems`, `_write_locks`) uses
      `asyncio` primitives so multiple coroutines queue at the await without
      tying up executor threads. The actual MAVFTP read runs via
      `asyncio.to_thread` so the event loop stays free. A per-vehicle fetch
      lock dedups same-sys_id fetches; a per-device semaphore bounds how many
      vehicles on one link download in parallel.

Generation counter:
    Each invalidation bumps `_generations[sys_id]`. After `to_thread` returns
    a fetched snapshot we only store it if the generation hasn't changed —
    otherwise an invalidation that races with the fetch could be lost
    (covered by Codex pre-step finding 2).
"""
from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Awaitable, Callable, Optional, Sequence

from gcs.backend.full_param_store import FullParamStore
from gcs.backend.full_param_values import (
    ArmedWriteRejected,
    FullParamError,
    ParamChange,
    WriteBatchResult,
)
from gcs.backend.full_param_writes import send_param_changes
from navpy.plugins.vehicle.ardupilot.full_param_snapshot import FullParamSnapshot

if TYPE_CHECKING:
    from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

log = logging.getLogger(__name__)


# Max concurrent full-param MAVFTP downloads per physical link (device).
# Different vehicles on one link overlap up to this bound, so a small fleet
# loads in parallel — the per-vehicle cost is mostly the autopilot generating
# `param.pck`, which overlaps well — while a large fleet on one shared radio
# can't flood it. Same-vehicle fetches are still deduped by a per-sys_id lock.
MAX_CONCURRENT_BUS_FETCHES = 6


class FullParamCache(FullParamStore):
    def __init__(
        self,
        max_concurrent_bus_fetches: int = MAX_CONCURRENT_BUS_FETCHES,
    ) -> None:
        super().__init__()
        # Async sequencing — only touched from event loop.
        self._max_bus_fetches = max(1, int(max_concurrent_bus_fetches))
        self._sid_fetch_locks: dict[int, asyncio.Lock] = {}
        self._bus_sems: dict[str, asyncio.Semaphore] = {}
        self._write_locks: dict[int, asyncio.Lock] = {}

    # -------------------------------------------------------------------
    # Fetch
    # -------------------------------------------------------------------
    async def get_or_fetch(
        self,
        sys_id: int,
        vehicle: "VehicleMav",
        device: str,
        *,
        refresh: bool = False,
        timeout: float = 30.0,
        max_retries: int = 1,
        progress_callback: Optional[Callable[[dict | None], None]] = None,
    ) -> FullParamSnapshot:
        """Return the cached snapshot or fetch a fresh one via MAVFTP.

        Always fetches with `?withdefaults=1` so the cache representation
        is consistent across callers (per Codex post-step finding 4).

        Concurrent callers for the same sys_id serialise on
        `_sid_fetch_locks[sys_id]` and each see the same snapshot thanks to the
        post-acquire double-check. Different vehicles on the same link download
        in parallel, bounded by `_bus_sems[device]` (max_concurrent_bus_fetches).

        If an invalidation arrives while a fetch is in flight, the fetched
        snapshot is discarded and the fetch is retried up to `max_retries`
        times before raising — otherwise the caller would see a snapshot
        that is fresh-looking but actually stale (per Codex post-step
        finding 1, second half).
        """
        # Fast path: cache hit.
        if not refresh:
            cached = self.peek_fresh(sys_id)
            if cached is not None:
                return cached

        # Dedup fetches for the SAME vehicle; different vehicles proceed
        # independently and overlap up to the per-link semaphore below.
        sid_lock = self._sid_fetch_lock(sys_id)
        async with sid_lock:
            # Re-check under the per-vehicle lock — another task may have
            # refreshed it.
            if not refresh:
                cached = self.peek_fresh(sys_id)
                if cached is not None:
                    return cached

            # Subscribe lazily so observed PARAM_VALUE messages can invalidate
            # the cache (per D7). One callback per vehicle for its lifetime.
            self._ensure_subscribed(sys_id, vehicle)

            # Bound concurrent MAVFTP downloads on this physical link so a large
            # fleet on one radio can't flood it, while a small fleet parallelises.
            async with self._bus_sem(device):
                attempts_left = max_retries + 1
                while attempts_left > 0:
                    attempts_left -= 1
                    with self._state_lock:
                        generation_at_start = self._generations.get(sys_id, 0)

                    snapshot: FullParamSnapshot = await asyncio.to_thread(
                        vehicle.fetch_full_param_snapshot,
                        with_defaults=True,
                        timeout=timeout,
                        progress_callback=progress_callback,
                    )

                    with self._state_lock:
                        if self._generations.get(sys_id, 0) == generation_at_start:
                            self._snapshots[sys_id] = snapshot
                            self._stale_sys_ids.discard(sys_id)
                            self._stale_names_by_sys_id.pop(sys_id, None)
                            return snapshot
                    # Generation bumped during fetch — retry rather than return
                    # a snapshot that was already invalidated.
                    log.info(
                        "full_params: snapshot for sys_id=%d invalidated during "
                        "fetch; retrying (%d attempt(s) left)",
                        sys_id, attempts_left,
                    )
                raise FullParamError(
                    f"snapshot for sys_id={sys_id} kept invalidating during "
                    "fetch; gave up after retries.",
                )

    # -------------------------------------------------------------------
    # Write batch
    # -------------------------------------------------------------------
    async def write_batch(
        self,
        sys_id: int,
        vehicle: "VehicleMav",
        changes: Sequence[ParamChange],
        *,
        armed_token: Optional[str] = None,
        per_param_timeout: float = 2.0,
        progress_callback: Optional[Callable[[dict], Awaitable[None]]] = None,
    ) -> WriteBatchResult:
        if not changes:
            return WriteBatchResult(snapshot_stale=False)

        write_lock = self._write_lock(sys_id)
        # Per Codex post-step finding 2: armed check, token consumption, and
        # snapshot lookup MUST happen under the per-vehicle write lock so two
        # concurrent armed PUTs can't both consume tokens AND so the second
        # writer can't read a snapshot the first batch already invalidated.
        async with write_lock:
            if vehicle.is_armed:
                if not self._consume_token(sys_id, armed_token):
                    raise ArmedWriteRejected(
                        "Vehicle is armed; long-press arm-token required.")

            snapshot = self.peek(sys_id)
            if snapshot is None:
                raise FullParamError(
                    "no full-param snapshot cached; GET parameters first.")

            results, sent_names = await send_param_changes(
                vehicle, snapshot, changes,
                per_param_timeout=per_param_timeout,
                progress_callback=progress_callback,
            )

        # Invalidate the names we actually sent to the vehicle — even on echo
        # timeout, because the autopilot may have stored the value before the
        # echo arrived (per Codex pre-step finding 3). Names rejected before
        # send (unknown, unsupported type, non-finite) never reached the wire,
        # so they don't stale the cache.
        if sent_names:
            self.invalidate_keys(sys_id, sent_names)
        # Per Codex post-step finding 3: snapshot is stale iff we actually
        # reached `set_parameter` for any supported write — regardless of
        # whether the echo verified — because cached values are now stale.
        return WriteBatchResult(
            snapshot_stale=bool(sent_names),
            results=results,
        )

    # -------------------------------------------------------------------
    # Internals
    # -------------------------------------------------------------------
    def _bus_sem(self, device: str) -> asyncio.Semaphore:
        # Async primitives must be bound to the running loop. Lazily create
        # them so tests using a fresh loop work without setUp coordination.
        sem = self._bus_sems.get(device)
        if sem is None:
            sem = asyncio.Semaphore(self._max_bus_fetches)
            self._bus_sems[device] = sem
        return sem

    def _sid_fetch_lock(self, sys_id: int) -> asyncio.Lock:
        lock = self._sid_fetch_locks.get(sys_id)
        if lock is None:
            lock = asyncio.Lock()
            self._sid_fetch_locks[sys_id] = lock
        return lock

    def _write_lock(self, sys_id: int) -> asyncio.Lock:
        lock = self._write_locks.get(sys_id)
        if lock is None:
            lock = asyncio.Lock()
            self._write_locks[sys_id] = lock
        return lock


# Module-level singleton consumed by routes + invalidation hooks.
full_param_cache = FullParamCache()
