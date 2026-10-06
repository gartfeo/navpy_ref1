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
      `threading.RLock`.
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
import math
import secrets
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Awaitable, Callable, Optional, Sequence

from navpy.modules.vehicle._mavftp.param_pck import (
    AP_TYPE_FLOAT, AP_TYPE_INT8, AP_TYPE_INT16, AP_TYPE_INT32,
    ParamRecord,
)
from navpy.modules.vehicle.full_param_snapshot import FullParamSnapshot

from gcs.backend.param_readonly import is_readonly

if TYPE_CHECKING:
    from navpy.modules.vehicle.vehicle_mav import VehicleMav

log = logging.getLogger(__name__)


# AP-packed type → MAVLink on-wire param type. AP codes (1..4) are NOT
# numerically equal to MAV_PARAM_TYPE_* (which start at 1 = MAV_PARAM_TYPE_UINT8).
# Per Codex pre-step finding 1: backend derives this from the cached snapshot
# rather than trusting the client.
def _mav_param_type_for(ap_type: int) -> Optional[int]:
    from pymavlink.dialects.v20.ardupilotmega import (
        MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_REAL32,
    )
    return {
        AP_TYPE_INT8: MAV_PARAM_TYPE_INT8,
        AP_TYPE_INT16: MAV_PARAM_TYPE_INT16,
        AP_TYPE_INT32: MAV_PARAM_TYPE_INT32,
        AP_TYPE_FLOAT: MAV_PARAM_TYPE_REAL32,
    }.get(ap_type)


# AP integer storage ranges (signed). `param.pck` carries the storage type
# only, so this is the storage range — not logical @Range metadata.
_AP_INT_BOUNDS: dict[int, tuple[int, int]] = {
    AP_TYPE_INT8: (-128, 127),
    AP_TYPE_INT16: (-32768, 32767),
    AP_TYPE_INT32: (-(2 ** 31), 2 ** 31 - 1),
}


def _coerce_to_stored(ap_type: int, value):
    """Coerce a value to what an AP param of this type would STORE.

    Integer types (int8/16/32) truncate toward zero and clamp to the signed
    storage range — mirroring the autopilot's `AP_Param::set_float` and the
    frontend `coerceValueForRecord`, so every full-param write path (cell
    editor, `.param` import, raw PUT) stages the same value. Float passes a
    finite value through. Non-finite / non-numeric returns ``None`` (rejected
    before send — the wire never sees it).

    Deliberately NOT `vehicle_mav._value_in_int_range`: that is the shared
    vehicle-layer guard, keyed by MAVLink wire types (incl. unsigned/64-bit)
    and validating only after integer conversion. This is the full-param
    boundary using AP storage semantics.
    """
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    bounds = _AP_INT_BOUNDS.get(ap_type)
    if bounds is None:
        return f  # float (ap_type 4) — finite passthrough
    lo, hi = bounds
    return min(hi, max(lo, math.trunc(f)))


# ---- DTOs --------------------------------------------------------------
@dataclass(frozen=True)
class ParamChange:
    name: str
    value: float | int


@dataclass
class CellResult:
    ok: bool
    value: float | int | None = None
    error: Optional[str] = None


@dataclass
class WriteBatchResult:
    snapshot_stale: bool
    results: dict[str, CellResult] = field(default_factory=dict)


@dataclass(frozen=True)
class ArmedToken:
    nonce: str
    expires_at_unix_s: float


# ---- Errors ------------------------------------------------------------
class FullParamError(Exception):
    """Backend full-param coordinator error."""


class ArmedWriteRejected(FullParamError):
    """Vehicle is armed and the request did not present a valid arm token."""


class UnknownParam(FullParamError):
    """Caller wrote a name that isn't in the cached snapshot."""


# ---- Coordinator -------------------------------------------------------
ARMED_TOKEN_TTL_S = 30.0

# Max concurrent full-param MAVFTP downloads per physical link (device).
# Different vehicles on one link overlap up to this bound, so a small fleet
# loads in parallel — the per-vehicle cost is mostly the autopilot generating
# `param.pck`, which overlaps well — while a large fleet on one shared radio
# can't flood it. Same-vehicle fetches are still deduped by a per-sys_id lock.
MAX_CONCURRENT_BUS_FETCHES = 6


class FullParamCache:
    def __init__(
        self,
        max_concurrent_bus_fetches: int = MAX_CONCURRENT_BUS_FETCHES,
    ) -> None:
        # Sync state — touched from event loop AND MavBus reader thread.
        self._state_lock = threading.RLock()
        self._snapshots: dict[int, FullParamSnapshot] = {}
        self._stale_sys_ids: set[int] = set()
        self._stale_names_by_sys_id: dict[int, set[str]] = {}
        self._generations: dict[int, int] = {}
        self._tokens: dict[int, ArmedToken] = {}
        self._subscribed: set[int] = set()
        # Async sequencing — only touched from event loop.
        self._max_bus_fetches = max(1, int(max_concurrent_bus_fetches))
        self._sid_fetch_locks: dict[int, asyncio.Lock] = {}
        self._bus_sems: dict[str, asyncio.Semaphore] = {}
        self._write_locks: dict[int, asyncio.Lock] = {}

    # -------------------------------------------------------------------
    # Public state access (thread-safe)
    # -------------------------------------------------------------------
    def peek(self, sys_id: int) -> Optional[FullParamSnapshot]:
        """Return retained snapshot metadata, including stale snapshots.

        Writes intentionally use stale snapshots for name/type lookup; reads
        use `peek_fresh()` so stale values are refetched.
        """
        with self._state_lock:
            return self._snapshots.get(sys_id)

    def peek_fresh(self, sys_id: int) -> Optional[FullParamSnapshot]:
        with self._state_lock:
            if sys_id in self._stale_sys_ids:
                return None
            return self._snapshots.get(sys_id)

    def is_stale(self, sys_id: int) -> bool:
        with self._state_lock:
            return sys_id in self._stale_sys_ids

    def invalidate(self, sys_id: int) -> None:
        with self._state_lock:
            self._snapshots.pop(sys_id, None)
            self._stale_sys_ids.discard(sys_id)
            self._stale_names_by_sys_id.pop(sys_id, None)
            self._bump_generation(sys_id)

    def invalidate_keys(self, sys_id: int, names: Sequence[str]) -> None:
        """Bump generation and mark snapshot stale if any of `names` is in it.

        Always bumps the generation when called with non-empty names — even
        if no snapshot is currently cached — so a fetch in flight at the
        time of the invalidation knows its result may be stale. (Per Codex
        post-step finding 1.)
        """
        upper = {n.upper() for n in names if n}
        if not upper:
            return
        with self._state_lock:
            self._bump_generation(sys_id)
            snap = self._snapshots.get(sys_id)
            if snap is None:
                return
            overlap = upper & set(snap.by_name.keys())
            if overlap:
                self._stale_sys_ids.add(sys_id)
                self._stale_names_by_sys_id.setdefault(sys_id, set()).update(overlap)

    def forget_vehicle(self, sys_id: int) -> None:
        with self._state_lock:
            self._snapshots.pop(sys_id, None)
            self._stale_sys_ids.discard(sys_id)
            self._stale_names_by_sys_id.pop(sys_id, None)
            self._tokens.pop(sys_id, None)
            self._subscribed.discard(sys_id)
            self._bump_generation(sys_id)
        # Async locks are not removed here; stale lock objects are cheap
        # and removing them would race with in-flight fetches.

    # -------------------------------------------------------------------
    # Armed-write tokens (O7)
    # -------------------------------------------------------------------
    def issue_armed_token(self, sys_id: int) -> ArmedToken:
        token = ArmedToken(
            nonce=secrets.token_urlsafe(16),
            expires_at_unix_s=time.time() + ARMED_TOKEN_TTL_S,
        )
        with self._state_lock:
            self._tokens[sys_id] = token
        return token

    def _consume_token(self, sys_id: int, presented: Optional[str]) -> bool:
        if not presented:
            return False
        with self._state_lock:
            token = self._tokens.get(sys_id)
            if token is None:
                return False
            self._tokens.pop(sys_id, None)
        return token.nonce == presented and time.time() < token.expires_at_unix_s

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

            results: dict[str, CellResult] = {}
            attempted_supported_write = False
            total = len(changes)
            sent_names: list[str] = []
            for idx, change in enumerate(changes):
                if progress_callback is not None:
                    # `idx` params already completed when starting this one.
                    await progress_callback(
                        {"written": idx, "total": total, "done": False})
                key = change.name.upper()
                if is_readonly(key):
                    results[key] = CellResult(
                        ok=False, error="read-only parameter")
                    continue
                rec = snapshot.by_name.get(key)
                if rec is None:
                    results[key] = CellResult(
                        ok=False, error="unknown parameter")
                    continue
                mav_type = _mav_param_type_for(rec.ap_type)
                if mav_type is None:
                    results[key] = CellResult(
                        ok=False,
                        error=f"unsupported AP type {rec.ap_type}",
                    )
                    continue
                # Honest-permissive: stage the value the autopilot would store
                # (truncate + clamp for ints), so a raw PUT of 2.1 to an int8
                # param writes 2 rather than failing with a misleading echo
                # error. Non-finite values can't be stored — reject precisely,
                # before send.
                stored = _coerce_to_stored(rec.ap_type, change.value)
                if stored is None:
                    results[key] = CellResult(
                        ok=False,
                        error=f"{change.value!r} is not a finite number",
                    )
                    continue
                attempted_supported_write = True
                sent_names.append(key)
                try:
                    ok = await asyncio.to_thread(
                        vehicle.set_parameter,
                        key, stored,
                        mav_param_type=mav_type,
                        timeout=per_param_timeout,
                    )
                except Exception as exc:
                    log.exception(
                        "full_params: set_parameter %s raised: %s", key, exc)
                    results[key] = CellResult(
                        ok=False, error=f"{type(exc).__name__}: {exc}")
                    continue
                if ok:
                    results[key] = CellResult(ok=True, value=stored)
                else:
                    results[key] = CellResult(
                        ok=False, error="echo did not match within timeout")
            if progress_callback is not None:
                await progress_callback(
                    {"written": total, "total": total, "done": True})

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
            snapshot_stale=attempted_supported_write,
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

    def _bump_generation(self, sys_id: int) -> None:
        # Caller holds _state_lock.
        self._generations[sys_id] = self._generations.get(sys_id, 0) + 1

    def _ensure_subscribed(self, sys_id: int, vehicle: "VehicleMav") -> None:
        with self._state_lock:
            if sys_id in self._subscribed:
                return
            self._subscribed.add(sys_id)

        def _observed_param_value(msg) -> None:
            try:
                pname = msg.param_id.rstrip("\x00").upper()
                pvalue = float(msg.param_value)
            except Exception:
                return
            with self._state_lock:
                snap = self._snapshots.get(sys_id)
                dirty_names = self._stale_names_by_sys_id.get(sys_id, set()).copy()
            if snap is None:
                return
            rec = snap.by_name.get(pname)
            if rec is None:
                return
            if pname in dirty_names:
                return
            try:
                cached_value = float(rec.value)
            except (TypeError, ValueError):
                return
            # Only invalidate when the value actually changed — GET replies
            # echo the same value and shouldn't bust the cache. Use a small
            # tolerance for float drift.
            if abs(cached_value - pvalue) > 1e-6 * max(abs(cached_value), 1.0):
                self.invalidate_keys(sys_id, [pname])

        vehicle.on_message("PARAM_VALUE", _observed_param_value)


# Module-level singleton consumed by routes + invalidation hooks.
full_param_cache = FullParamCache()
