"""Thread-safe full-parameter state: snapshots, staleness, tokens, observer.

The synchronous half of ``full_params.FullParamCache``: the FastAPI event
loop and the MavBus reader thread (PARAM_VALUE observer) both touch it, so
every state dict is guarded by one runtime RLock.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Sequence

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from gcs.backend.full_param_values import ArmedToken
from gcs.backend.producer_inventory import producers
from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.full_param_snapshot import FullParamSnapshot

if TYPE_CHECKING:
    from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

ARMED_TOKEN_TTL_S = 30.0


class FullParamStore:
    def __init__(self) -> None:
        # Sync state — touched from event loop AND MavBus reader thread.
        self._state_lock = rt.RLock()
        self._snapshots: dict[int, FullParamSnapshot] = {}
        self._stale_sys_ids: set[int] = set()
        self._stale_names_by_sys_id: dict[int, set[str]] = {}
        self._generations: dict[int, int] = {}
        self._tokens: dict[int, ArmedToken] = {}
        self._subscribed: set[int] = set()

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
            nonce=rt.token_urlsafe(16),
            expires_at_unix_s=rt.time() + ARMED_TOKEN_TTL_S,
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
        return token.nonce == presented and rt.time() < token.expires_at_unix_s

    # -------------------------------------------------------------------
    # Internals
    # -------------------------------------------------------------------
    def _bump_generation(self, sys_id: int) -> None:
        # Caller holds _state_lock.
        self._generations[sys_id] = self._generations.get(sys_id, 0) + 1

    def _ensure_subscribed(self, sys_id: int, vehicle: "VehicleMav") -> None:
        with self._state_lock:
            if sys_id in self._subscribed:
                return
            self._subscribed.add(sys_id)

        def _observed_param_value(msg: MAVLink_message) -> None:
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

        producers.keep_subscription(
            "full_param_observer", vehicle.on_message("PARAM_VALUE", _observed_param_value))


__all__ = ["ARMED_TOKEN_TTL_S", "FullParamStore"]
