"""Unit tests for `gcs.backend.full_params.FullParamCache`.

Drives the coordinator with stub vehicles and asserts:
- bus-level serialisation across two vehicles on the same device
- per-vehicle write-batch lock
- generation guard prevents stale snapshots from landing after a race
- armed-write token: TTL, one-shot, nonce mismatch
- `invalidate_keys` marks retained snapshots stale when a name overlaps
- observed PARAM_VALUE invalidates only when value differs from cache
- `forget_vehicle` drops snapshot + token + subscription
"""
from __future__ import annotations

import asyncio
import struct
import time
import unittest
from dataclasses import dataclass
from typing import Optional
from unittest.mock import MagicMock

from gcs.backend.full_params import (
    ARMED_TOKEN_TTL_S,
    ArmedWriteRejected,
    FullParamCache,
    FullParamError,
    ParamChange,
    _coerce_to_stored,
    _mav_param_type_for,
)
from navpy.modules.vehicle._mavftp.param_pck import (
    AP_TYPE_FLOAT, AP_TYPE_INT8, AP_TYPE_INT16, AP_TYPE_INT32,
    FLAG_HAS_DEFAULT, PCK_MAGIC, ParamRecord, parse_param_pck,
)
from navpy.modules.vehicle.full_param_snapshot import FullParamSnapshot


def _build_snapshot(
    *, target_system: int = 42, with_defaults: bool = True,
    extra_records: tuple = (),
) -> FullParamSnapshot:
    rec_a = bytes([(0 << 4) | 1, (3 - 1) << 4 | 0]) + b"AAA" + struct.pack("<b", 5)
    rec_b = bytes([(FLAG_HAS_DEFAULT << 4) | AP_TYPE_FLOAT, (1 - 1) << 4 | 2]) \
        + b"B" + struct.pack("<f", 1.5) + struct.pack("<f", 0.0)
    blob = struct.pack("<HHH", PCK_MAGIC, 2, 2) + rec_a + rec_b
    pck = parse_param_pck(blob, defaults_requested=with_defaults)
    return FullParamSnapshot(
        target_system=target_system,
        fetched_at_unix_s=time.time(),
        with_defaults=with_defaults,
        pck=pck,
    )


def _stub_vehicle(target_system: int = 42, *, fetch_delay: float = 0.0,
                  is_armed: bool = False) -> MagicMock:
    v = MagicMock()
    v.target_system = target_system
    v.is_armed = is_armed
    v._callbacks = {}

    def _on_message(name, cb):
        v._callbacks.setdefault(name, []).append(cb)
    v.on_message.side_effect = _on_message

    async def _fetch_async(*args, **kwargs):
        if fetch_delay > 0:
            await asyncio.sleep(fetch_delay)
        return _build_snapshot(target_system=target_system,
                               with_defaults=kwargs.get("with_defaults", True))

    def _fetch_sync(**kwargs):
        # called from asyncio.to_thread — just block briefly via time.sleep.
        if fetch_delay > 0:
            time.sleep(fetch_delay)
        return _build_snapshot(target_system=target_system,
                               with_defaults=kwargs.get("with_defaults", True))

    v.fetch_full_param_snapshot.side_effect = _fetch_sync
    return v


# ---------------------------------------------------------------------
class TestCoerceToStored(unittest.TestCase):
    """AP storage coercion. These vectors MUST stay identical to the frontend
    `coerceValueForRecord` vectors in test_full_params_js.py — one rule, two
    implementations (Codex anti-drift)."""

    def test_int_truncates_toward_zero(self):
        self.assertEqual(_coerce_to_stored(AP_TYPE_INT32, 2.1), 2)
        self.assertEqual(_coerce_to_stored(AP_TYPE_INT32, 2.9), 2)
        self.assertEqual(_coerce_to_stored(AP_TYPE_INT32, -3.9), -3)

    def test_int_clamps_to_storage_range(self):
        self.assertEqual(_coerce_to_stored(AP_TYPE_INT8, 200), 127)
        self.assertEqual(_coerce_to_stored(AP_TYPE_INT8, -200), -128)
        self.assertEqual(_coerce_to_stored(AP_TYPE_INT16, 70000), 32767)

    def test_int_result_is_int(self):
        self.assertIsInstance(_coerce_to_stored(AP_TYPE_INT8, 5.0), int)

    def test_float_passthrough(self):
        self.assertEqual(_coerce_to_stored(AP_TYPE_FLOAT, 2.1), 2.1)

    def test_non_finite_returns_none(self):
        self.assertIsNone(_coerce_to_stored(AP_TYPE_INT8, float("inf")))
        self.assertIsNone(_coerce_to_stored(AP_TYPE_INT8, float("nan")))
        self.assertIsNone(_coerce_to_stored(AP_TYPE_FLOAT, float("inf")))

    def test_non_numeric_returns_none(self):
        self.assertIsNone(_coerce_to_stored(AP_TYPE_INT8, "abc"))
        self.assertIsNone(_coerce_to_stored(AP_TYPE_INT8, None))


# ---------------------------------------------------------------------
class TestArmedToken(unittest.TestCase):
    def setUp(self):
        self.cache = FullParamCache()

    def test_issue_returns_nonce_and_expiry(self):
        token = self.cache.issue_armed_token(42)
        self.assertTrue(token.nonce)
        self.assertGreater(token.expires_at_unix_s, time.time())
        self.assertLess(token.expires_at_unix_s,
                        time.time() + ARMED_TOKEN_TTL_S + 1.0)

    def test_consume_one_shot(self):
        token = self.cache.issue_armed_token(42)
        self.assertTrue(self.cache._consume_token(42, token.nonce))
        self.assertFalse(self.cache._consume_token(42, token.nonce))  # second use rejected

    def test_consume_wrong_nonce(self):
        self.cache.issue_armed_token(42)
        self.assertFalse(self.cache._consume_token(42, "garbage"))

    def test_consume_expired(self):
        token = self.cache.issue_armed_token(42)
        # Manually rewind the expiry so we can test TTL without sleeping.
        with self.cache._state_lock:
            self.cache._tokens[42] = type(token)(
                nonce=token.nonce, expires_at_unix_s=time.time() - 1.0,
            )
        self.assertFalse(self.cache._consume_token(42, token.nonce))


# ---------------------------------------------------------------------
class TestInvalidateKeys(unittest.TestCase):
    def setUp(self):
        self.cache = FullParamCache()
        snap = _build_snapshot(target_system=42)
        self.cache._snapshots[42] = snap

    def test_overlapping_name_marks_stale(self):
        self.cache.invalidate_keys(42, ["AAB"])
        self.assertIsNotNone(self.cache.peek(42))
        self.assertIsNone(self.cache.peek_fresh(42))
        self.assertTrue(self.cache.is_stale(42))
        self.assertEqual(self.cache._stale_names_by_sys_id[42], {"AAB"})

    def test_no_overlap_no_drop(self):
        self.cache.invalidate_keys(42, ["UNKNOWN_PARAM"])
        self.assertIsNotNone(self.cache.peek(42))
        self.assertIsNotNone(self.cache.peek_fresh(42))
        self.assertFalse(self.cache.is_stale(42))

    def test_empty_names_no_op(self):
        self.cache.invalidate_keys(42, [])
        self.assertIsNotNone(self.cache.peek(42))
        self.assertFalse(self.cache.is_stale(42))


# ---------------------------------------------------------------------
class TestForgetVehicle(unittest.TestCase):
    def test_clears_snapshot_token_subscription(self):
        cache = FullParamCache()
        cache._snapshots[42] = _build_snapshot()
        cache.invalidate_keys(42, ["AAB"])
        cache.issue_armed_token(42)
        cache._subscribed.add(42)
        cache.forget_vehicle(42)
        self.assertIsNone(cache.peek(42))
        self.assertFalse(cache.is_stale(42))
        self.assertNotIn(42, cache._stale_names_by_sys_id)
        self.assertNotIn(42, cache._tokens)
        self.assertNotIn(42, cache._subscribed)


# ---------------------------------------------------------------------
class TestObservedParamValue(unittest.TestCase):
    def setUp(self):
        self.cache = FullParamCache()
        self.vehicle = _stub_vehicle(42)

    def _wire_subscription_with_snapshot(self):
        snap = _build_snapshot(target_system=42)
        self.cache._snapshots[42] = snap
        self.cache._ensure_subscribed(42, self.vehicle)
        # Pull the registered callback so tests can fire it directly.
        return self.vehicle._callbacks["PARAM_VALUE"][0]

    def test_observed_same_value_no_invalidate(self):
        cb = self._wire_subscription_with_snapshot()
        msg = MagicMock(); msg.param_id = "AAB\x00"; msg.param_value = 1.5
        cb(msg)
        self.assertIsNotNone(self.cache.peek(42))

    def test_observed_changed_value_invalidates(self):
        cb = self._wire_subscription_with_snapshot()
        msg = MagicMock(); msg.param_id = "AAB"; msg.param_value = 9.0
        cb(msg)
        self.assertIsNotNone(self.cache.peek(42))
        self.assertTrue(self.cache.is_stale(42))
        self.assertIsNone(self.cache.peek_fresh(42))

    def test_observed_unknown_name_ignored(self):
        cb = self._wire_subscription_with_snapshot()
        msg = MagicMock(); msg.param_id = "NOT_A_PARAM"; msg.param_value = 9.0
        cb(msg)
        self.assertIsNotNone(self.cache.peek(42))

    def test_observed_dirty_name_does_not_reinvalidate(self):
        cb = self._wire_subscription_with_snapshot()
        self.cache.invalidate_keys(42, ["AAB"])
        before = self.cache._generations[42]

        msg = MagicMock(); msg.param_id = "AAB"; msg.param_value = 9.0
        cb(msg)

        self.assertEqual(self.cache._generations[42], before)
        self.assertTrue(self.cache.is_stale(42))

    def test_observed_unrelated_name_still_invalidates(self):
        cb = self._wire_subscription_with_snapshot()
        self.cache.invalidate_keys(42, ["AAB"])
        before = self.cache._generations[42]

        msg = MagicMock(); msg.param_id = "AAA"; msg.param_value = 6.0
        cb(msg)

        self.assertGreater(self.cache._generations[42], before)
        self.assertTrue(self.cache.is_stale(42))
        self.assertEqual(self.cache._stale_names_by_sys_id[42], {"AAA", "AAB"})

    def test_subscribe_idempotent(self):
        snap = _build_snapshot(target_system=42)
        self.cache._snapshots[42] = snap
        self.cache._ensure_subscribed(42, self.vehicle)
        self.cache._ensure_subscribed(42, self.vehicle)
        self.assertEqual(
            len(self.vehicle._callbacks["PARAM_VALUE"]), 1)


# ---------------------------------------------------------------------
class TestGetOrFetch(unittest.IsolatedAsyncioTestCase):
    async def test_cache_hit_skips_fetch(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        # Pre-populate.
        cache._snapshots[42] = _build_snapshot(target_system=42)
        snap = await cache.get_or_fetch(42, v, "dev:A")
        self.assertEqual(snap.target_system, 42)
        v.fetch_full_param_snapshot.assert_not_called()

    async def test_refresh_forces_fetch(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        await cache.get_or_fetch(42, v, "dev:A", refresh=True)
        v.fetch_full_param_snapshot.assert_called_once()

    async def test_stale_snapshot_forces_fetch_and_clears_stale(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        cache.invalidate_keys(42, ["AAB"])
        self.assertIsNotNone(cache.peek(42))
        self.assertTrue(cache.is_stale(42))

        snap = await cache.get_or_fetch(42, v, "dev:A")

        self.assertIsNotNone(snap)
        self.assertFalse(cache.is_stale(42))
        self.assertNotIn(42, cache._stale_names_by_sys_id)
        self.assertIsNotNone(cache.peek_fresh(42))
        v.fetch_full_param_snapshot.assert_called_once()

    async def test_progress_callback_is_passed_to_vehicle_fetch(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        events = []
        callback = events.append

        await cache.get_or_fetch(
            42,
            v,
            "dev:A",
            progress_callback=callback,
        )

        _, kwargs = v.fetch_full_param_snapshot.call_args
        self.assertIs(kwargs["progress_callback"], callback)

    async def test_concurrent_callers_double_check(self):
        """Two concurrent gets for an empty cache must result in ONE fetch."""
        cache = FullParamCache()
        v = _stub_vehicle(42, fetch_delay=0.05)
        await asyncio.gather(
            cache.get_or_fetch(42, v, "dev:A"),
            cache.get_or_fetch(42, v, "dev:A"),
        )
        # Bus lock + double-check ensures only one fetch ran.
        self.assertEqual(v.fetch_full_param_snapshot.call_count, 1)

    async def test_two_vehicles_same_bus_parallelise(self):
        """Two vehicles on the SAME device download concurrently (bounded
        parallelism) so a small fleet doesn't queue behind UAV 1.

        Same deterministic overlap proof as the different-buses case: both
        fetches must reach a shared ``Barrier(2)`` before either can return,
        so overlap is guaranteed structurally rather than inferred from a
        wall-clock/sleep window. If a regression serialised same-bus fetches
        (e.g. a per-device lock instead of a semaphore) the first would block
        at the barrier and time out with ``BrokenBarrierError``.
        """
        import threading
        cache = FullParamCache()  # default cap (6) comfortably admits both
        # Trips only when BOTH vehicles' fetches are simultaneously in flight.
        both_in_flight = threading.Barrier(2, timeout=10.0)

        def _make_fetch(target_system: int):
            def _fetch(**kw):
                both_in_flight.wait()
                return _build_snapshot(
                    target_system=target_system,
                    with_defaults=kw.get("with_defaults", True),
                )
            return _fetch

        v1 = _stub_vehicle(1)
        v2 = _stub_vehicle(2)
        v1.fetch_full_param_snapshot.side_effect = _make_fetch(1)
        v2.fetch_full_param_snapshot.side_effect = _make_fetch(2)

        snaps = await asyncio.gather(
            cache.get_or_fetch(1, v1, "dev:A"),
            cache.get_or_fetch(2, v2, "dev:A"),
        )
        # Both fetches ran and each passed the barrier (proving overlap).
        self.assertEqual({s.target_system for s in snaps}, {1, 2})
        v1.fetch_full_param_snapshot.assert_called_once()
        v2.fetch_full_param_snapshot.assert_called_once()

    async def test_bus_fetch_concurrency_is_bounded(self):
        """The per-link semaphore caps how many vehicles download at once, so
        a large fleet on one radio can't flood it.

        Two independent, deterministic assertions — no wall-clock threshold:
          * upper bound (the cap): a live counter records the max number of
            fetches simultaneously in flight; it must never exceed the cap.
          * lower bound (real overlap): each fetch blocks until `cap` fetches
            are concurrently in flight (the ``reached_cap`` event), so the cap
            is provably reached rather than inferred from a sleep window. The
            event is set once and never cleared, so vehicles beyond the first
            admitted batch pass straight through — this holds for any fleet
            size >= cap, with no barrier-vs-count coupling. If a regression
            serialised the fetches the cap can never be reached, the first
            fetch times out, and the test fails.
        """
        import threading
        cap = 2
        cache = FullParamCache(max_concurrent_bus_fetches=cap)
        tracker_lock = threading.Lock()
        in_flight = {"count": 0, "max": 0}
        reached_cap = threading.Event()

        def _make_fetch(target_system: int):
            def _fetch(**kw):
                with tracker_lock:
                    in_flight["count"] += 1
                    in_flight["max"] = max(in_flight["max"], in_flight["count"])
                    if in_flight["count"] >= cap:
                        reached_cap.set()
                try:
                    # Hold the permit until `cap` fetches overlap, so the cap
                    # is reached deterministically under any load. A serialised
                    # regression never sets the event → this times out → the
                    # AssertionError fails the test fast.
                    if not reached_cap.wait(timeout=10.0):
                        raise AssertionError(
                            "bus fetches never reached the cap concurrently; "
                            "likely serialised")
                    return _build_snapshot(
                        target_system=target_system,
                        with_defaults=kw.get("with_defaults", True),
                    )
                finally:
                    with tracker_lock:
                        in_flight["count"] -= 1
            return _fetch

        sids = (1, 2, 3, 4)
        vs = []
        for sid in sids:
            v = _stub_vehicle(sid)
            v.fetch_full_param_snapshot.side_effect = _make_fetch(sid)
            vs.append(v)
        await asyncio.gather(*[
            cache.get_or_fetch(sid, v, "dev:A") for sid, v in zip(sids, vs)
        ])
        # 4 vehicles, cap 2 → the cap was reached and never exceeded.
        self.assertEqual(in_flight["max"], cap)

    async def test_two_vehicles_different_buses_parallelise(self):
        """Two vehicles on DIFFERENT physical links download concurrently —
        a per-device semaphore must never serialise across links.

        Proven deterministically, not by wall-clock: each fetch arrives at a
        shared ``Barrier(2)`` and blocks until the OTHER fetch also arrives.
        If the two fetches truly overlap the barrier trips at once and both
        return; if a regression serialised them (e.g. a single global fetch
        lock), the first fetch blocks at the barrier and the second never
        starts, so the barrier times out and raises ``BrokenBarrierError`` —
        failing the test with no timing threshold to tune. The barrier itself
        is the overlap: both must be in flight before either can return.
        """
        import threading
        cache = FullParamCache()
        # Trips only when BOTH vehicles' fetches are simultaneously in flight.
        # Generous timeout: it only bounds the failure (serialised) case; a
        # genuinely parallel run trips it the instant the second fetch arrives.
        both_in_flight = threading.Barrier(2, timeout=10.0)

        def _make_fetch(target_system: int):
            def _fetch(**kw):
                # Blocks until the other vehicle's fetch is also here. Raises
                # BrokenBarrierError on timeout if they were serialised.
                both_in_flight.wait()
                return _build_snapshot(
                    target_system=target_system,
                    with_defaults=kw.get("with_defaults", True),
                )
            return _fetch

        v1 = _stub_vehicle(1)
        v2 = _stub_vehicle(2)
        v1.fetch_full_param_snapshot.side_effect = _make_fetch(1)
        v2.fetch_full_param_snapshot.side_effect = _make_fetch(2)

        snaps = await asyncio.gather(
            cache.get_or_fetch(1, v1, "dev:A"),
            cache.get_or_fetch(2, v2, "dev:B"),
        )
        # Both fetches ran and each passed the barrier (proving overlap).
        self.assertEqual({s.target_system for s in snaps}, {1, 2})
        v1.fetch_full_param_snapshot.assert_called_once()
        v2.fetch_full_param_snapshot.assert_called_once()

    async def test_invalidate_during_fetch_retries(self):
        """Generation guard: an invalidation that happens while a fetch is
        running must cause the just-fetched snapshot to be retried so the
        caller never sees a snapshot that was already invalidated."""
        cache = FullParamCache()
        v = _stub_vehicle(42, fetch_delay=0.1)

        async def _invalidator():
            await asyncio.sleep(0.03)
            cache.invalidate(42)

        await asyncio.gather(
            cache.get_or_fetch(42, v, "dev:A"),
            _invalidator(),
        )
        # First fetch was retried; second attempt's snapshot should land.
        self.assertIsNotNone(cache.peek(42))
        # fetch_full_param_snapshot called twice (initial + retry).
        self.assertEqual(v.fetch_full_param_snapshot.call_count, 2)

    async def test_invalidate_keys_during_first_fetch_with_no_cache_bumps_generation(self):
        """Codex post-step finding 1: invalidate_keys must bump generation
        even when no snapshot is cached, so a fetch in flight detects the
        race."""
        cache = FullParamCache()
        v = _stub_vehicle(42)
        # Establish the race by ordering, not by hoping the event loop runs
        # an invalidator inside a short worker-thread sleep.
        import threading
        loop = asyncio.get_running_loop()
        fetching = asyncio.Event()
        invalidated = threading.Event()
        calls = 0

        def _fetch(**kwargs):
            nonlocal calls
            calls += 1
            if calls == 1:
                loop.call_soon_threadsafe(fetching.set)
                if not invalidated.wait(timeout=10.0):
                    raise AssertionError("invalidation never reached the in-flight fetch")
            return _build_snapshot(target_system=42)

        v.fetch_full_param_snapshot.side_effect = _fetch

        async def _invalidator():
            await fetching.wait()
            try:
                # No snapshot in cache yet — must still bump generation.
                self.assertIsNone(cache.peek(42))
                cache.invalidate_keys(42, ["AAA"])
            finally:
                invalidated.set()

        await asyncio.gather(
            cache.get_or_fetch(42, v, "dev:A"),
            _invalidator(),
        )
        # Without the bump, the first snapshot would have landed and we'd
        # see exactly one fetch. With the bump, the first attempt is
        # discarded and a retry runs.
        self.assertEqual(v.fetch_full_param_snapshot.call_count, 2)


# ---------------------------------------------------------------------
class TestWriteBatch(unittest.IsolatedAsyncioTestCase):
    async def test_write_batch_no_snapshot_raises(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        with self.assertRaises(FullParamError):
            await cache.write_batch(
                42, v, [ParamChange(name="AAB", value=2.0)],
            )

    async def test_armed_no_token_rejected(self):
        cache = FullParamCache()
        v = _stub_vehicle(42, is_armed=True)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        with self.assertRaises(ArmedWriteRejected):
            await cache.write_batch(
                42, v, [ParamChange(name="AAB", value=2.0)],
            )

    async def test_armed_with_valid_token_accepted(self):
        cache = FullParamCache()
        v = _stub_vehicle(42, is_armed=True)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        token = cache.issue_armed_token(42)
        result = await cache.write_batch(
            42, v, [ParamChange(name="AAB", value=2.0)],
            armed_token=token.nonce,
        )
        self.assertTrue(result.results["AAB"].ok)
        self.assertTrue(result.snapshot_stale)

    async def test_write_batch_emits_progress(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        events = []

        async def _cb(ev):
            events.append(dict(ev))

        changes = [ParamChange(name="AAA", value=6.0),
                   ParamChange(name="AAB", value=2.0)]
        await cache.write_batch(42, v, changes, progress_callback=_cb)

        self.assertTrue(events)
        # Every event reports the full batch size.
        self.assertTrue(all(e["total"] == 2 for e in events))
        # `written` is monotonic non-decreasing and ends at the full count.
        writtens = [e["written"] for e in events]
        self.assertEqual(writtens, sorted(writtens))
        self.assertEqual(events[-1], {"written": 2, "total": 2, "done": True})
        # Only the final event is `done`.
        self.assertFalse(any(e["done"] for e in events[:-1]))

    async def test_write_batch_rejects_readonly(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(
            42, v,
            [ParamChange(name="MIS_TOTAL", value=5),
             ParamChange(name="STAT_BOOTCNT", value=1)],
        )
        self.assertFalse(result.results["MIS_TOTAL"].ok)
        self.assertIn("read-only", result.results["MIS_TOTAL"].error)
        self.assertFalse(result.results["STAT_BOOTCNT"].ok)
        # Never sent to the vehicle, and no supported write attempted so the
        # snapshot is not marked stale.
        v.set_parameter.assert_not_called()
        self.assertFalse(result.snapshot_stale)

    async def test_write_batch_no_changes_emits_nothing(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        events = []

        async def _cb(ev):
            events.append(dict(ev))

        await cache.write_batch(42, v, [], progress_callback=_cb)
        self.assertEqual(events, [])

    async def test_unknown_param_per_cell_error(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(
            42, v, [ParamChange(name="DOES_NOT_EXIST", value=1)],
        )
        self.assertFalse(result.results["DOES_NOT_EXIST"].ok)
        self.assertIn("unknown", result.results["DOES_NOT_EXIST"].error.lower())

    async def test_write_marks_snapshot_stale_even_on_failure(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = False
        cache._snapshots[42] = _build_snapshot(target_system=42)
        await cache.write_batch(42, v, [ParamChange(name="AAB", value=2.0)])
        # Values are stale because we attempted a write, but metadata is
        # retained for subsequent write type lookup.
        self.assertIsNotNone(cache.peek(42))
        self.assertIsNone(cache.peek_fresh(42))
        self.assertTrue(cache.is_stale(42))

    async def test_failed_write_still_marks_snapshot_stale(self):
        """If we attempted a supported write, snapshot_stale must be True
        even when set_parameter returned False (echo timeout) — the cached
        values are stale because the autopilot may have stored anyway."""
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = False
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(42, v, [ParamChange(name="AAB", value=2.0)])
        self.assertFalse(result.results["AAB"].ok)
        self.assertTrue(result.snapshot_stale)

    async def test_unknown_param_alone_does_not_mark_stale(self):
        """A batch of only-unknown names doesn't reach set_parameter so
        the cache is unaffected and `snapshot_stale` is False."""
        cache = FullParamCache()
        v = _stub_vehicle(42)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(
            42, v, [ParamChange(name="DOES_NOT_EXIST", value=1)],
        )
        self.assertFalse(result.snapshot_stale)
        # Cache survives — invalidate_keys was called with names but none
        # of them are in the snapshot, so the snapshot stays.
        self.assertIsNotNone(cache.peek(42))

    async def test_set_parameter_called_with_derived_mav_type(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        await cache.write_batch(42, v, [ParamChange(name="AAB", value=2.0)])
        v.set_parameter.assert_called_once()
        kwargs = v.set_parameter.call_args.kwargs
        self.assertEqual(kwargs["mav_param_type"], _mav_param_type_for(AP_TYPE_FLOAT))

    async def test_fractional_int_coerced_before_send(self):
        """A raw PUT of 2.1 to an int8 param sends the stored integer 2 and
        reports it — no misleading echo error (honest-permissive boundary)."""
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(42, v, [ParamChange(name="AAA", value=2.1)])
        v.set_parameter.assert_called_once()
        self.assertEqual(v.set_parameter.call_args.args[1], 2)  # stored int, not 2.1
        self.assertEqual(
            v.set_parameter.call_args.kwargs["mav_param_type"],
            _mav_param_type_for(AP_TYPE_INT8),
        )
        self.assertTrue(result.results["AAA"].ok)
        self.assertEqual(result.results["AAA"].value, 2)

    async def test_out_of_range_int_clamped_before_send(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(42, v, [ParamChange(name="AAA", value=200)])
        self.assertEqual(v.set_parameter.call_args.args[1], 127)
        self.assertEqual(result.results["AAA"].value, 127)

    async def test_non_finite_rejected_precisely_and_not_sent(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(
            42, v, [ParamChange(name="AAA", value=float("inf"))])
        v.set_parameter.assert_not_called()
        cell = result.results["AAA"]
        self.assertFalse(cell.ok)
        self.assertIn("finite", cell.error)
        # Rejected before send -> the cache must NOT be staled.
        self.assertFalse(result.snapshot_stale)
        self.assertIsNotNone(cache.peek_fresh(42))

    async def test_mixed_batch_stales_only_the_sent_name(self):
        """One valid write + one non-finite known param: only the sent name
        stales; the rejected one never reaches the wire."""
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)
        result = await cache.write_batch(42, v, [
            ParamChange(name="AAB", value=2.0),           # valid float -> sent
            ParamChange(name="AAA", value=float("nan")),  # non-finite -> rejected
        ])
        self.assertTrue(result.results["AAB"].ok)
        self.assertFalse(result.results["AAA"].ok)
        self.assertTrue(result.snapshot_stale)            # AAB reached the wire
        v.set_parameter.assert_called_once()              # AAA never sent
        with cache._state_lock:
            stale_names = cache._stale_names_by_sys_id.get(42, set())
        self.assertIn("AAB", stale_names)
        self.assertNotIn("AAA", stale_names)

    async def test_unsupported_ap_type_per_cell_error_no_stale(self):
        """Defensive branch: a record whose AP type has no wire-type mapping
        yields a per-cell error and is not sent or staled."""
        cache = FullParamCache()
        v = _stub_vehicle(42)
        snap = _build_snapshot(target_system=42)
        snap.by_name["WEIRD"] = ParamRecord(
            name="WEIRD", value=1, ap_type=9, flags=0,
            default=None, default_known=False)
        cache._snapshots[42] = snap
        result = await cache.write_batch(42, v, [ParamChange(name="WEIRD", value=1)])
        v.set_parameter.assert_not_called()
        self.assertFalse(result.results["WEIRD"].ok)
        self.assertIn("unsupported", result.results["WEIRD"].error.lower())
        self.assertFalse(result.snapshot_stale)

    async def test_repeated_write_uses_stale_snapshot_metadata(self):
        cache = FullParamCache()
        v = _stub_vehicle(42)
        v.set_parameter.return_value = True
        cache._snapshots[42] = _build_snapshot(target_system=42)

        first = await cache.write_batch(
            42, v, [ParamChange(name="AAB", value=2.0)])
        second = await cache.write_batch(
            42, v, [ParamChange(name="AAB", value=3.0)])

        self.assertTrue(first.results["AAB"].ok)
        self.assertTrue(second.results["AAB"].ok)
        self.assertTrue(cache.is_stale(42))
        self.assertEqual(v.set_parameter.call_count, 2)

    async def test_explicit_invalidate_drops_stale_snapshot_metadata(self):
        cache = FullParamCache()
        cache._snapshots[42] = _build_snapshot(target_system=42)
        cache.invalidate_keys(42, ["AAB"])
        self.assertTrue(cache.is_stale(42))

        cache.invalidate(42)

        self.assertIsNone(cache.peek(42))
        self.assertFalse(cache.is_stale(42))
        self.assertNotIn(42, cache._stale_names_by_sys_id)


if __name__ == "__main__":
    unittest.main()
