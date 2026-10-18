"""
Unit tests for message filtering (dedup + TTL).

Tests:
- DedupCache: LRU + expiry behavior
- MessageFilter: Integration of dedup + TTL checks
- MsgMeta: Metadata serialization
- MsgMetaProvider: Sequence generation
"""

import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from navpy.core.link.message_filter import (
    DedupCache,
    MessageFilter,
    FilterMetrics,
)
from navpy.core.link.task_confirmation_msg import TaskConfirmResponseMsg
from navpy.core.link.msg_meta import MsgMeta, MsgMetaProvider
from navpy.core.link.types import MsgType


class TestMsgMeta(unittest.TestCase):
    """Tests for MsgMeta dataclass."""

    def test_to_dict(self):
        """Test serialization to dictionary."""
        meta = MsgMeta(boot_id=12345, msg_seq=10, time_ms=1000000, ttl_ms=5000)
        d = meta.to_dict()

        self.assertEqual(d["bid"], 12345)
        self.assertEqual(d["seq"], 10)
        self.assertEqual(d["tms"], 1000000)
        self.assertEqual(d["ttl"], 5000)

    def test_from_dict(self):
        """Test deserialization from dictionary."""
        d = {"bid": 12345, "seq": 10, "tms": 1000000, "ttl": 5000}
        meta = MsgMeta.from_dict(d)

        self.assertEqual(meta.boot_id, 12345)
        self.assertEqual(meta.msg_seq, 10)
        self.assertEqual(meta.time_ms, 1000000)
        self.assertEqual(meta.ttl_ms, 5000)

    def test_from_dict_missing_fields_returns_none(self):
        """Test backward compatibility - returns None if required fields missing."""
        # Missing boot_id
        self.assertIsNone(MsgMeta.from_dict({"seq": 10}))
        # Missing msg_seq
        self.assertIsNone(MsgMeta.from_dict({"bid": 12345}))
        # Empty dict
        self.assertIsNone(MsgMeta.from_dict({}))

    def test_from_dict_optional_fields_default(self):
        """Test optional fields default to 0."""
        d = {"bid": 12345, "seq": 10}
        meta = MsgMeta.from_dict(d)

        self.assertEqual(meta.time_ms, 0)
        self.assertEqual(meta.ttl_ms, 0)

    def test_msg_uid(self):
        """Test msg_uid property."""
        meta = MsgMeta(boot_id=12345, msg_seq=10, time_ms=0, ttl_ms=0)
        self.assertEqual(meta.msg_uid, (12345, 10))

    def test_get_full_uid(self):
        """Test full UID with sender_id."""
        meta = MsgMeta(boot_id=12345, msg_seq=10, time_ms=0, ttl_ms=0)
        self.assertEqual(meta.get_full_uid(1), (1, 12345, 10))


class TestMsgMetaProvider(unittest.TestCase):
    """Tests for MsgMetaProvider singleton."""

    def setUp(self):
        MsgMetaProvider.reset_instance()

    def tearDown(self):
        MsgMetaProvider.reset_instance()

    def test_singleton(self):
        """Test that get_instance returns same instance."""
        p1 = MsgMetaProvider.get_instance()
        p2 = MsgMetaProvider.get_instance()
        self.assertIs(p1, p2)

    def test_boot_id_is_random(self):
        """Test that boot_id is set on creation."""
        provider = MsgMetaProvider.get_instance()
        self.assertIsInstance(provider.boot_id, int)
        self.assertGreaterEqual(provider.boot_id, 0)
        self.assertLessEqual(provider.boot_id, 0xFFFFFFFF)

    def test_msg_seq_increments(self):
        """Test that msg_seq increments with each call."""
        provider = MsgMetaProvider.get_instance()

        meta1 = provider.create_meta(ttl_ms=5000)
        meta2 = provider.create_meta(ttl_ms=5000)
        meta3 = provider.create_meta(ttl_ms=5000)

        self.assertEqual(meta1.msg_seq, 1)
        self.assertEqual(meta2.msg_seq, 2)
        self.assertEqual(meta3.msg_seq, 3)

    def test_boot_id_consistent(self):
        """Test that boot_id is same for all messages."""
        provider = MsgMetaProvider.get_instance()

        meta1 = provider.create_meta(ttl_ms=5000)
        meta2 = provider.create_meta(ttl_ms=5000)

        self.assertEqual(meta1.boot_id, meta2.boot_id)

    def test_time_ms_is_set(self):
        """Test that time_ms is set to current time."""
        provider = MsgMetaProvider.get_instance()

        before_ms = int(time.time() * 1000)
        meta = provider.create_meta(ttl_ms=5000)
        after_ms = int(time.time() * 1000)

        self.assertGreaterEqual(meta.time_ms, before_ms)
        self.assertLessEqual(meta.time_ms, after_ms)

    def test_thread_safety(self):
        """Test that sequence numbers are unique across threads."""
        provider = MsgMetaProvider.get_instance()
        results = []
        lock = threading.Lock()

        def create_metas():
            for _ in range(100):
                meta = provider.create_meta(ttl_ms=5000)
                with lock:
                    results.append(meta.msg_seq)

        threads = [threading.Thread(target=create_metas) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # All sequence numbers should be unique
        self.assertEqual(len(results), len(set(results)))


class TestDedupCache(unittest.TestCase):
    """Tests for DedupCache."""

    def test_first_message_not_duplicate(self):
        """First message with a UID should not be a duplicate."""
        cache = DedupCache()
        uid = (1, 12345, 1)

        self.assertFalse(cache.is_duplicate(uid))

    def test_same_uid_is_duplicate(self):
        """Same UID sent twice should be detected as duplicate."""
        cache = DedupCache()
        uid = (1, 12345, 1)

        self.assertFalse(cache.is_duplicate(uid))  # First time
        self.assertTrue(cache.is_duplicate(uid))   # Duplicate

    def test_different_uid_not_duplicate(self):
        """Different UIDs should not be duplicates of each other."""
        cache = DedupCache()
        uid1 = (1, 12345, 1)
        uid2 = (1, 12345, 2)  # Different msg_seq
        uid3 = (2, 12345, 1)  # Different sender_id

        self.assertFalse(cache.is_duplicate(uid1))
        self.assertFalse(cache.is_duplicate(uid2))
        self.assertFalse(cache.is_duplicate(uid3))

    def test_lru_eviction(self):
        """Test that oldest entries are evicted when cache is full."""
        cache = DedupCache(max_size=3, expiry_ms=60000)

        uid1 = (1, 100, 1)
        uid2 = (1, 100, 2)
        uid3 = (1, 100, 3)
        uid4 = (1, 100, 4)

        cache.is_duplicate(uid1)
        cache.is_duplicate(uid2)
        cache.is_duplicate(uid3)

        self.assertEqual(cache.size(), 3)

        # Adding a 4th should evict the oldest (uid1)
        cache.is_duplicate(uid4)

        # uid1 should no longer be in cache (evicted), so not duplicate
        self.assertFalse(cache.is_duplicate(uid1))

    def test_expiry(self):
        """Test that expired entries are not considered duplicates."""
        cache = DedupCache(max_size=100, expiry_ms=100)  # 100ms expiry

        uid = (1, 12345, 1)

        self.assertFalse(cache.is_duplicate(uid))  # First time
        self.assertTrue(cache.is_duplicate(uid))   # Still valid

        # Wait for expiry
        time.sleep(0.15)

        # Should not be duplicate anymore (expired)
        self.assertFalse(cache.is_duplicate(uid))

    def test_clear(self):
        """Test clearing the cache."""
        cache = DedupCache()

        cache.is_duplicate((1, 100, 1))
        cache.is_duplicate((1, 100, 2))

        self.assertEqual(cache.size(), 2)

        cache.clear()

        self.assertEqual(cache.size(), 0)

    def test_thread_safety(self):
        """Test thread safety of dedup cache."""
        cache = DedupCache(max_size=1000, expiry_ms=60000)
        duplicates_found = []
        lock = threading.Lock()

        def check_uids(thread_id):
            for i in range(100):
                uid = (thread_id, 12345, i)
                result = cache.is_duplicate(uid)
                with lock:
                    duplicates_found.append(result)

        threads = [threading.Thread(target=check_uids, args=(i,)) for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # No duplicates should have been found (all unique UIDs)
        self.assertFalse(any(duplicates_found))


class TestFilterMetrics(unittest.TestCase):
    """Tests for FilterMetrics."""

    def test_initial_values(self):
        """Test that all counters start at zero."""
        metrics = FilterMetrics()

        stats = metrics.get_stats()
        self.assertEqual(stats["rx_total"], 0)
        self.assertEqual(stats["rx_dropped_duplicate"], 0)
        self.assertEqual(stats["rx_dropped_expired"], 0)
        self.assertEqual(stats["rx_dropped_decode_error"], 0)
        self.assertEqual(stats["rx_passed"], 0)

    def test_increment_total(self):
        """Test incrementing total counter."""
        metrics = FilterMetrics()

        metrics.increment_total()
        metrics.increment_total()

        self.assertEqual(metrics.get_stats()["rx_total"], 2)

    def test_increment_all_counters(self):
        """Test incrementing all counters."""
        metrics = FilterMetrics()

        metrics.increment_total()
        metrics.increment_duplicate()
        metrics.increment_expired()
        metrics.increment_decode_error()
        metrics.increment_passed()

        stats = metrics.get_stats()
        self.assertEqual(stats["rx_total"], 1)
        self.assertEqual(stats["rx_dropped_duplicate"], 1)
        self.assertEqual(stats["rx_dropped_expired"], 1)
        self.assertEqual(stats["rx_dropped_decode_error"], 1)
        self.assertEqual(stats["rx_passed"], 1)

    def test_reset(self):
        """Test resetting all counters."""
        metrics = FilterMetrics()

        metrics.increment_total()
        metrics.increment_total()
        metrics.reset()

        self.assertEqual(metrics.get_stats()["rx_total"], 0)

    def test_thread_safety(self):
        """Test thread safety of metrics."""
        metrics = FilterMetrics()

        def increment_many():
            for _ in range(1000):
                metrics.increment_total()

        threads = [threading.Thread(target=increment_many) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(metrics.get_stats()["rx_total"], 10000)


class TestMessageFilter(unittest.TestCase):
    """Tests for MessageFilter integration."""

    def test_confirmation_response_dedup_keeps_later_recall_for_same_round(self):
        message_filter = MessageFilter(enable_ttl=False)
        meta = MsgMeta(boot_id=1234, msg_seq=9, time_ms=1000, ttl_ms=5000)
        approve = TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=77,
            is_confirmed=True,
            meta=meta,
        )
        recall = TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=77,
            is_confirmed=False,
            meta=MsgMeta(1234, 9, 1001, 5000),
        )

        self.assertTrue(message_filter.should_process(approve))
        self.assertFalse(message_filter.should_process(approve))
        self.assertTrue(message_filter.should_process(recall))
        self.assertFalse(message_filter.should_process(recall))

    def _create_mock_message(self, sender_id=1, msg_type=MsgType.CHECK_IN,
                              boot_id=12345, msg_seq=1, time_ms=None, ttl_ms=5000):
        """Create a mock message with metadata."""
        msg = MagicMock()
        msg.sender_id = sender_id
        msg.receiver_id = None
        msg.msg_type.return_value = msg_type

        if time_ms is None:
            time_ms = int(time.time() * 1000)

        meta = MsgMeta(boot_id=boot_id, msg_seq=msg_seq, time_ms=time_ms, ttl_ms=ttl_ms)
        msg.meta = meta
        msg.has_meta.return_value = True
        msg.get_msg_uid.return_value = meta.get_full_uid(sender_id)

        return msg

    def test_first_message_passes(self):
        """First message should pass all filters."""
        filter = MessageFilter()
        msg = self._create_mock_message()

        self.assertTrue(filter.should_process(msg))
        self.assertEqual(filter.metrics.get_stats()["rx_passed"], 1)

    def test_duplicate_message_dropped(self):
        """Duplicate message should be dropped."""
        filter = MessageFilter()
        msg1 = self._create_mock_message(msg_seq=1)
        msg2 = self._create_mock_message(msg_seq=1)  # Same msg_seq

        self.assertTrue(filter.should_process(msg1))
        self.assertFalse(filter.should_process(msg2))

        stats = filter.metrics.get_stats()
        self.assertEqual(stats["rx_passed"], 1)
        self.assertEqual(stats["rx_dropped_duplicate"], 1)

    def test_different_seq_not_duplicate(self):
        """Messages with different seq should both pass."""
        filter = MessageFilter()
        msg1 = self._create_mock_message(msg_seq=1)
        msg2 = self._create_mock_message(msg_seq=2)

        self.assertTrue(filter.should_process(msg1))
        self.assertTrue(filter.should_process(msg2))

        self.assertEqual(filter.metrics.get_stats()["rx_passed"], 2)

    def test_expired_message_dropped(self):
        """Expired message should be dropped."""
        filter = MessageFilter()

        now_ms = int(time.time() * 1000)

        # Message created 2 seconds ago with 1 second TTL
        old_time_ms = now_ms - 2000
        msg = self._create_mock_message(time_ms=old_time_ms, ttl_ms=1000)

        self.assertFalse(filter.should_process(msg))

        stats = filter.metrics.get_stats()
        self.assertEqual(stats["rx_dropped_expired"], 1)

    def test_valid_ttl_message_passes(self):
        """Message within TTL should pass."""
        filter = MessageFilter()

        # Message created 500ms ago with 5 second TTL
        recent_time_ms = int(time.time() * 1000) - 500
        msg = self._create_mock_message(time_ms=recent_time_ms, ttl_ms=5000)

        self.assertTrue(filter.should_process(msg))

    def test_no_meta_permissive_mode_passes(self):
        """Message without meta should pass in permissive mode."""
        filter = MessageFilter(strict_ttl=False)

        msg = MagicMock()
        msg.has_meta.return_value = False
        msg.msg_type.return_value = MsgType.CHECK_IN

        self.assertTrue(filter.should_process(msg))

    def test_no_meta_strict_mode_dropped(self):
        """Message without meta should be dropped in strict mode."""
        filter = MessageFilter(strict_ttl=True)

        msg = MagicMock()
        msg.has_meta.return_value = False
        msg.msg_type.return_value = MsgType.CHECK_IN

        self.assertFalse(filter.should_process(msg))

    def test_far_future_message_dropped(self):
        """A stamp ahead of the local clock beyond the sync tolerance is dropped."""
        filter = MessageFilter()
        future_time_ms = int(time.time() * 1000) + 120000
        msg = self._create_mock_message(time_ms=future_time_ms, ttl_ms=5000)

        self.assertFalse(filter.should_process(msg))
        self.assertEqual(filter.metrics.get_stats()["rx_dropped_expired"], 1)

    def test_clock_ahead_within_tolerance_passes_beyond_is_dropped(self):
        """Synchronised clocks may differ slightly; a peer clock further ahead
        than the tolerance is not synchronised, and is warned about once."""
        logger = MagicMock()
        filter = MessageFilter(logger=logger)
        tolerance = MessageFilter.CLOCK_SYNC_TOLERANCE_MS
        with patch("navpy.core.link.message_filter.rt.time", return_value=1000.0):
            near = self._create_mock_message(msg_seq=1, time_ms=1_000_000 + tolerance - 100)
            far = self._create_mock_message(msg_seq=2, time_ms=1_000_000 + tolerance + 100)
            far_again = self._create_mock_message(msg_seq=3, time_ms=1_000_000 + tolerance + 100)
            self.assertTrue(filter.should_process(near))
            self.assertFalse(filter.should_process(far))
            self.assertFalse(filter.should_process(far_again))
        self.assertEqual(logger.warning.call_count, 1)
        self.assertIn("not synchronised", logger.warning.call_args.args[0])

    def test_stale_heartbeat_is_dropped(self):
        """Heartbeats are aged like any other message (no clock reference)."""
        filter = MessageFilter()
        stale_ms = int(time.time() * 1000) - 6000
        hb = self._create_mock_message(
            sender_id=3, msg_type=MsgType.SWARM_HEARTBEAT, time_ms=stale_ms, ttl_ms=5000,
        )
        self.assertFalse(filter.should_process(hb))
        self.assertEqual(filter.metrics.get_stats()["rx_dropped_expired"], 1)

    def test_inbound_backlog_shows_as_age(self):
        """Regression (.logs/base-wall-1): with the receiver's reader backed up,
        heartbeats arrive late too, the backlog growing to 6 s and staying. A
        6.5 s-old bid must be dropped and a fresh ack admitted, instead of the
        backlog being taken for clock skew."""
        filter = MessageFilter()
        now_ms = 1_000_000
        backlog_ms = [*range(0, 6000, 200), *[6000] * 40]
        with patch("navpy.core.link.message_filter.rt.time", return_value=now_ms / 1000):
            for seq, delay_ms in enumerate(backlog_ms, start=1):
                filter.should_process(self._create_mock_message(
                    sender_id=2, msg_type=MsgType.SWARM_HEARTBEAT, msg_seq=seq,
                    time_ms=now_ms - delay_ms, ttl_ms=5000))
            stale_bid = self._create_mock_message(sender_id=2, msg_seq=100, time_ms=now_ms - 6500, ttl_ms=5000)
            fresh_ack = self._create_mock_message(sender_id=2, msg_seq=101, time_ms=now_ms - 30, ttl_ms=5000)
            self.assertFalse(filter.should_process(stale_bid))
            self.assertTrue(filter.should_process(fresh_ack))

    def test_zero_ttl_skips_ttl_check(self):
        """Message with ttl_ms=0 should skip TTL check."""
        filter = MessageFilter()

        # Very old message but with 0 TTL (no expiry)
        old_time_ms = int(time.time() * 1000) - 1000000  # Very old
        msg = self._create_mock_message(time_ms=old_time_ms, ttl_ms=0)

        self.assertTrue(filter.should_process(msg))

    def test_reset_stats(self):
        """Test resetting filter stats."""
        filter = MessageFilter()
        msg = self._create_mock_message()

        filter.should_process(msg)
        self.assertEqual(filter.metrics.get_stats()["rx_total"], 1)

        filter.reset_stats()
        self.assertEqual(filter.metrics.get_stats()["rx_total"], 0)

    def test_zero_meta_passes_permissive(self):
        """Messages with boot_id=0, msg_seq=0 should pass in permissive mode (legacy)."""
        filter = MessageFilter(strict_ttl=False)

        # Two messages with zero meta (from MAVLink roundtrip without proper meta)
        msg1 = MagicMock()
        msg1.has_meta.return_value = True
        msg1.meta = MsgMeta(boot_id=0, msg_seq=0, time_ms=0, ttl_ms=0)
        msg1.msg_type.return_value = MsgType.CHECK_IN

        msg2 = MagicMock()
        msg2.has_meta.return_value = True
        msg2.meta = MsgMeta(boot_id=0, msg_seq=0, time_ms=0, ttl_ms=0)
        msg2.msg_type.return_value = MsgType.CHECK_IN

        # Both should pass (not treated as duplicates)
        self.assertTrue(filter.should_process(msg1))
        self.assertTrue(filter.should_process(msg2))

        stats = filter.metrics.get_stats()
        self.assertEqual(stats["rx_passed"], 2)
        self.assertEqual(stats["rx_dropped_duplicate"], 0)

    def test_zero_meta_dropped_strict(self):
        """Messages with boot_id=0, msg_seq=0 should be dropped in strict mode."""
        filter = MessageFilter(strict_ttl=True)

        msg = MagicMock()
        msg.has_meta.return_value = True
        msg.meta = MsgMeta(boot_id=0, msg_seq=0, time_ms=0, ttl_ms=0)
        msg.msg_type.return_value = MsgType.CHECK_IN

        self.assertFalse(filter.should_process(msg))

    def test_ttl_enforced_before_any_heartbeat(self):
        """A stale message is dropped even from a peer never heard before."""
        filter = MessageFilter()
        old_time_ms = int(time.time() * 1000) - 10000
        msg = self._create_mock_message(
            sender_id=99, msg_seq=1, time_ms=old_time_ms, ttl_ms=1000,
        )

        self.assertFalse(filter.should_process(msg))
        self.assertEqual(filter.metrics.get_stats()["rx_dropped_expired"], 1)


if __name__ == "__main__":
    unittest.main()
