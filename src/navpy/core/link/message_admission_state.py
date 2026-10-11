"""Stateful collaborators for message admission.

The message filter owns admission order and metrics.  This module owns the two
thread-safe time-based stores used by that pipeline: logical-message
deduplication and per-peer clock-offset estimation.
"""

import threading
import time
from collections import OrderedDict
from typing import Dict, Optional, Tuple


# Type alias for message UID: (sender_id, boot_id, msg_seq)
MsgUID = Tuple[int, int, int]


class DedupCache:
    """
    LRU cache with time-based expiry for message deduplication.

    Tracks seen message UIDs (sender_id, boot_id, msg_seq) to prevent
    processing duplicate messages.

    Thread-safe.

    Attributes:
        max_size: Maximum number of entries (default 3000)
        expiry_ms: Time after which entries expire (default 15000ms)
    """

    def __init__(self, max_size: int = 3000, expiry_ms: int = 15000) -> None:
        """
        Initialize dedup cache.

        Args:
            max_size: Maximum entries before LRU eviction (2000-5000 recommended)
            expiry_ms: Expiry window in ms (10000-20000 recommended)
        """
        self._max_size = max_size
        self._expiry_ms = expiry_ms
        self._cache: OrderedDict[MsgUID, int] = OrderedDict()  # UID -> timestamp_ms
        self._lock = threading.Lock()
        self._check_count = 0  # Counter for periodic cleanup

    def is_duplicate(self, uid: MsgUID) -> bool:
        """
        Check if message UID has been seen before.

        If not seen, adds to cache and returns False.
        If seen and not expired, returns True (duplicate).
        If seen but expired, updates timestamp and returns False.

        Thread-safe.

        Args:
            uid: Message unique identifier (sender_id, boot_id, msg_seq)

        Returns:
            True if this is a duplicate message, False otherwise
        """
        now_ms = int(time.time() * 1000)

        with self._lock:
            # Clean expired entries periodically (every ~100 checks)
            self._check_count += 1
            if self._check_count >= 100:
                self._check_count = 0
                self._cleanup_expired(now_ms)

            if uid in self._cache:
                seen_at = self._cache[uid]
                age_ms = now_ms - seen_at

                if age_ms < self._expiry_ms:
                    # Still valid, this is a duplicate
                    return True

                # Expired, treat as new and update timestamp
                self._cache.move_to_end(uid)
                self._cache[uid] = now_ms
                return False

            # New message, add to cache
            self._cache[uid] = now_ms

            # LRU eviction if over capacity
            while len(self._cache) > self._max_size:
                self._cache.popitem(last=False)

            return False

    def _cleanup_expired(self, now_ms: int) -> None:
        """Remove expired entries. Must be called with lock held."""
        expired = []
        for uid, seen_at in self._cache.items():
            if now_ms - seen_at >= self._expiry_ms:
                expired.append(uid)
        for uid in expired:
            del self._cache[uid]

    def clear(self) -> None:
        """Clear all cached entries."""
        with self._lock:
            self._cache.clear()

    def size(self) -> int:
        """Get current cache size."""
        with self._lock:
            return len(self._cache)


class ClockOffsetEstimator:
    """
    Estimate clock offset for each peer using an exponential moving average.

    Updated on each heartbeat to compute ``local_time - remote_time``.  The
    estimate lets the admission pipeline evaluate TTL without requiring
    perfectly synchronized clocks.  Samples that deviate too far from an
    existing estimate are rejected to guard against clock jumps.
    """

    # Reject samples that deviate more than 5s from current estimate
    MAX_SAMPLE_DEVIATION_MS = 5000

    def __init__(self, alpha: float = 0.1) -> None:
        """
        Initialize clock offset estimator.

        Args:
            alpha: EMA smoothing factor (0.1 = slow adaptation, 0.5 = fast)
        """
        self._alpha = alpha
        self._offsets: Dict[int, float] = {}  # sender_id -> offset_ms
        self._lock = threading.Lock()

    def update_offset(self, sender_id: int, remote_time_ms: int) -> float:
        """
        Update the clock offset estimate for a peer from a heartbeat.

        The first sample is always accepted.  Later samples that deviate more
        than ``MAX_SAMPLE_DEVIATION_MS`` leave the estimate unchanged.
        """
        now_ms = int(time.time() * 1000)
        sample = now_ms - remote_time_ms

        with self._lock:
            if sender_id in self._offsets:
                old_offset = self._offsets[sender_id]
                # Reject outlier: NTP jump or clock correction
                if abs(sample - old_offset) > self.MAX_SAMPLE_DEVIATION_MS:
                    return old_offset
                # EMA update: offset = (1-alpha)*old + alpha*sample
                new_offset = (1 - self._alpha) * old_offset + self._alpha * sample
            else:
                # First sample always accepted
                new_offset = float(sample)

            self._offsets[sender_id] = new_offset
            return new_offset

    def get_offset(self, sender_id: int) -> Optional[float]:
        """Return a peer's offset in milliseconds, or ``None`` if unknown."""
        with self._lock:
            return self._offsets.get(sender_id)

    def adjust_time(self, sender_id: int, remote_time_ms: int) -> int:
        """Adjust a remote timestamp into the local clock domain."""
        offset = self.get_offset(sender_id)
        if offset is None:
            # No offset known, assume clocks are synchronized
            return remote_time_ms
        return int(remote_time_ms + offset)

    def clear(self) -> None:
        """Clear all offset estimates."""
        with self._lock:
            self._offsets.clear()

    def get_all_offsets(self) -> Dict[int, float]:
        """Get snapshot of all peer offsets."""
        with self._lock:
            return dict(self._offsets)


__all__ = ["ClockOffsetEstimator", "DedupCache", "MsgUID"]
