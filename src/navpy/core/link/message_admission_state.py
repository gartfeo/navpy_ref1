"""Stateful collaborators for message admission.

The message filter owns admission order and metrics.  This module owns the
thread-safe time-based store used by that pipeline: logical-message
deduplication.
"""

from collections import OrderedDict
from typing import Tuple

from navpy.core.kernel import runtime_time as rt


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
        self._lock = rt.Lock()
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
        now_ms = int(rt.time() * 1000)

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


__all__ = ["DedupCache", "MsgUID"]
