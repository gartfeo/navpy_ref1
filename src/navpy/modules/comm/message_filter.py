"""
Message filtering for deduplication and TTL enforcement.

Provides central enforcement of:
1. Deduplication: Process each logical message once
2. Time validity (TTL): Drop stale messages

Components:
- DedupCache: LRU cache with expiry for tracking seen message UIDs
- ClockOffsetEstimator: Per-peer clock offset estimation using EMA
- FilterMetrics: Counters for monitoring
- MessageFilter: Main filter integrating all checks
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, TYPE_CHECKING

from navpy.modules.comm.message_admission_state import (
    ClockOffsetEstimator,
    DedupCache,
    MsgUID,
)

if TYPE_CHECKING:
    from navpy.modules.comm.messages.msg_abc import MsgABC
    from navpy.modules.comm.messages.msg_meta import MsgMeta
    from navpy.logger.cache_logger import ILogger

@dataclass
class FilterMetrics:
    """
    Counters for message filtering statistics.

    Thread-safe via lock.
    """
    rx_total: int = 0
    rx_dropped_duplicate: int = 0
    rx_dropped_expired: int = 0
    rx_dropped_decode_error: int = 0
    rx_passed: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def increment_total(self) -> None:
        with self._lock:
            self.rx_total += 1

    def increment_duplicate(self) -> None:
        with self._lock:
            self.rx_dropped_duplicate += 1

    def increment_expired(self) -> None:
        with self._lock:
            self.rx_dropped_expired += 1

    def increment_decode_error(self) -> None:
        with self._lock:
            self.rx_dropped_decode_error += 1

    def increment_passed(self) -> None:
        with self._lock:
            self.rx_passed += 1

    def get_stats(self) -> Dict[str, int]:
        """Get snapshot of current statistics."""
        with self._lock:
            return {
                "rx_total": self.rx_total,
                "rx_dropped_duplicate": self.rx_dropped_duplicate,
                "rx_dropped_expired": self.rx_dropped_expired,
                "rx_dropped_decode_error": self.rx_dropped_decode_error,
                "rx_passed": self.rx_passed,
            }

    def reset(self) -> None:
        """Reset all counters."""
        with self._lock:
            self.rx_total = 0
            self.rx_dropped_duplicate = 0
            self.rx_dropped_expired = 0
            self.rx_dropped_decode_error = 0
            self.rx_passed = 0

class MessageFilter:
    """
    Central message filter for deduplication and TTL enforcement.

    Integrates:
    - DedupCache for duplicate detection
    - ClockOffsetEstimator for time adjustment
    - TTL checking using adjusted timestamps
    - Metrics collection

    Pipeline order:
    1. Check if message has metadata (backward compat: pass if no meta)
    2. Dedup check (drop if duplicate)
    3. TTL check (drop if expired)
    4. Pass to listeners

    Thread-safe.
    """

    # Maximum allowed clock skew before rejecting (60 seconds into future)
    MAX_FUTURE_MS = 60000

    def __init__(
        self,
        dedup_cache: Optional[DedupCache] = None,
        offset_estimator: Optional[ClockOffsetEstimator] = None,
        logger: Optional["ILogger"] = None,
        strict_ttl: bool = False,
        enable_ttl: bool = True,
        enable_dedup: bool = True,
    ):
        """
        Initialize message filter.

        Args:
            dedup_cache: Custom DedupCache, or None to create default
            offset_estimator: Custom ClockOffsetEstimator, or None to create default
            logger: Logger for debug output
            strict_ttl: If True, drop messages without metadata. If False, pass them.
        """
        self._dedup = dedup_cache or DedupCache()
        self._offset = offset_estimator or ClockOffsetEstimator()
        self._logger = logger
        self._strict_ttl = strict_ttl
        self._enable_ttl = enable_ttl
        self._enable_dedup = enable_dedup
        self.metrics = FilterMetrics()

    def _is_valid_meta(self, msg: "MsgABC") -> bool:
        """
        Check if message has valid (non-zero) metadata for dedup/TTL.

        Messages from old senders or after MAVLink roundtrip without proper
        meta will have boot_id=0 and msg_seq=0, which should be treated
        as legacy messages without dedup/TTL.
        """
        if not msg.has_meta():
            return False
        # Treat boot_id=0 AND msg_seq=0 as invalid/legacy meta
        # (a real boot_id of 0 is astronomically unlikely: 1 in 4 billion)
        return msg.meta.boot_id != 0 or msg.meta.msg_seq != 0

    def should_process(self, msg: "MsgABC") -> bool:
        """Return whether a message passes metadata, dedup, and TTL checks."""
        from navpy.modules.comm.messages.types import MsgType

        self.metrics.increment_total()

        if not self._is_valid_meta(msg):
            return self._handle_invalid_metadata(msg)

        meta = msg.meta
        if msg.msg_type() == MsgType.SWARM_HEARTBEAT:
            self._offset.update_offset(msg.sender_id, meta.time_ms)

        if self._drop_duplicate(msg):
            return False

        # Skip TTL when no offset is known for this peer (pre-heartbeat)
        if self._offset.get_offset(msg.sender_id) is None:
            return self._pass()

        # TTL check — skip for heartbeats (they ARE the clock reference)
        is_heartbeat = msg.msg_type() == MsgType.SWARM_HEARTBEAT
        if self._drop_for_ttl(
            msg,
            meta=meta,
            is_heartbeat=is_heartbeat,
        ):
            return False

        return self._pass()

    def _handle_invalid_metadata(self, msg: "MsgABC") -> bool:
        if self._strict_ttl and self._enable_ttl:
            if self._logger:
                self._logger.debug(
                    f"Dropping message without valid metadata: "
                    f"{msg.msg_type().name}"
                )
            return False
        return self._pass()

    def _drop_duplicate(self, msg: "MsgABC") -> bool:
        if not self._enable_dedup:
            return False
        uid = msg.get_msg_uid()
        if not uid or not self._dedup.is_duplicate(uid):
            return False

        self.metrics.increment_duplicate()
        if self._logger:
            self._logger.debug(
                f"Dropping duplicate message: "
                f"{msg.msg_type().name} from {msg.sender_id}"
            )
        return True

    def _drop_for_ttl(
        self,
        msg: "MsgABC",
        *,
        meta: "MsgMeta",
        is_heartbeat: bool,
    ) -> bool:
        if not self._enable_ttl or meta.ttl_ms <= 0 or is_heartbeat:
            return False

        now_ms = int(time.time() * 1000)
        adjusted_time = self._offset.adjust_time(msg.sender_id, meta.time_ms)
        age_ms = now_ms - adjusted_time
        if age_ms < -self.MAX_FUTURE_MS:
            if self._logger:
                self._logger.warning(
                    f"Dropping message from far future ({-age_ms}ms ahead): "
                    f"{msg.msg_type().name} from {msg.sender_id}"
                )
            self.metrics.increment_expired()
            return True

        if age_ms > meta.ttl_ms:
            self.metrics.increment_expired()
            if self._logger:
                self._logger.debug(
                    f"Dropping expired message (age={age_ms}ms, "
                    f"ttl={meta.ttl_ms}ms): "
                    f"{msg.msg_type().name} from {msg.sender_id}"
                )
            return True
        return False

    def _pass(self) -> bool:
        self.metrics.increment_passed()
        return True

    def log_stats(self) -> None:
        """Log current filter statistics."""
        if self._logger:
            stats = self.metrics.get_stats()
            self._logger.info(
                f"MessageFilter stats: total={stats['rx_total']}, "
                f"passed={stats['rx_passed']}, "
                f"dropped_dup={stats['rx_dropped_duplicate']}, "
                f"dropped_exp={stats['rx_dropped_expired']}"
            )

    def reset_stats(self) -> None:
        """Reset filter statistics."""
        self.metrics.reset()

    @property
    def dedup_cache(self) -> DedupCache:
        """Access the dedup cache."""
        return self._dedup

    @property
    def offset_estimator(self) -> ClockOffsetEstimator:
        """Access the offset estimator."""
        return self._offset

    @property
    def enable_ttl(self) -> bool:
        return self._enable_ttl

    @property
    def enable_dedup(self) -> bool:
        return self._enable_dedup

    def set_enable_ttl(self, enabled: bool) -> None:
        self._enable_ttl = enabled

    def set_enable_dedup(self, enabled: bool) -> None:
        self._enable_dedup = enabled
