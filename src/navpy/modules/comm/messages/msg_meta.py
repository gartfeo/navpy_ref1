"""
Message metadata for deduplication and time validity (TTL).

Provides:
- MsgMeta: dataclass containing boot_id, msg_seq, time_ms, ttl_ms
- MsgMetaProvider: singleton that generates unique message IDs per sender
"""

import random
import time
import threading
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple


@dataclass
class MsgMeta:
    """
    Metadata fields for swarm message deduplication and TTL.

    Attributes:
        boot_id: Random 32-bit ID generated on process start, identifies sender instance
        msg_seq: Monotonically increasing counter per sender, unique per message
        time_ms: Sender timestamp in milliseconds when message was created
        ttl_ms: Validity window in milliseconds (how long message is valid)
    """
    boot_id: int
    msg_seq: int
    time_ms: int
    ttl_ms: int

    def to_dict(self) -> Dict[str, Any]:
        """Serialize metadata to dictionary."""
        return {
            "bid": self.boot_id,
            "seq": self.msg_seq,
            "tms": self.time_ms,
            "ttl": self.ttl_ms,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> Optional["MsgMeta"]:
        """
        Deserialize metadata from dictionary.

        Returns None if required fields are missing (backward compatibility).
        """
        # Check for required fields - return None if missing for backward compat
        if "bid" not in data or "seq" not in data:
            return None
        return cls(
            boot_id=data["bid"],
            msg_seq=data["seq"],
            time_ms=data.get("tms", 0),
            ttl_ms=data.get("ttl", 0),
        )

    @property
    def msg_uid(self) -> Tuple[int, int]:
        """
        Returns unique identifier tuple (boot_id, msg_seq).

        Note: sender_id should be combined externally to form full UID:
        (sender_sysid, boot_id, msg_seq)
        """
        return (self.boot_id, self.msg_seq)

    def get_full_uid(self, sender_id: int) -> Tuple[int, int, int]:
        """
        Returns full unique identifier: (sender_sysid, boot_id, msg_seq).
        """
        return (sender_id, self.boot_id, self.msg_seq)


class MsgMetaProvider:
    """
    Provides message metadata (boot_id, msg_seq) for outgoing messages.

    Thread-safe singleton that generates unique message identifiers.

    Usage:
        provider = MsgMetaProvider.get_instance()
        meta = provider.create_meta(ttl_ms=5000)
    """

    _instance: Optional["MsgMetaProvider"] = None
    _lock = threading.Lock()

    def __init__(self):
        """Initialize with random boot_id and sequence counter at 0."""
        self._boot_id: int = random.randint(0, 0xFFFFFFFF)
        self._msg_seq: int = 0
        self._seq_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "MsgMetaProvider":
        """Get or create the singleton instance."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """Reset singleton (for testing only)."""
        with cls._lock:
            cls._instance = None

    @property
    def boot_id(self) -> int:
        """Returns the boot_id for this process instance."""
        return self._boot_id

    def _next_seq(self) -> int:
        """Thread-safe increment and return of sequence number."""
        with self._seq_lock:
            self._msg_seq += 1
            return self._msg_seq

    def create_meta(self, ttl_ms: int) -> MsgMeta:
        """
        Create metadata for an outgoing message.

        Args:
            ttl_ms: Time-to-live in milliseconds

        Returns:
            MsgMeta with current boot_id, next sequence number,
            current time, and specified TTL
        """
        return MsgMeta(
            boot_id=self._boot_id,
            msg_seq=self._next_seq(),
            time_ms=int(time.time() * 1000),
            ttl_ms=ttl_ms,
        )

    def create_meta_for_type(self, msg_type: "MsgType") -> MsgMeta:
        """
        Create metadata using TTL from defaults table for the message type.

        Args:
            msg_type: The message type to look up TTL for

        Returns:
            MsgMeta with appropriate TTL for the message type
        """
        from navpy.modules.comm.messages.ttl_defaults import TTL_DEFAULTS, DEFAULT_TTL_MS
        ttl_ms = TTL_DEFAULTS.get(msg_type, DEFAULT_TTL_MS)
        return self.create_meta(ttl_ms)
