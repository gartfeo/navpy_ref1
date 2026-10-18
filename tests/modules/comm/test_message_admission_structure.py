"""Regression tests for the message-admission composition boundary."""

from navpy.core.link.message_admission_state import DedupCache
from navpy.core.link.message_filter import DedupCache as FacadeDedupCache
from navpy.core.link.message_filter import MessageFilter
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.swarm_heartbeat_msg import SwarmHeartbeatMsg


def test_legacy_and_package_exports_reference_the_state_owners() -> None:
    assert FacadeDedupCache is DedupCache


def test_duplicate_rejection_precedes_the_ttl_check() -> None:
    events: list[str] = []

    class RecordingDedup:
        def is_duplicate(self, uid: tuple[int, int, int]) -> bool:
            events.append("dedup")
            return True

    # time_ms=30 is long expired: only the duplicate counter may move.
    message_filter = MessageFilter(dedup_cache=RecordingDedup())
    heartbeat = SwarmHeartbeatMsg(
        sender_id=7,
        meta=MsgMeta(
            boot_id=10,
            msg_seq=20,
            time_ms=30,
            ttl_ms=5000,
        ),
    )

    assert message_filter.should_process(heartbeat) is False
    assert events == ["dedup"]
    assert message_filter.metrics.get_stats() == {
        "rx_total": 1,
        "rx_dropped_duplicate": 1,
        "rx_dropped_expired": 0,
        "rx_dropped_decode_error": 0,
        "rx_passed": 0,
    }
