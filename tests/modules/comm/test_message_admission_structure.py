"""Regression tests for the message-admission composition boundary."""

from navpy.modules.comm import (
    ClockOffsetEstimator as PackageClockOffsetEstimator,
)
from navpy.modules.comm import DedupCache as PackageDedupCache
from navpy.modules.comm.message_admission_state import (
    ClockOffsetEstimator,
    DedupCache,
)
from navpy.modules.comm.message_filter import (
    ClockOffsetEstimator as FacadeClockOffsetEstimator,
)
from navpy.modules.comm.message_filter import DedupCache as FacadeDedupCache
from navpy.modules.comm.message_filter import MessageFilter
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg


def test_legacy_and_package_exports_reference_the_state_owners() -> None:
    assert FacadeDedupCache is DedupCache
    assert PackageDedupCache is DedupCache
    assert FacadeClockOffsetEstimator is ClockOffsetEstimator
    assert PackageClockOffsetEstimator is ClockOffsetEstimator


def test_heartbeat_updates_clock_before_duplicate_rejection() -> None:
    events: list[str] = []

    class RecordingDedup:
        def is_duplicate(self, uid: tuple[int, int, int]) -> bool:
            events.append("dedup")
            return True

    class RecordingOffsets:
        def update_offset(self, sender_id: int, remote_time_ms: int) -> float:
            events.append("offset")
            return 0.0

        def get_offset(self, sender_id: int) -> float:
            raise AssertionError("duplicate rejection must precede TTL lookup")

    message_filter = MessageFilter(
        dedup_cache=RecordingDedup(),
        offset_estimator=RecordingOffsets(),
    )
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
    assert events == ["offset", "dedup"]
    assert message_filter.metrics.get_stats() == {
        "rx_total": 1,
        "rx_dropped_duplicate": 1,
        "rx_dropped_expired": 0,
        "rx_dropped_decode_error": 0,
        "rx_passed": 0,
    }
