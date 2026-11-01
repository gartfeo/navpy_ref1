"""NAV_FAIL reasons and their fixed display numbers (gcs_mission_phase.md).

Every producer passes one of these constants. The number travels in the
swarm heartbeat state byte for the GCS, so the table is append-only: never
renumber or reuse an index. 0 is any reason not in the table.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Optional

NAV_FAIL = "nav_fail"  # the caller names no reason
NO_DETECTION = "no_detection"
MISSED_DOCK = "missed_dock"
NAVIGATION_REJECTED = "navigation_rejected"
SOURCE_COMMAND_STALE = "source_command_stale"
DISCONTINUITY_SOURCE_MISSING = "discontinuity_source_missing"
FINAL_APPROACH_RECORD_UNAVAILABLE = "final_approach_record_unavailable"
SOURCE_EVENT_ACTIVE_POI_MISSING = "source_event_active_poi_missing"
SOURCE_EVENT_PUMP = "source_event_pump"

UNKNOWN_REASON_INDEX = 0
# Indexes 10-13 are spare; 13 is the largest the state byte carries.
MAX_REASON_INDEX = 13
NAV_FAIL_REASON_INDEX: Mapping[str, int] = {
    NAV_FAIL: 1,
    NO_DETECTION: 2,
    MISSED_DOCK: 3,
    NAVIGATION_REJECTED: 4,
    SOURCE_COMMAND_STALE: 5,
    DISCONTINUITY_SOURCE_MISSING: 6,
    FINAL_APPROACH_RECORD_UNAVAILABLE: 7,
    SOURCE_EVENT_ACTIVE_POI_MISSING: 8,
    SOURCE_EVENT_PUMP: 9,
}


def reason_index(reason: Optional[str]) -> int:
    return NAV_FAIL_REASON_INDEX.get(reason or "", UNKNOWN_REASON_INDEX)


__all__ = [
    "DISCONTINUITY_SOURCE_MISSING",
    "FINAL_APPROACH_RECORD_UNAVAILABLE",
    "MAX_REASON_INDEX",
    "MISSED_DOCK",
    "NAVIGATION_REJECTED",
    "NAV_FAIL",
    "NAV_FAIL_REASON_INDEX",
    "NO_DETECTION",
    "SOURCE_COMMAND_STALE",
    "SOURCE_EVENT_ACTIVE_POI_MISSING",
    "SOURCE_EVENT_PUMP",
    "UNKNOWN_REASON_INDEX",
    "reason_index",
]
