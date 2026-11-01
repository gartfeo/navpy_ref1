"""What one mission download found.

A count of 0 cannot tell an empty mission table from a failed transfer; the
GCS mission route answers the two differently (an empty vehicle is not a
download failure).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MissionDownloadOutcome(str, Enum):
    #: A valid MISSION_COUNT of 0 arrived.
    EMPTY = "empty"
    #: A positive count arrived and every item arrived, staged into the store.
    COMPLETE = "complete"
    #: No count, a missing item, a cancel or the deadline; the store is unchanged.
    FAILED = "failed"


@dataclass(frozen=True)
class MissionDownload:
    outcome: MissionDownloadOutcome
    #: Items downloaded; 0 unless complete.
    count: int = 0

    @property
    def failed(self) -> bool:
        return self.outcome is MissionDownloadOutcome.FAILED


__all__ = ["MissionDownload", "MissionDownloadOutcome"]
