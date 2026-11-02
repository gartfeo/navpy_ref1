"""Per-open frame admission for the encoded stream: no GStreamer here.

A repeated PTS is the same picture, so it is dropped and counted. A PTS that
runs backward, a missing PTS or a new picture size means the stream was reset:
the open ends, and only a new run may open it again.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


class EncodedStreamReset(RuntimeError):
    """The stream this open was reading is no longer the same stream."""


@dataclass(frozen=True)
class FrameFacts:
    pts_ns: Optional[int]
    width: int
    height: int
    corrupted: bool = False


class EncodedFrameGate:
    """Decide for each decoded sample: publish, drop, or end the open."""

    def __init__(self) -> None:
        self.accepted = 0
        self.duplicates = 0
        self.corrupted = 0
        self.first_pts_ns: Optional[int] = None
        self.last_pts_ns: Optional[int] = None
        self._size: Optional[tuple[int, int]] = None

    def admit(self, facts: FrameFacts) -> Optional[int]:
        """The frame's 1-based sequence in this open; None when it is dropped."""
        if facts.pts_ns is None:
            raise EncodedStreamReset("decoded frame without a PTS")
        size = (facts.width, facts.height)
        if self._size is not None and size != self._size:
            raise EncodedStreamReset(f"picture size changed from {self._size} to {size}")
        if self.last_pts_ns is not None and facts.pts_ns < self.last_pts_ns:
            raise EncodedStreamReset(f"PTS ran backward from {self.last_pts_ns} to {facts.pts_ns}")
        if facts.pts_ns == self.last_pts_ns:
            self.duplicates += 1
            return None
        self._size = size
        self.last_pts_ns = facts.pts_ns
        if self.first_pts_ns is None:
            self.first_pts_ns = facts.pts_ns
        if facts.corrupted:     # a concealed picture is not a measurement
            self.corrupted += 1
            return None
        self.accepted += 1
        return self.accepted


__all__ = ["EncodedFrameGate", "EncodedStreamReset", "FrameFacts"]
