"""A camera's announced video stream: operator display only.

Nothing here feeds navigation or the final-approach command path
(docs/design/gcs_video_streams.md).
"""

from __future__ import annotations

from dataclasses import dataclass

VIDEO_STREAM_ENCODINGS = frozenset({"H264", "H265"})


@dataclass(frozen=True)
class VideoStreamSpec:
    """One configured stream, as a ground station is told about it."""

    stream_id: int
    name: str
    uri: str
    encoding: str  # one of VIDEO_STREAM_ENCODINGS
    resolution_h: int
    resolution_v: int
    framerate: float


__all__ = ["VIDEO_STREAM_ENCODINGS", "VideoStreamSpec"]
