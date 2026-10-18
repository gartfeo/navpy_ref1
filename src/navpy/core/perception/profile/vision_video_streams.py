"""Optional per-camera video streams announced to ground stations.

Operator display only: nothing here feeds navigation or the final-approach
command path (docs/design/gcs_video_streams.md, section 1).

Profile format, per device (sibling of ``camera`` and ``gimbal``)::

    "video_streams": [
        {"stream_id": 1, "name": "tracking overlay",
         "uri": "rtsp://192.168.144.30:8554/overlay", "encoding": "H264",
         "resolution_h": 1280, "resolution_v": 720, "framerate": 30.0}
    ]

No ``video_streams`` key means the camera announces nothing.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from navpy.core.ports.video_stream import VIDEO_STREAM_ENCODINGS, VideoStreamSpec

VIDEO_STREAMS_KEY = "video_streams"
VIDEO_STREAM_URI_SCHEME = "rtsp://"

# Field sizes of MAVLink VIDEO_STREAM_INFORMATION: stream_id and count are
# uint8, resolutions uint16, name char[32], uri char[160].
_MAX_STREAM_ID = 255
_MAX_RESOLUTION = 65535
_MAX_NAME_BYTES = 32
_MAX_URI_BYTES = 160
_FIELDS = frozenset({
    "stream_id",
    "name",
    "uri",
    "encoding",
    "resolution_h",
    "resolution_v",
    "framerate",
})


def parse_video_streams(
    device: Mapping[str, object],
    device_name: str,
) -> tuple[VideoStreamSpec, ...]:
    """Validated streams of one profile device, ordered by stream id."""
    raw = device.get(VIDEO_STREAMS_KEY)
    if raw is None:
        return ()
    where = f"device '{device_name}' {VIDEO_STREAMS_KEY}"
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise ValueError(f"{where} must be a list")
    streams = tuple(
        _parse_stream(entry, f"{where}[{index}]")
        for index, entry in enumerate(raw)
    )
    seen: set[int] = set()
    for stream in streams:
        if stream.stream_id in seen:
            raise ValueError(f"{where} has duplicate stream_id {stream.stream_id}")
        seen.add(stream.stream_id)
    # MAVLink numbers a camera's streams 1..count (1 first, 2 second, ...);
    # the GCS discards ids outside that range, so a gap could never finish.
    if seen != set(range(1, len(streams) + 1)):
        raise ValueError(f"{where} stream_ids must be 1..{len(streams)} without gaps")
    return tuple(sorted(streams, key=lambda stream: stream.stream_id))


def _parse_stream(entry: object, where: str) -> VideoStreamSpec:
    if not isinstance(entry, Mapping):
        raise ValueError(f"{where} must be a mapping")
    missing = sorted(_FIELDS - set(entry))
    if missing:
        raise ValueError(f"{where} is missing {', '.join(missing)}")
    unknown = sorted(set(entry) - _FIELDS)
    if unknown:
        raise ValueError(f"{where} has unknown keys {', '.join(unknown)}")
    encoding = entry["encoding"]
    if encoding not in VIDEO_STREAM_ENCODINGS:
        raise ValueError(
            f"{where}.encoding must be one of {sorted(VIDEO_STREAM_ENCODINGS)}"
        )
    return VideoStreamSpec(
        stream_id=_int_in_range(entry["stream_id"], 1, _MAX_STREAM_ID, f"{where}.stream_id"),
        name=_text(entry["name"], _MAX_NAME_BYTES, f"{where}.name"),
        uri=_uri(entry["uri"], f"{where}.uri"),
        encoding=str(encoding),
        resolution_h=_int_in_range(
            entry["resolution_h"], 1, _MAX_RESOLUTION, f"{where}.resolution_h"
        ),
        resolution_v=_int_in_range(
            entry["resolution_v"], 1, _MAX_RESOLUTION, f"{where}.resolution_v"
        ),
        framerate=_positive_float(entry["framerate"], f"{where}.framerate"),
    )


def _int_in_range(value: object, low: int, high: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def _positive_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a positive number")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be a positive number")
    return number


def _text(value: object, max_bytes: int, name: str) -> str:
    if not isinstance(value, str) or not value or not value.isascii():
        raise ValueError(f"{name} must be a non-empty ASCII string")
    if len(value) > max_bytes or not value.isprintable():
        raise ValueError(
            f"{name} must be printable and at most {max_bytes} characters"
        )
    return value


def _uri(value: object, name: str) -> str:
    uri = _text(value, _MAX_URI_BYTES, name)
    if not uri.startswith(VIDEO_STREAM_URI_SCHEME) or any(c.isspace() for c in uri):
        raise ValueError(f"{name} must be an rtsp:// URI without spaces")
    if len(uri) == len(VIDEO_STREAM_URI_SCHEME):
        raise ValueError(f"{name} must name a host")
    return uri


__all__ = [
    "VIDEO_STREAMS_KEY",
    "parse_video_streams",
]
