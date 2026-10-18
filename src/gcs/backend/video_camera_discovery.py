"""One camera component's video-stream discovery (MAVLink camera protocol v2).

Display only: this records what a camera announces; nothing here connects to
a stream or feeds navigation (docs/design/gcs_video_streams.md, section 1).

One generation per camera lifetime as seen by the GCS: the owner
(``video_streams.VehicleVideoStreams``) creates a new :class:`CameraDiscovery`
when a camera HEARTBEAT arrives while the camera is unknown or lost, which
drops the previous generation's streams. Requests are serialized: at most one
is outstanding, so a COMMAND_ACK from the camera belongs to it.

A send fails when its payload has not arrived within
``CAMERA_REQUEST_TIMEOUT_S`` (an ACK alone does not complete it) or when the
camera ACKs with a rejecting result. Each request gets up to
``CAMERA_REQUEST_MAX_SENDS`` sends; exhausting them leaves the camera
"failed" for the rest of the generation.

Pure state: every method takes ``now`` (monotonic seconds) and returns the
request to send, if any; the owner sends it.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

from pymavlink.dialects.v20.ardupilotmega import (
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_IN_PROGRESS,
    MAVLINK_MSG_ID_CAMERA_INFORMATION,
    MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
)

from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_LOST_AFTER_S,
    CAMERA_REQUEST_MAX_SENDS,
    CAMERA_REQUEST_TIMEOUT_S,
)

# param2 of MAV_CMD_REQUEST_MESSAGE(VIDEO_STREAM_INFORMATION): every stream.
ALL_STREAMS = 0

PHASE_INFO = "info"
PHASE_ALL_STREAMS = "all_streams"
PHASE_MISSING_STREAMS = "missing_streams"
PHASE_DONE = "done"
PHASE_FAILED = "failed"

# ACK results that do not end a send: the payload is still expected.
_ACK_PENDING_OK = frozenset({MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS})


@dataclass(frozen=True)
class CameraRequest:
    """One MAV_CMD_REQUEST_MESSAGE to send: param1 message id, param2.

    ``confirmation`` is COMMAND_LONG's resend counter (0 on the first send).
    """

    message_id: int
    param2: int
    confirmation: int = 0


@dataclass(frozen=True)
class VideoStream:
    """A validated stream: the RTSP host and path it was announced with."""

    stream_id: int
    name: str
    encoding: Optional[str]
    host: str
    path: str


@dataclass
class _Outstanding:
    request: CameraRequest
    sends: int
    sent_at: float
    failed: bool = False


class CameraDiscovery:
    """Discovery state of one (sysid, camera component) generation."""

    def __init__(self, component: int, now: float) -> None:
        self.component = component
        self.phase = PHASE_INFO
        self._last_heartbeat = now
        self._outstanding: Optional[_Outstanding] = None
        self._count: Optional[int] = None
        # stream id -> validated stream, or None when its URI was rejected
        # (received, so not requested again, but never shown).
        self._received: dict[int, Optional[VideoStream]] = {}
        self._missing: list[int] = []

    def heartbeat(self, now: float) -> None:
        self._last_heartbeat = now

    def is_lost(self, now: float) -> bool:
        """No camera HEARTBEAT for ``CAMERA_LOST_AFTER_S``."""
        return now - self._last_heartbeat >= CAMERA_LOST_AFTER_S

    def streams(self) -> list[VideoStream]:
        """Validated streams by id; none unless discovery completed."""
        if self.phase != PHASE_DONE:
            return []
        return [self._received[i] for i in sorted(self._received)
                if self._received[i] is not None]

    def poll(self, now: float) -> Optional[CameraRequest]:
        """Issue the next send, a resend after a failed send, or nothing."""
        if self.phase in (PHASE_DONE, PHASE_FAILED):
            return None
        out = self._outstanding
        if out is None:
            return self._send(self._next_request(), 1, now)
        if not out.failed and now - out.sent_at < CAMERA_REQUEST_TIMEOUT_S:
            return None
        if self.phase == PHASE_ALL_STREAMS and self._received:
            # Some streams arrived: ask for the rest one id at a time.
            self._enter_missing()
            return self.poll(now)
        if out.sends >= CAMERA_REQUEST_MAX_SENDS:
            self.phase = PHASE_FAILED
            self._outstanding = None
            return None
        return self._send(out.request, out.sends + 1, now)

    def on_ack(self, result: int) -> None:
        """A COMMAND_ACK for MAV_CMD_REQUEST_MESSAGE from this camera."""
        if self._outstanding is not None and result not in _ACK_PENDING_OK:
            self._outstanding.failed = True

    def on_camera_information(self, flags: int) -> None:
        if self.phase != PHASE_INFO:
            return
        self._outstanding = None
        if flags & CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM:
            self.phase = PHASE_ALL_STREAMS
        else:
            self.phase = PHASE_DONE

    def on_stream_information(
        self, stream_id: int, count: int, stream: Optional[VideoStream],
    ) -> None:
        """One VIDEO_STREAM_INFORMATION; *stream* is None if its URI was rejected."""
        if self.phase not in (PHASE_ALL_STREAMS, PHASE_MISSING_STREAMS):
            return
        if self._count is None and count >= 1:
            self._count = count
        if self._count is None or count != self._count:
            return
        if not 1 <= stream_id <= self._count:
            return
        self._received[stream_id] = stream
        if self.phase == PHASE_ALL_STREAMS:
            if len(self._received) == self._count:
                self._finish()
            return
        if stream_id in self._missing:
            self._missing.remove(stream_id)
            out = self._outstanding
            if out is not None and out.request.param2 == stream_id:
                self._outstanding = None
        if not self._missing:
            self._finish()

    def _next_request(self) -> CameraRequest:
        if self.phase == PHASE_INFO:
            return CameraRequest(MAVLINK_MSG_ID_CAMERA_INFORMATION, 0)
        if self.phase == PHASE_ALL_STREAMS:
            return CameraRequest(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, ALL_STREAMS)
        return CameraRequest(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, self._missing[0])

    def _send(self, request: CameraRequest, sends: int, now: float) -> CameraRequest:
        request = replace(request, confirmation=sends - 1)
        self._outstanding = _Outstanding(request, sends, now)
        return request

    def _enter_missing(self) -> None:
        # Only reached with streams received, so the count is known.
        count = self._count or 0
        self.phase = PHASE_MISSING_STREAMS
        self._outstanding = None
        self._missing = [i for i in range(1, count + 1) if i not in self._received]

    def _finish(self) -> None:
        self.phase = PHASE_DONE
        self._outstanding = None
        self._missing = []
