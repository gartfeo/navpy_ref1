"""Per-vehicle video-stream discovery and the snapshot's ``video_streams`` list.

Display only (docs/design/gcs_video_streams.md): the backend records what the
vehicle's own companion camera components announce and derives a WHEP URL for
the browser. It never connects to a stream, and no navigation module imports
this code (``tests/guards/test_video_streams_isolation.py``).

Only independent camera components of this vehicle's companion are followed
(source sysid = the vehicle, source component = ``MAV_COMP_ID_CAMERA..6``);
autopilot-proxied cameras behind component 1 are out of scope in v1.

The MAVLink reader thread calls the ``on_*`` handlers; the telemetry loop
calls :meth:`tick` and :meth:`snapshot` every cycle, which is what turns
request timeouts and lost cameras into state. Sends happen outside the lock.
"""
from __future__ import annotations

import logging
from typing import Callable, Optional, Protocol

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_REQUEST_MESSAGE,
    MAV_TYPE_CAMERA,
    MAVLink_command_long_message,
    MAVLink_message,
    VIDEO_STREAM_ENCODING_H264,
    VIDEO_STREAM_ENCODING_H265,
)

from gcs.backend.companion_identity import is_from_companion, source_ids
from gcs.backend.video_camera_discovery import (
    PHASE_FAILED, CameraDiscovery, CameraRequest, VideoStream,
)
from gcs.backend.video_stream_policy import VideoPolicy, current_video_policy
from gcs.backend.video_stream_uri import validate_rtsp_uri, whep_url, RtspLocation
from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.mavlink_camera_components import (
    GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT,
)

log = logging.getLogger("gcs.backend.video_streams")

CAMERA_COMPONENT_IDS: frozenset[int] = frozenset(GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT)
ENCODING_NAMES = {VIDEO_STREAM_ENCODING_H264: "H264", VIDEO_STREAM_ENCODING_H265: "H265"}


def _int_field(msg: MAVLink_message, name: str) -> Optional[int]:
    value = getattr(msg, name, None)
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _text_field(msg: MAVLink_message, name: str) -> str:
    value = getattr(msg, name, "")
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    return str(value).rstrip("\x00")


class CameraRequestSink(Protocol):
    """The part of a vehicle the requests go out through."""

    def send_mavlink_message(self, message: MAVLink_message) -> None: ...


class VehicleVideoStreams:
    """Discovery generations of one vehicle's camera components."""

    def __init__(
        self,
        sys_id: int,
        vehicle: CameraRequestSink,
        policy: Callable[[], VideoPolicy] = current_video_policy,
    ) -> None:
        self._sys_id = sys_id
        self._vehicle = vehicle
        self._policy = policy
        self._lock = rt.Lock()
        self._cameras: dict[int, CameraDiscovery] = {}

    def on_heartbeat(self, msg: MAVLink_message) -> None:
        component = self._camera_component(msg)
        if component is None or getattr(msg, "type", None) != MAV_TYPE_CAMERA:
            return
        now = rt.monotonic()
        with self._lock:
            camera = self._cameras.get(component)
            if camera is None or camera.is_lost(now):
                camera = CameraDiscovery(component, now)
                self._cameras[component] = camera
                log.info("Camera %d:%d seen: discovery started", self._sys_id, component)
            camera.heartbeat(now)
            request = self._poll(camera, now)
        self._send(component, request)

    def on_command_ack(self, msg: MAVLink_message) -> None:
        component = self._camera_component(msg)
        if component is None or _int_field(msg, "command") != MAV_CMD_REQUEST_MESSAGE:
            return
        own_system = getattr(self._vehicle, "transport_source_system", None)
        if isinstance(own_system, int) and _int_field(msg, "target_system") != own_system:
            return  # another GCS's request
        result = _int_field(msg, "result")
        if result is None:
            return
        self._update(component, lambda camera: camera.on_ack(result))

    def on_camera_information(self, msg: MAVLink_message) -> None:
        component = self._camera_component(msg)
        flags = _int_field(msg, "flags")
        if component is None or flags is None:
            return
        self._update(component, lambda camera: camera.on_camera_information(flags))

    def on_video_stream_information(self, msg: MAVLink_message) -> None:
        component = self._camera_component(msg)
        stream_id = _int_field(msg, "stream_id")
        count = _int_field(msg, "count")
        if component is None or stream_id is None or count is None:
            return
        stream = self._validated_stream(component, stream_id, msg)
        self._update(
            component,
            lambda camera: camera.on_stream_information(stream_id, count, stream),
        )

    def tick(self) -> None:
        """Expire request sends and drop cameras whose heartbeats stopped."""
        now = rt.monotonic()
        requests: list[tuple[int, Optional[CameraRequest]]] = []
        with self._lock:
            for component, camera in list(self._cameras.items()):
                if camera.is_lost(now):
                    del self._cameras[component]
                    log.info("Camera %d:%d lost: its streams removed", self._sys_id, component)
                    continue
                requests.append((component, self._poll(camera, now)))
        for component, request in requests:
            self._send(component, request)

    def snapshot(self) -> list[dict]:
        """Streams ordered by camera component, then stream id."""
        whep_port = self._policy().whep_port
        with self._lock:
            found = [(c, self._cameras[c].streams()) for c in sorted(self._cameras)]
        return [
            {
                "key": f"{component}:{stream.stream_id}",
                "camera_component": component,
                "stream_id": stream.stream_id,
                "name": stream.name,
                "encoding": stream.encoding,
                "whep_url": whep_url(RtspLocation(stream.host, stream.path), whep_port),
            }
            for component, streams in found
            for stream in streams
        ]

    def _camera_component(self, msg: MAVLink_message) -> Optional[int]:
        if not is_from_companion(msg, self._sys_id, CAMERA_COMPONENT_IDS):
            return None
        ids = source_ids(msg)
        return ids[1] if ids is not None else None

    def _validated_stream(
        self, component: int, stream_id: int, msg: MAVLink_message,
    ) -> Optional[VideoStream]:
        uri = _text_field(msg, "uri")
        verdict = validate_rtsp_uri(uri, self._policy().networks)
        if verdict.location is None:
            log.warning(
                "Camera %d:%d stream %d URI rejected (%s): %r",
                self._sys_id, component, stream_id, verdict.reason, uri,
            )
            return None
        return VideoStream(
            stream_id=stream_id,
            name=_text_field(msg, "name"),
            encoding=ENCODING_NAMES.get(_int_field(msg, "encoding")),
            host=verdict.location.host,
            path=verdict.location.path,
        )

    def _update(
        self, component: int, change: Callable[[CameraDiscovery], None],
    ) -> None:
        now = rt.monotonic()
        with self._lock:
            camera = self._cameras.get(component)
            if camera is None:
                return  # not seen in this generation: an answer to nothing
            change(camera)
            request = self._poll(camera, now)
        self._send(component, request)

    def _poll(self, camera: CameraDiscovery, now: float) -> Optional[CameraRequest]:
        before = camera.phase
        request = camera.poll(now)
        if camera.phase == PHASE_FAILED and before != PHASE_FAILED:
            log.warning(
                "Camera %d:%d discovery failed: retries exhausted; next try after "
                "the camera is lost and seen again", self._sys_id, camera.component,
            )
        return request

    def _send(self, component: int, request: Optional[CameraRequest]) -> None:
        if request is None:
            return
        message = MAVLink_command_long_message(
            self._sys_id, component, MAV_CMD_REQUEST_MESSAGE, request.confirmation,
            request.message_id, request.param2, 0, 0, 0, 0, 0,
        )
        try:
            self._vehicle.send_mavlink_message(message)
        except OSError as exc:
            # The send times out like a lost packet and is retried.
            log.debug("Camera %d:%d request send failed: %s", self._sys_id, component, exc)
