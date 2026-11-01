"""Camera-protocol answers from NavPy's camera components.

Each camera mount is one MAVLink camera component (MAV_COMP_ID_CAMERA.. by
gimbal device id). It announces itself with HEARTBEAT, whose system_status is
its detector's frame readiness and whose custom_mode is NavPy's boot id, and
answers REQUEST_MESSAGE for CAMERA_INFORMATION and VIDEO_STREAM_INFORMATION
(docs/design/gcs_video_streams.md, section 1;
docs/design/gcs_preflight_camera_row.md). Operator display only: nothing here
reads or feeds navigation state.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

from pymavlink.dialects.v20.ardupilotmega import (
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,
    MAV_AUTOPILOT_INVALID,
    MAV_CMD_REQUEST_MESSAGE,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_TYPE_CAMERA,
    MAVLINK_MSG_ID_CAMERA_INFORMATION,
    VIDEO_STREAM_ENCODING_H264,
    VIDEO_STREAM_ENCODING_H265,
    VIDEO_STREAM_TYPE_RTSP,
    MAVLink_camera_information_message,
    MAVLink_command_ack_message,
    MAVLink_heartbeat_message,
    MAVLink_message,
    MAVLink_video_stream_information_message,
)

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.camera_health import CameraHealth
from navpy.core.ports.video_stream import VideoStreamSpec
from navpy.plugins.vehicle.ardupilot.camera_stream_requests import (
    is_camera_stream_request,
    requested_message_id,
)
from navpy.plugins.vehicle.ardupilot.mavlink_camera_components import (
    CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID,
)

_ENCODING_BY_NAME = {
    "H264": VIDEO_STREAM_ENCODING_H264,
    "H265": VIDEO_STREAM_ENCODING_H265,
}
_MAVLINK_VERSION = 3
_CAMERA_NAME_BYTES = 32
_BOOT_ID_MAX = 0xFFFFFFFF  # HEARTBEAT custom_mode is uint32


@dataclass(frozen=True)
class StreamingCamera:
    """One camera component, its detector readiness and its streams (may be none)."""

    gimbal_device_id: int
    model_name: str
    resolution_h: int  # configured image size; 0 when unknown
    resolution_v: int
    streams: tuple[VideoStreamSpec, ...]
    camera_health: CameraHealth

    def __post_init__(self) -> None:
        if self.gimbal_device_id not in CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID:
            raise ValueError(
                f"camera '{self.model_name}' has no camera component for "
                f"gimbal_device_id {self.gimbal_device_id}"
            )

    @property
    def component_id(self) -> int:
        return CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID[self.gimbal_device_id]


def new_camera_boot_id() -> int:
    """A random id for this NavPy process; never 0, which means older NavPy."""
    return rt.rng().randint(1, _BOOT_ID_MAX)


def camera_heartbeat(ready: bool, boot_id: int) -> MAVLink_heartbeat_message:
    """ACTIVE when the mount's detector gets frames, else CRITICAL."""
    return MAVLink_heartbeat_message(
        type=MAV_TYPE_CAMERA,
        autopilot=MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=boot_id,
        system_status=MAV_STATE_ACTIVE if ready else MAV_STATE_CRITICAL,
        mavlink_version=_MAVLINK_VERSION,
    )


class CameraStreamResponder:
    """Answer one camera-protocol request: ACK, then the requested payload."""

    def __init__(self, own_system: int, cameras: Iterable[StreamingCamera]) -> None:
        self._own_system = own_system
        self._cameras = {camera.component_id: camera for camera in cameras}
        self._boot_monotonic_s = rt.monotonic()

    @property
    def component_ids(self) -> tuple[int, ...]:
        return tuple(self._cameras)

    @property
    def cameras(self) -> tuple[StreamingCamera, ...]:
        return tuple(self._cameras.values())

    def answer(self, request: MAVLink_message) -> tuple[MAVLink_message, ...]:
        """Messages to send from ``request.target_component``; () to ignore."""
        if not is_camera_stream_request(request, self._own_system):
            return ()
        camera = self._cameras.get(request.target_component)
        if camera is None:
            return ()
        if requested_message_id(request) == MAVLINK_MSG_ID_CAMERA_INFORMATION:
            return (_ack(request, MAV_RESULT_ACCEPTED), self._information(camera))
        streams = _requested_streams(camera.streams, request.param2)
        if not streams:
            return (_ack(request, MAV_RESULT_DENIED),)
        count = len(camera.streams)
        return (
            _ack(request, MAV_RESULT_ACCEPTED),
            *(_stream_information(stream, count) for stream in streams),
        )

    def _information(
        self,
        camera: StreamingCamera,
    ) -> MAVLink_camera_information_message:
        model = camera.model_name.encode("ascii", "replace")[:_CAMERA_NAME_BYTES]
        vendor = camera.camera_health.vendor.encode("ascii", "replace")[:_CAMERA_NAME_BYTES]
        return MAVLink_camera_information_message(
            time_boot_ms=int(
                (rt.monotonic() - self._boot_monotonic_s) * 1000
            ) & 0xFFFFFFFF,
            vendor_name=list(vendor.ljust(_CAMERA_NAME_BYTES, b"\0")),
            model_name=list(model.ljust(_CAMERA_NAME_BYTES, b"\0")),
            firmware_version=0,
            focal_length=math.nan,
            sensor_size_h=math.nan,
            sensor_size_v=math.nan,
            resolution_h=camera.resolution_h,
            resolution_v=camera.resolution_v,
            lens_id=0,
            flags=CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM if camera.streams else 0,
            cam_definition_version=0,
            cam_definition_uri=b"",
            gimbal_device_id=camera.gimbal_device_id,
        )


def _requested_streams(
    streams: tuple[VideoStreamSpec, ...],
    param2: float,
) -> tuple[VideoStreamSpec, ...]:
    """param2 0 asks for every stream; any other value names one stream id."""
    if not math.isfinite(param2) or not float(param2).is_integer():
        return ()
    stream_id = int(param2)
    if stream_id == 0:
        return streams
    return tuple(stream for stream in streams if stream.stream_id == stream_id)


def _ack(request: MAVLink_message, result: int) -> MAVLink_command_ack_message:
    return MAVLink_command_ack_message(
        command=MAV_CMD_REQUEST_MESSAGE,
        result=result,
        progress=0,
        result_param2=0,
        target_system=request.get_srcSystem(),
        target_component=request.get_srcComponent(),
    )


def _stream_information(
    stream: VideoStreamSpec,
    count: int,
) -> MAVLink_video_stream_information_message:
    # flags 0: the companion announces configuration, it does not probe
    # whether the stream server is running.
    return MAVLink_video_stream_information_message(
        stream_id=stream.stream_id,
        count=count,
        type=VIDEO_STREAM_TYPE_RTSP,
        flags=0,
        framerate=stream.framerate,
        resolution_h=stream.resolution_h,
        resolution_v=stream.resolution_v,
        bitrate=0,
        rotation=0,
        hfov=0,
        name=stream.name.encode("ascii"),
        uri=stream.uri.encode("ascii"),
        encoding=_ENCODING_BY_NAME[stream.encoding],
    )


__all__ = [
    "CameraStreamResponder",
    "StreamingCamera",
    "camera_heartbeat",
    "new_camera_boot_id",
]
