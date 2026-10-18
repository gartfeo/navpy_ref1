"""Camera components with video streams, from the resolved vision profile.

Operator display only (docs/design/gcs_video_streams.md, section 1).
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from navpy.core.perception.profile.vision_profile_types import CameraMountSpec
from navpy.core.perception.profile.vision_video_streams import parse_video_streams
from navpy.plugins.vehicle.ardupilot.camera_stream_messages import StreamingCamera


def streaming_cameras(
    mount_specs: Iterable[CameraMountSpec],
) -> tuple[StreamingCamera, ...]:
    """One camera per mount whose profile device has ``video_streams``."""
    cameras = []
    for spec in mount_specs:
        name = spec.device.get("name", "")
        name = name if isinstance(name, str) else ""
        streams = parse_video_streams(spec.device, name)
        if not streams:
            continue
        camera = spec.device.get("camera")
        camera = camera if isinstance(camera, Mapping) else {}
        cameras.append(StreamingCamera(
            gimbal_device_id=spec.gimbal_device_id,
            model_name=name,
            resolution_h=_image_size(camera.get("image_width")),
            resolution_v=_image_size(camera.get("image_height")),
            streams=streams,
        ))
    return tuple(cameras)


def _image_size(value: object) -> int:
    """A configured image size for CAMERA_INFORMATION (uint16); 0 when unknown."""
    if isinstance(value, bool) or not isinstance(value, int):
        return 0
    return value if 0 < value <= 0xFFFF else 0


__all__ = ["streaming_cameras"]
