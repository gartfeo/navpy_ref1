"""Camera components, from the resolved vision profile and detector nodes.

Operator display only (docs/design/gcs_video_streams.md, section 1;
docs/design/gcs_preflight_camera_row.md).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from navpy.core.perception.profile.vision_profile_types import CameraMountSpec
from navpy.core.ports.camera_health import CameraHealth
from navpy.core.perception.profile.vision_video_streams import parse_video_streams
from navpy.plugins.vehicle.ardupilot.camera_stream_messages import StreamingCamera


def streaming_cameras(
    mount_specs: Sequence[CameraMountSpec],
    health: Sequence[CameraHealth],
) -> tuple[StreamingCamera, ...]:
    """One camera per mount; ``health[i]`` is mount i's detector readiness.

    A mount without a gimbal device id has no camera component: it is left
    out, unless it has ``video_streams``, which then fail as before.
    """
    if len(mount_specs) != len(health):
        raise ValueError(
            f"{len(mount_specs)} camera mounts but {len(health)} health ports"
        )
    cameras = []
    for spec, mount_health in zip(mount_specs, health):
        name = spec.device.get("name", "")
        name = name if isinstance(name, str) else ""
        streams = parse_video_streams(spec.device, name)
        if spec.gimbal_device_id is None and not streams:
            continue
        camera = spec.device.get("camera")
        camera = camera if isinstance(camera, Mapping) else {}
        cameras.append(StreamingCamera(
            gimbal_device_id=spec.gimbal_device_id,
            model_name=name,
            resolution_h=_image_size(camera.get("image_width")),
            resolution_v=_image_size(camera.get("image_height")),
            streams=streams,
            camera_health=mount_health,
        ))
    return tuple(cameras)


def _image_size(value: object) -> int:
    """A configured image size for CAMERA_INFORMATION (uint16); 0 when unknown."""
    if isinstance(value, bool) or not isinstance(value, int):
        return 0
    return value if 0 < value <= 0xFFFF else 0


__all__ = ["streaming_cameras"]
