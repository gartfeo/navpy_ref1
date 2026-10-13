"""Current camera optics for the final-approach completion check (a7).

Camera state only (intrinsics, zoom, image bounds), never a command input.
A dock lost after a zoom/intrinsics change is not judged with the optics of
the last frame (visual_pass.py latch_detection_loss).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Protocol

import numpy as np

from navpy.core.approach.vision_nav.visual_pass import PASS_MAX_ZOOM_OVER_WIDEST
from navpy.core.perception.visual_ray_projection import camera_half_fov_deg
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.pixel_observation import PixelCalibration, PixelProjectionKind


class OpticsMount(Protocol):
    @property
    def image_width(self) -> int | None: ...

    @property
    def image_height(self) -> int | None: ...

    def get_k_for_frame(self, frame_w: int, frame_h: int) -> np.ndarray: ...

    def get_gimbal_data(self) -> GimbalData: ...

    def get_zoom_levels(self) -> list[str]: ...

    def get_fresh_zoom_command(self) -> str | None: ...


def fully_zoomed_out(mount: OpticsMount) -> bool:
    """Fresh zoom readback at the widest level (temporary a7 guard).

    A camera with at most one zoom level has no zoom control and is always at
    its widest. Otherwise only a fresh readback counts, by the mount's own
    freshness (LIVE_ZOOM_READBACK_STALE_S); stale or missing fails closed.
    """
    levels = [float(level) for level in mount.get_zoom_levels()]
    if len(levels) <= 1:
        return True
    current = mount.get_fresh_zoom_command()
    if current is None:
        return False
    widest = min(levels)
    return widest > 0.0 and float(current) <= PASS_MAX_ZOOM_OVER_WIDEST * widest


def mount_half_fov_reader(
    mounts: Callable[[], Sequence[OpticsMount]],
) -> Callable[[str], float | None]:
    """Return a reader of the named camera's current short half-FOV (deg).

    None (no centred-loss pass) when the camera is unknown, its optics are
    unreadable, or it is not fully zoomed out.
    """

    def read(source_name: str) -> float | None:
        for mount in mounts():
            try:
                if str(mount.get_gimbal_data().name) != source_name:
                    continue
                if not fully_zoomed_out(mount):
                    return None
                width, height = mount.image_width, mount.image_height
                if width is None or height is None:
                    return None
                calibration = PixelCalibration.from_matrix(
                    mount.get_k_for_frame(int(width), int(height))
                )
                return camera_half_fov_deg(
                    calibration,
                    PixelProjectionKind.PINHOLE,
                    (float(width), float(height)),
                )
            except Exception:  # noqa: BLE001 - unknown optics fail closed
                return None
        return None

    return read


__all__ = ["OpticsMount", "fully_zoomed_out", "mount_half_fov_reader"]
