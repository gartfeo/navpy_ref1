"""Camera-aware approach strategy selection.

Two strategies based on camera capabilities:
- OFFSET: Body-fixed camera. Fly to a bank-corrected offset point behind
          the target where the camera FOV intersects the ground.
- ORBIT:  Gimbal with tracking. Orbit directly around the target at a
          camera-geometry-matched radius via goto_loiter.
"""

from dataclasses import dataclass
from enum import Enum, unique
from typing import TYPE_CHECKING, List, Optional

if TYPE_CHECKING:
    from navpy.modules.vision.camera_mount import CameraMount
    from navpy.modules.common.models.location import Location


@unique
class ApproachKind(Enum):
    OFFSET = "offset"
    ORBIT = "orbit"


@dataclass(frozen=True)
class ApproachPlan:
    """Result of approach strategy computation."""
    kind: ApproachKind
    approach_location: "Location"
    offset_distance: float
    orbit_radius: Optional[float] = None


def classify_approach(
    mounts: List["CameraMount"],
    devices: List[dict],
) -> ApproachKind:
    """Determine approach strategy from camera mount capabilities.

    Any camera with tracking enabled uses ORBIT (goto_loiter around target).
    Fixed cameras use OFFSET (fly to bank-corrected offset point).
    """
    for device in devices:
        tracking = device.get("gimbal", {}).get("tracking", {})
        if tracking.get("enabled", False):
            return ApproachKind.ORBIT
    return ApproachKind.OFFSET
