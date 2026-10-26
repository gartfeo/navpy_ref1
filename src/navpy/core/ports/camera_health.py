"""What NavPy's camera component reports about one detector camera.

Operator display only (docs/design/gcs_preflight_camera_row.md): the camera
HEARTBEAT's system_status and CAMERA_INFORMATION's vendor_name. Nothing on the
navigation command path reads it (tests/guards/test_camera_health_isolation.py).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

# CAMERA_INFORMATION vendor_name of NavPy's camera components. Only cameras
# with these vendors report detector readiness in their HEARTBEAT
# system_status and NavPy's boot id in its custom_mode; an older NavPy sends
# an empty vendor and ACTIVE whatever its detector does.
NAVPY_CAMERA_VENDOR = "NavPy"
NAVPY_SIM_CAMERA_VENDOR = "NavPy sim"


@dataclass(frozen=True)
class CameraHealth:
    """One mount's vendor name and its detector-readiness reading."""

    vendor: str
    ready: Callable[[], bool]


__all__ = ["CameraHealth", "NAVPY_CAMERA_VENDOR", "NAVPY_SIM_CAMERA_VENDOR"]
