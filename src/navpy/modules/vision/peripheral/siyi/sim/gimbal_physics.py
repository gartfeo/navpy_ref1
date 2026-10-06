"""Public deterministic SIYI gimbal-physics adapter.

The angular and zoom plants own their state independently. ``update(dt)``
advances exactly the supplied physical interval; scheduling speed never enters
this module.
"""

from __future__ import annotations

from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import (
    ANGLE_SEEK_DEADBAND,
    MAX_SLEW_RATE,
    MODE_FOLLOW,
    MODE_FPV,
    MODE_LOCK,
    GimbalAngularPlant,
    clamp as _clamp,
    wrap180 as _wrap180,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_frame_transform import (
    matrix_to_xyz as _matrix_to_xyz,
    zyx_to_matrix as _zyx_to_matrix,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_zoom_plant import (
    GimbalZoomPlant,
    ZOOM_INCREMENTAL_SPEED,
    ZOOM_SEEK_SPEED,
)
from navpy.modules.vision.peripheral.siyi.siyi_sdk import ZR10
from navpy.modules.vision.peripheral.siyi.sim.gimbal_physics_facets import (
    GimbalPhysicsControlFacet,
    GimbalPhysicsParts,
    GimbalPhysicsReadFacet,
    GimbalPhysicsSnapshot,
)


class GimbalPhysics(GimbalPhysicsReadFacet, GimbalPhysicsControlFacet):
    """One-field public adapter over independent angular and zoom plants."""

    def __init__(
        self,
        initial_pitch: float = 0.0,
        initial_yaw: float = 0.0,
        initial_zoom: float = 1.0,
    ) -> None:
        self._parts = GimbalPhysicsParts(
            GimbalAngularPlant(initial_pitch, initial_yaw),
            GimbalZoomPlant(initial_zoom),
        )


__all__ = [
    "ANGLE_SEEK_DEADBAND",
    "GimbalPhysics",
    "GimbalPhysicsParts",
    "GimbalPhysicsSnapshot",
    "MAX_SLEW_RATE",
    "MODE_FOLLOW",
    "MODE_FPV",
    "MODE_LOCK",
    "ZOOM_INCREMENTAL_SPEED",
    "ZOOM_SEEK_SPEED",
    "ZR10",
    "_clamp",
    "_matrix_to_xyz",
    "_wrap180",
    "_zyx_to_matrix",
]
