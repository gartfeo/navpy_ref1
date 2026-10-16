"""Public deterministic SIYI gimbal-physics adapter.

The angular and zoom plants own their state independently. ``update(dt)``
advances exactly the supplied physical interval; scheduling speed never enters
this module.
"""

from __future__ import annotations

from navpy.sim.gimbal.siyi.gimbal_angles import (
    ANGLE_SEEK_DEADBAND,
    GimbalAngularPlant,
    clamp as _clamp,
)
from navpy.sim.gimbal.siyi.gimbal_frame_transform import (
    matrix_to_zyx as _matrix_to_zyx,
    zyx_to_matrix as _zyx_to_matrix,
)
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import (
    GimbalZoomPlant,
    ZOOM_SEEK_SPEED,
)
from navpy.sim.gimbal.siyi.gimbal_physics_facets import (
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
    "ZOOM_SEEK_SPEED",
    "_clamp",
    "_matrix_to_zyx",
    "_zyx_to_matrix",
]
