"""Turn one associated pose into a rendered detection.

Split from ``direct_target_pixel_source``, which owns the message plumbing,
the lock, and the publish slot. This owns the optics: which pose the ray is
built from, which attitude de-rotates it, and how the result is stamped.

The split matters for one reason beyond size. The source's whole job is to
decide WHEN a pose is legitimate -- bracketed by two truth samples, inside
the association gate, belonging to the current leg. Rendering must not be
able to reach around that decision, so it takes the pose it is handed and
has no access to the vehicle at all.
"""

from __future__ import annotations

import time
from collections.abc import Callable

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
from navpy.modules.vision.sim.ideal_target_projector import (
    IdealTargetProjector,
    UasFrameConvention,
)
from navpy.modules.vision.sim.pose_associator import AssociatedPose
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.simulation_object import SimulationObject


def _static_camera(name: str) -> Callable[[], GimbalData]:
    """A camera bolted to the airframe: no slew, no lock, no pointing."""
    def read() -> GimbalData:
        return GimbalData(att=Attitude(0.0, 0.0, 0.0), name=name)

    return read


class DirectPixelRenderer:
    """Project the known target from a supplied pose, at a supplied clock."""

    def __init__(
        self,
        target: Location,
        *,
        source_name: str,
        aircraft_sequence: str,
        aircraft_degrees: bool,
        frame_size: FrameSize,
        source_now_s: Callable[[], float],
        wall_now_s: Callable[[], float] = time.time,
    ) -> None:
        self._target = SimulationObject(1, target, 2)
        self._wall_now_s = wall_now_s
        camera = IdealCameraState(_static_camera(source_name))
        camera.prepare()
        self._projector = IdealTargetProjector(
            camera,
            frame_size,
            UasFrameConvention(aircraft_sequence, aircraft_degrees),
            source_now_s,
        )

    def render(
        self,
        associated: AssociatedPose,
        pose: tuple[Location, Attitude],
    ) -> DetectedObject | None:
        """Project the target, or None if the geometry has no answer."""
        location, render_attitude = pose
        navigation_attitude = associated.attitude_sample.attitude
        projected = self._projector.project(
            location,
            self._target,
            render_attitude,
            timestamp_s=associated.attitude_timestamp_s,
            # Yaw is ZEROED, not passed: the ray is already built in truth
            # attitude, and the navigation copy exists so the law can use the
            # pitch/roll it is allowed to see. Feeding the estimate's yaw
            # here would put compass bias back into the command path.
            navigation_attitude=Attitude(
                pitch=float(navigation_attitude.pitch),
                yaw=0.0,
                roll=float(navigation_attitude.roll),
            ),
            uas_body_rates_rad_s=associated.attitude_sample.body_rates_rad_s,
        )
        if projected is None:
            return None
        projected.record_receipt(
            max(associated.attitude_receipt_s, associated.truth_receipt_s),
            self._wall_now_s,
            associated.air_speed_mps,
        )
        return projected


__all__ = ["DirectPixelRenderer"]
