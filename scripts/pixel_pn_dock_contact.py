"""Dock contact for the standalone pixel-PN harnesses (physical_docking.md).

The harnesses fly one known POI outside the NAV state machine, so nothing
there arms the contact sensor. This wires the sim contact sensor on the
vehicle's position stream, with the harness POI as the only dock, and arms it
once the first frame is recorded for guidance. Completion is then the
same as in the app: `poi_passed_override()` turns true only on contact.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Protocol

from navpy.core.approach.dock_contact_episode import GuidanceDockContact
from navpy.core.approach.navigation_guidance import (
    GuidanceNavigationService,
)
from navpy.core.kernel.location import Location
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.subscription import Subscription
from navpy.plugins.vehicle.ardupilot.pose_streams import resolve_pose_stream_rate_hz
from navpy.sim.world.dock_contact_feed import SimDockContactFeed
from navpy.sim.world.dock_contact_sensor import SimDockContactSensor, max_segment_s
from navpy.sim.world.simulation_object import SimulationObject


class HarnessVehicle(Protocol):
    @property
    def target_system(self) -> int: ...

    def on_message(
        self, message_name: str, callback: Callable[[Any], None],
    ) -> Subscription: ...


class HarnessDockContact:
    """The harness POI as the one sim dock, armed for the recorded task."""

    def __init__(
        self,
        vehicle: HarnessVehicle,
        poi: Location,
        guidance: GuidanceNavigationService,
        logger: ILogger,
    ) -> None:
        self._poi = poi
        self._docks: tuple[SimulationObject, ...] = ()
        rate_hz = resolve_pose_stream_rate_hz(vehicle)
        sensor = SimDockContactSensor(lambda: self._docks, rate_hz)
        self._contact = GuidanceDockContact(
            sensor,
            lambda: vehicle.target_system,
            guidance.begin_dock_episode,
            guidance.latch_docked,
            logger,
        )
        self._feed = SimDockContactFeed(sensor, vehicle.on_message, max_segment_s(rate_hz))

    def arm(self, detection: DetectedObject) -> None:
        """Arm for this detection's task; its obj id names the dock."""
        self._docks = (
            SimulationObject(detection.identity.obj_id, self._poi, 0),
        )
        self._contact.arm(detection)

    def close(self) -> None:
        self._contact.disarm()
        self._feed.close()


def recording_dispatch(
    guidance: GuidanceNavigationService,
    nav: Callable[[DetectedObject], bool],
    dock_contact: HarnessDockContact,
) -> Callable[[DetectedObject], bool]:
    """Record the first frame for guidance, arm contact, then navigate."""
    recorded = False

    def dispatch(detection: DetectedObject) -> bool:
        nonlocal recorded
        if not recorded:
            recorded = guidance.record_confirmed_detection(detection)
            if not recorded:
                return False
            dock_contact.arm(detection)
        return nav(detection)

    return dispatch


__all__ = ["HarnessDockContact", "recording_dispatch"]
