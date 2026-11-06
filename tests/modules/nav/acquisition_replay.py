"""Seeded acquisition replays through the real navigation composition (HITL step 25).

Each tick runs the real ``NavigationCycle`` (sense, decide, act) of a rig built by ``create_nav_test_rig``
with the HITL gate settings and ``ApproachKind.ORBIT``. Only the world is scripted: the detector's
frames (search, own detection, image error converging as the gimbal locks, missed and invalid frames,
a blackout) and the aircraft's GPS position (straight lane, then the commanded circle). The orbit
planner is stubbed to a 300 m circle (step 23 owns the planner). Truth and compass heading can be
poisoned throughout. Every clock is the replay's own time: nothing reads the wall clock.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from navpy.core.flow.nav_state import NavState
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.detect_response import DetectResponse
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from tests.detection_factory import make_detected_poi
from tests.modules.nav.nav_test_rig import create_nav_test_rig
from tests.modules.nav.test_nav_controller import (
    _create_mock_args,
    _create_mock_detector,
    _create_mock_navigation,
    _create_mock_vehicle,
)

GATES = {
    "lock_dwell_s": 2.0,
    "lock_max_image_error_deg": 1.0,
    "lock_max_frame_gap_s": 0.5,
    "orbit_radial_tolerance_m": 25.0,
    "orbit_dwell_s": 10.0,
}
DT_S = 0.1
FOCAL_PX = 500.0  # tests.detection_factory's camera matrix
CX_PX, CY_PX = 320.0, 240.0
RADIUS_M = 300.0
HOME = (40.0, -74.0)
OWN_CENTRE = Location(40.0027, -74.0, 0.0, is_absolute=False)  # the own-detected pose (ground ray)
TRUTH_POISON = Location(40.05, -73.9, 55.0, is_absolute=False)  # 6 km off: any use shows at once
M_PER_DEG_LAT = 111_320.0


def offset(centre: Location, north_m: float, east_m: float, alt: float = 80.0, heading: float = 0.0) -> Location:
    return Location(
        centre.lat + north_m / M_PER_DEG_LAT,
        centre.lng + east_m / (M_PER_DEG_LAT * math.cos(math.radians(centre.lat))),
        alt, heading=heading, is_absolute=False,
    )


def frame(ts: float, error_deg: float, now, *, atomic: bool = True, truth: bool = False, **kw):
    """One detection with the POI ``error_deg`` off the optical axis."""
    return make_detected_poi(
        obj_id=1, x_error=CX_PX + FOCAL_PX * math.tan(math.radians(error_deg)), y_error=CY_PX,
        timestamp=ts, camera_frame_timestamp_s=ts, timestamp_now_s=now, pose_is_frame_atomic=atomic,
        t_g_loc_debug=TRUTH_POISON if truth else None, is_simulation=truth, **kw,
    )


@dataclass
class World:
    """The scripted detector and aircraft one tick at a time."""

    t: float = 0.0
    detections: list = field(default_factory=list)
    position: Location = field(default_factory=lambda: offset(OWN_CENTRE, -900.0, 0.0))
    heading_deg: float = 0.0


class AcquisitionRig:
    """The real navigation graph with the gate profile, driven by a ``World``."""

    def __init__(self, *, profile: dict | None = None, truth: bool = False, compass_seed: int | None = None,
                 approach_kind: ApproachKind = ApproachKind.ORBIT) -> None:
        self.world = World()
        self.truth = truth
        self._compass = None if compass_seed is None else np.random.default_rng(compass_seed)
        vehicle = _create_mock_vehicle()
        vehicle.location = Mock(side_effect=self._location)
        type(vehicle).attitude = property(lambda _self: self._attitude())
        detector = _create_mock_detector()
        detector.is_simulation = truth
        detector.is_detection_armed = True
        detector.get_detect_data = Mock(side_effect=lambda _request: DetectResponse(
            list(self.world.detections),
            primary_poi=self.world.detections[0] if self.world.detections else None,
        ))
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.legacy_pois.ground_location = Mock(return_value=OWN_CENTRE)
        self.navigation = navigation
        self.rig = create_nav_test_rig(
            vehicle, detector, navigation, _create_mock_args(auto_confirm=True), Mock(),
            approach_kind=approach_kind,
            vision_profile={"final_approach_gates": GATES} if profile is None else profile,
        )
        self.rig.clock.decision_s = lambda: self.world.t
        self.sequence = self.rig.confirmation_action._acquisition
        orbit_start = getattr(self.sequence, "_orbit_start", None)
        if orbit_start is not None:
            orbit_start._ports = dataclasses.replace(
                orbit_start._ports,
                plan_orbit=lambda *_a, **_k: (SimpleNamespace(orbit_radius=RADIUS_M), 80.0),
                board_centre=lambda _poi: None,
            )

    def now(self) -> float:
        return self.world.t

    def _location(self, _relative: bool) -> Location:
        position = self.world.position
        if self._compass is None:
            return position
        return dataclasses.replace(position, heading=float(self._compass.uniform(-720.0, 720.0)))

    def _attitude(self) -> Attitude:
        yaw = 0.0 if self._compass is None else float(self._compass.uniform(-180.0, 180.0))
        return Attitude(0.0, 0.0, yaw)

    def tick(self) -> None:
        self.rig.cycle.run()

    @property
    def phase(self) -> NavState:
        return self.rig.phase.current

    def confirmed(self) -> bool:
        active = self.rig.confirmation_manager.active_poi
        return active is not None and self.rig.confirmation_manager.get_status(active) is ConfirmationStatus.CONFIRMED

    def logged(self, prefix: str) -> bool:
        return any(str(call.args[0]).startswith(prefix) for call in self.rig.logger.info.call_args_list if call.args)

    def orbit_calls(self) -> list:
        return self.navigation.vehicle_commands.peer_poi_loiter.call_args_list


@dataclass
class Replay:
    seed: int
    ticks: list  # per tick: t, phase, confirmed, new_valid, error_deg, frame_ts, radial_m
    events: dict  # first time of each stage
    orbit_centre: Location | None


def run_replay(seed: int, *, truth: bool = False, compass: bool = False, max_s: float = 150.0) -> Replay:
    """One seeded replay; stops one tick after NAV is entered."""
    rng = np.random.default_rng(seed)
    search_s = rng.uniform(2.0, 6.0)
    error0, tau = rng.uniform(3.0, 8.0), rng.uniform(0.5, 1.5)
    blackout_at, blackout_s = search_s + rng.uniform(3.0, 25.0), rng.uniform(0.6, 1.2)
    radial0, bearing0 = rng.uniform(60.0, 150.0), rng.uniform(0.0, 2 * math.pi)
    replay = AcquisitionRig(truth=truth, compass_seed=seed + 1000 if compass else None)
    world = replay.world
    last = None
    ticks, events = [], {}
    orbit_at = None
    for step in range(int(max_s / DT_S)):
        world.t = round(step * DT_S, 6)
        new_valid, error, radial = False, None, None
        if world.t < search_s or blackout_at <= world.t < blackout_at + blackout_s:
            world.detections = []
        elif last is not None and rng.random() < 0.1:
            world.detections = [last]  # missed frame: the tracker still holds the last one
        else:
            error = error0 * math.exp(-(world.t - search_s) / tau) + abs(rng.normal(0.0, 0.15))
            atomic = rng.random() >= 0.03
            last = frame(world.t, error, replay.now, atomic=atomic, truth=truth)
            world.detections = [last]
            new_valid = atomic
            events.setdefault("detected", world.t)
        if orbit_at is not None:
            radial = radial0 * math.exp(-(world.t - orbit_at) / 8.0) + rng.normal(0.0, 4.0)
            bearing = bearing0 + 24.0 / RADIUS_M * (world.t - orbit_at)
            world.position = offset(OWN_CENTRE, (RADIUS_M + radial) * math.cos(bearing),
                                    (RADIUS_M + radial) * math.sin(bearing))
        replay.tick()
        if orbit_at is None and replay.orbit_calls():
            orbit_at = world.t  # commanded on this tick; the aircraft turns onto it from the next
            events["orbit"] = world.t
        for stage in ("locked", "established"):
            if stage not in events and replay.logged(f"ACQ {stage}:"):
                events[stage] = world.t
        confirmed = replay.confirmed()
        if confirmed:
            events.setdefault("confirmed", world.t)
        ticks.append(dict(t=world.t, phase=replay.phase, confirmed=confirmed, new_valid=new_valid,
                          error_deg=error, frame_ts=None if not world.detections else world.detections[0].pixel.source_timestamp_s,
                          radial_m=radial))
        if replay.phase is NavState.NAV:
            events["nav"] = world.t
            break
    calls = replay.orbit_calls()
    return Replay(seed, ticks, events, calls[0].args[0] if calls else None)


def oracle_allows(ticks: list, index: int) -> bool:
    """Independent check from the replay's own log: may confirm or NAV happen at tick ``index``?

    Orbit: every tick of the last orbit dwell had the scripted radial error within the tolerance.
    Lock: the chain of new frames ending at the newest one is unbroken (each valid, within the image
    error, gaps within the frame gap) and spans the lock dwell; this tick still shows a frame.
    """
    t_end = ticks[index]["t"]
    window = [tick for tick in ticks[: index + 1] if tick["t"] >= t_end - GATES["orbit_dwell_s"] - 1e-9]
    if window[0]["t"] > t_end - GATES["orbit_dwell_s"] + 1e-9 or any(
        tick["radial_m"] is None or abs(tick["radial_m"]) > GATES["orbit_radial_tolerance_m"] for tick in window
    ):
        return False
    if ticks[index]["frame_ts"] is None:
        return False
    new = [tick for tick in ticks[: index + 1] if tick["error_deg"] is not None]
    chain = []
    for tick in reversed(new):
        if not tick["new_valid"] or tick["error_deg"] > GATES["lock_max_image_error_deg"]:
            break
        if chain and chain[-1]["t"] - tick["t"] > GATES["lock_max_frame_gap_s"] + 1e-9:
            break
        chain.append(tick)
    if not chain or chain[0] is not new[-1]:
        return False
    return chain[0]["t"] - chain[-1]["t"] >= GATES["lock_dwell_s"] - 1e-9


__all__ = ["AcquisitionRig", "GATES", "OWN_CENTRE", "Replay", "World", "frame", "offset", "oracle_allows",
           "run_replay"]
