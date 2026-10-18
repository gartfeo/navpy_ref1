"""d1: the simulator contact sensor, the stand-in for the dock's net."""

from __future__ import annotations

import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.app.perception.real_detector_factory import RealDetectorFactory
from navpy.app.perception.sim_detector_factory import SimDetectorFactory
from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import DockContactIdentity
from navpy.core.ports.subscription import Subscription
from navpy.core.ports.pose_stream_rate import POSE_STREAM_RATE_HZ
from navpy.sim.world.dock_contact_feed import (
    POSITION_MESSAGE,
    SimDockContactFeed,
    position_sample,
)
from navpy.sim.world.dock_contact_geometry import (
    SIM_DOCK_CONTACT_BANK_LIMIT_DEG,
    DockFrame,
    chord_bow_bound_m,
    flat_earth_error_bound_m,
    flat_segment_distance,
)
from navpy.sim.world.dock_contact_sensor import (
    SIM_DOCK_CONTACT_MAX_SEGMENT_PERIODS,
    SIM_DOCK_NET_RADIUS_ENV,
    SIM_DOCK_NET_RADIUS_M,
    PositionSample,
    SimDockContactSensor,
    max_segment_s,
    sim_dock_net_radius_m,
)
from navpy.sim.world.simulation_object import SimulationObject

LAT, LON, ALT = 40.0, 44.0, 100.0
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(LAT))
IDENTITY = DockContactIdentity(vehicle=7, task_id=3, dock_uid=1, episode=4)
MAX_SEGMENT_S = max_segment_s(POSE_STREAM_RATE_HZ)  # 0.05 s at 40 Hz


def _s(time_s: float, location: Location, speed_mps: float = 25.0) -> PositionSample:
    return PositionSample(time_s, location, speed_mps)


def _at(north_m: float, east_m: float = 0.0, up_m: float = 0.0) -> Location:
    return Location(
        LAT + north_m / M_PER_DEG_LAT,
        LON + east_m / M_PER_DEG_LON,
        ALT + up_m,
        is_absolute=True,
    )


def _docks(*extra: SimulationObject) -> list[SimulationObject]:
    return [
        SimulationObject(0, _at(0.0, 30.0), 10),  # another UAV's dock
        SimulationObject(1, _at(0.0), 10),        # this UAV's own dock
        *extra,
    ]


class _World:
    def __init__(self) -> None:
        self.docks = _docks()

    def __call__(self) -> list[SimulationObject]:
        return self.docks


def _armed(world=None, identity=IDENTITY):
    world = world or _World()
    sensor = SimDockContactSensor(world)
    events = []
    sensor.arm(identity, events.append)
    return sensor, events, world


def _fly_past(sensor, east_m: float, *, t0: float = 10.0, dt: float = 0.025,
              up_m: float = 0.0) -> None:
    """A straight pass from 2 m short to 2 m past the dock, 25 m/s."""
    for index in range(7):
        sensor.sample(_s(t0 + index * dt, _at(-2.0 + index * 0.625, east_m, up_m)))


def test_net_radius_and_segment_constants():
    assert SIM_DOCK_NET_RADIUS_M == 0.5
    assert SIM_DOCK_CONTACT_MAX_SEGMENT_PERIODS == 2.0
    assert SIM_DOCK_CONTACT_BANK_LIMIT_DEG == 45.0
    assert MAX_SEGMENT_S == pytest.approx(2.0 / POSE_STREAM_RATE_HZ)


def test_a_segment_through_the_net_docks_once_with_its_closest_distance():
    sensor, events, _ = _armed()
    _fly_past(sensor, east_m=0.2)
    _fly_past(sensor, east_m=0.0, t0=20.0)  # published once per arming

    [event] = events
    assert event.identity == IDENTITY
    # The first segment that enters the net: from -0.75 m to -0.125 m north,
    # 0.2 m east, so its closest point is hypot(0.2, 0.125) = 0.236 m.
    assert event.closest_3d_m == pytest.approx(math.hypot(0.2, 0.125), abs=0.01)
    assert event.closest_h_m == pytest.approx(event.closest_3d_m, abs=1e-3)
    assert event.closest_3d_m < SIM_DOCK_NET_RADIUS_M


def test_the_crossing_between_two_samples_counts():
    """Samples 2.5 m either side of the dock: only the segment touches it."""
    sensor, events, _ = _armed()
    sensor.sample(_s(1.0, _at(-1.25, 0.1)))
    sensor.sample(_s(1.04, _at(1.25, 0.1)))
    assert len(events) == 1


@pytest.mark.parametrize(("east_m", "up_m"), ((0.51, 0.0), (0.0, 0.51), (0.4, 0.4)))
def test_just_outside_the_net_does_not_dock(east_m, up_m):
    sensor, events, _ = _armed()
    _fly_past(sensor, east_m=east_m, up_m=up_m)
    assert events == []


def test_a_zero_radius_override_makes_a_dead_centre_pass_a_miss():
    """The forced-miss check (missed_dock.md): radius 0 never docks."""
    sensor = SimDockContactSensor(_World(), net_radius_m=0.0)
    events = []
    sensor.arm(IDENTITY, events.append)
    _fly_past(sensor, east_m=0.0)
    assert events == []


def test_a_wider_radius_override_docks_outside_the_default_net():
    sensor = SimDockContactSensor(_World(), net_radius_m=1.0)
    events = []
    sensor.arm(IDENTITY, events.append)
    _fly_past(sensor, east_m=0.8)
    assert len(events) == 1


@pytest.mark.parametrize(
    ("env", "expected"),
    (({}, SIM_DOCK_NET_RADIUS_M), ({SIM_DOCK_NET_RADIUS_ENV: " "}, SIM_DOCK_NET_RADIUS_M),
     ({SIM_DOCK_NET_RADIUS_ENV: "0"}, 0.0), ({SIM_DOCK_NET_RADIUS_ENV: "0.25"}, 0.25)),
)
def test_the_net_radius_reads_the_override(env, expected):
    assert sim_dock_net_radius_m(env) == expected


@pytest.mark.parametrize("raw", ("-0.1", "nan", "inf", "wide", "0.75"))
def test_an_invalid_net_radius_is_refused(raw):
    with pytest.raises(ValueError):
        sim_dock_net_radius_m({SIM_DOCK_NET_RADIUS_ENV: raw})


@pytest.mark.parametrize("radius_m", (-0.1, math.nan, math.inf))
def test_the_sensor_refuses_an_invalid_net_radius(radius_m):
    with pytest.raises(ValueError):
        SimDockContactSensor(_World(), net_radius_m=radius_m)


def test_another_uavs_dock_never_docks():
    sensor, events, _ = _armed()
    _fly_past(sensor, east_m=30.0)  # straight through dock uid 0
    assert events == []


def test_unarmed_or_disarmed_sensor_never_docks():
    world = _World()
    sensor = SimDockContactSensor(world)
    _fly_past(sensor, east_m=0.0)
    sensor.arm(IDENTITY, Mock())
    sensor.disarm()
    _fly_past(sensor, east_m=0.0, t0=20.0)  # no callback to call, no error


@pytest.mark.parametrize("second_time", (1.0, 0.5))
def test_stale_or_backwards_samples_never_create_a_crossing(second_time):
    sensor, events, _ = _armed()
    sensor.sample(_s(1.0, _at(-1.25, 0.1)))
    sensor.sample(_s(second_time, _at(1.25, 0.1)))  # duplicate time or clock back
    assert events == []


def test_aircraft_clock_back_starts_a_new_segment():
    sensor, events, _ = _armed()
    sensor.sample(_s(5.0, _at(-1.25, 0.1)))
    sensor.sample(_s(0.1, _at(-1.0, 3.0)))   # reboot: becomes the new start
    sensor.sample(_s(0.15, _at(1.25, 0.1)))   # joined only to the post-reboot sample
    assert events == []


def test_a_gap_longer_than_the_segment_limit_is_never_joined():
    sensor, events, _ = _armed()
    sensor.sample(_s(1.0, _at(-1.25, 0.1)))
    sensor.sample(_s(1.0 + MAX_SEGMENT_S + 0.001, _at(1.25, 0.1)))
    assert events == []


def test_an_invalid_sample_breaks_the_segment():
    sensor, events, _ = _armed()
    sensor.sample(_s(1.0, _at(-1.25, 0.1)))
    sensor.sample(_s(1.02, Location(float("nan"), LON, ALT, is_absolute=True)))
    sensor.sample(_s(1.04, _at(1.25, 0.1)))
    assert events == []


def test_rearming_resets_and_never_joins_across_the_reset():
    sensor, events, world = _armed()
    sensor.sample(_s(1.0, _at(-1.25, 0.1)))
    sensor.arm(IDENTITY, events.append)  # new assignment / approach restart
    sensor.sample(_s(1.04, _at(1.25, 0.1)))
    assert events == []


def test_a_world_reload_never_joins_across_the_reload():
    sensor, events, world = _armed()
    sensor.sample(_s(1.0, _at(-1.25, 0.1)))
    world.docks = _docks()  # refresh places new dock objects
    sensor.sample(_s(1.04, _at(1.25, 0.1)))
    assert events == []


def test_the_dock_altitude_is_absolute():
    """A pass 10 m above the dock's AMSL position is a miss."""
    sensor, events, _ = _armed()
    _fly_past(sensor, east_m=0.0, up_m=10.0)
    assert events == []


def test_the_callback_runs_outside_the_sensor_lock():
    """The latch may disarm (NAV exit on another thread) without deadlock."""
    world = _World()
    sensor = SimDockContactSensor(world)
    done = threading.Event()

    def on_docked(_event):
        worker = threading.Thread(target=sensor.disarm)
        worker.start()
        worker.join(timeout=2.0)
        done.set()

    sensor.arm(IDENTITY, on_docked)
    _fly_past(sensor, east_m=0.0)
    assert done.is_set()


def _position(boot_ms: int, location: Location) -> SimpleNamespace:
    return SimpleNamespace(
        time_boot_ms=boot_ms,
        lat=int(round(location.lat * 1e7)),
        lon=int(round(location.lng * 1e7)),
        alt=int(round(location.alt * 1e3)),
        vx=2400, vy=700, vz=0,
    )


def test_position_sample_decodes_global_position_int_at_amsl():
    sample = position_sample(_position(12_345, _at(0.0)))
    assert sample is not None
    assert sample.aircraft_time_s == pytest.approx(12.345)
    assert sample.location.alt == pytest.approx(ALT)
    assert sample.location.is_absolute
    assert sample.ground_speed_mps == pytest.approx(25.0)


@pytest.mark.parametrize(
    "message",
    (SimpleNamespace(), SimpleNamespace(time_boot_ms=True, lat=1, lon=1, alt=1)),
)
def test_position_sample_rejects_malformed_messages(message):
    assert position_sample(message) is None


def test_feed_samples_every_position_update_independent_of_detection():
    sensor, events, _ = _armed()
    callbacks = {}

    def subscribe(name, callback):
        callbacks[name] = callback
        return Subscription(lambda: callbacks.pop(name))

    feed = SimDockContactFeed(sensor, subscribe)
    assert set(callbacks) == {POSITION_MESSAGE}
    callbacks[POSITION_MESSAGE](_position(1_000, _at(-1.25, 0.1)))
    callbacks[POSITION_MESSAGE](_position(1_040, _at(1.25, 0.1)))
    assert len(events) == 1
    feed.close()
    assert callbacks == {}


def _sim_factory(simulated: bool) -> SimDetectorFactory:
    factory = object.__new__(SimDetectorFactory)
    factory._vehicle = Mock(
        is_simulated_autopilot=Mock(return_value=simulated),
        on_message=Mock(return_value=Subscription(lambda: None)),
    )
    factory._logger = Mock()
    factory._dock_contact = None
    factory._dock_contact_feed = None
    return factory


def test_sim_composition_wires_one_sensor_for_a_sim_autopilot():
    factory = _sim_factory(simulated=True)
    detector = Mock(sim_docks=Mock(return_value=()))
    factory._wire_dock_contact(detector)
    sensor = factory.dock_contact
    factory._wire_dock_contact(Mock())  # a second mount reuses it

    assert isinstance(sensor, SimDockContactSensor)
    assert factory.dock_contact is sensor
    factory._vehicle.on_message.assert_called_once()
    assert factory._vehicle.on_message.call_args.args[0] == POSITION_MESSAGE


def test_sim_composition_applies_and_logs_the_net_radius_override(monkeypatch):
    monkeypatch.setenv(SIM_DOCK_NET_RADIUS_ENV, "0")
    factory = _sim_factory(simulated=True)
    factory._wire_dock_contact(Mock(sim_docks=Mock(return_value=())))
    factory._logger.info.assert_called_once_with("SIM dock net radius: 0 m")
    assert factory.dock_contact._net_radius_m == 0.0


def test_sim_detector_on_a_real_autopilot_has_no_sensor():
    factory = _sim_factory(simulated=False)
    factory._wire_dock_contact(Mock())
    assert factory.dock_contact is None
    factory._vehicle.on_message.assert_not_called()


def test_the_real_build_has_no_sensor():
    assert object.__new__(RealDetectorFactory).dock_contact is None


def test_a_gap_the_old_allowance_joined_is_no_longer_joined():
    """Codex P2: 0.25 s at 25 m/s is a 6.25 m chord; a curved path between
    such samples could miss by 0.52 m while the chord touches the net."""
    sensor, events, _ = _armed()
    sensor.sample(_s(1.0, _at(-3.125, 0.45)))
    sensor.sample(_s(1.25, _at(3.125, 0.45)))
    assert events == []


def test_chord_bow_bound_covers_a_bank_limited_turn():
    """25 m/s at 45 deg bank turns on a ~63.7 m radius."""
    radius_m = 25.0 ** 2 / (9.80665 * math.tan(math.radians(45.0)))
    chord_m = 1.25
    expected = radius_m - math.sqrt(radius_m ** 2 - (chord_m / 2.0) ** 2)
    assert chord_bow_bound_m(chord_m, 25.0) == pytest.approx(expected)
    # A turn tighter than half the chord is not flyable: bounded by it.
    assert chord_bow_bound_m(chord_m, 0.0) == pytest.approx(chord_m / 2.0)


SPEED_MPS = 25.0
TURN_RADIUS_M = SPEED_MPS ** 2 / (9.80665 * math.tan(math.radians(45.0)))
TURN_DT_S = 0.99 * MAX_SEGMENT_S  # one missed sample, inside the limit


def _turn(apex_miss_m: float, *, dock_inside: bool) -> tuple[Location, Location]:
    """Two samples on a 45 deg bank turn at 25 m/s whose arc apex passes
    ``apex_miss_m`` west of the dock. With the dock inside the turn the
    chord is the sagitta closer to it than the arc; outside, farther."""
    half_angle = SPEED_MPS * TURN_DT_S / 2.0 / TURN_RADIUS_M
    side = 1.0 if dock_inside else -1.0
    centre_east = -apex_miss_m + side * TURN_RADIUS_M
    points = [
        (
            TURN_RADIUS_M * math.sin(angle),
            centre_east - side * TURN_RADIUS_M * math.cos(angle),
        )
        for angle in (-half_angle, half_angle)
    ]
    return _at(*points[0]), _at(*points[1])


def _fly_turn(start: Location, end: Location):
    sensor, events, _ = _armed()
    sensor.sample(_s(1.0, start, SPEED_MPS))
    sensor.sample(_s(1.0 + TURN_DT_S, end, SPEED_MPS))
    return events


def test_a_curved_miss_just_outside_the_net_never_docks():
    """Codex P2: the arc misses by 0.501 m, but the dock is inside the turn,
    so the chord passes ~0.498 m away, inside the net. Only the chord-bow
    bound (closest + bow < radius) keeps this miss from becoming contact."""
    start, end = _turn(0.501, dock_inside=True)
    sagitta_m = TURN_RADIUS_M * (
        1.0 - math.cos(SPEED_MPS * TURN_DT_S / 2.0 / TURN_RADIUS_M)
    )
    assert 0.501 - sagitta_m < SIM_DOCK_NET_RADIUS_M  # the chord alone docks
    assert _fly_turn(start, end) == []


def test_a_curved_miss_with_the_dock_outside_the_turn_never_docks():
    assert _fly_turn(*_turn(0.501, dock_inside=False)) == []


@pytest.mark.parametrize("dock_inside", (True, False))
def test_a_turn_well_inside_the_net_docks(dock_inside):
    events = _fly_turn(*_turn(0.4, dock_inside=dock_inside))
    assert len(events) == 1
    assert events[0].closest_3d_m < SIM_DOCK_NET_RADIUS_M


# --- reader-thread cost: a flat coarse filter that never misses a contact --


@pytest.mark.parametrize("lat_deg", (0.0, 40.0, 70.0))
def test_flat_frame_agrees_with_exact_geometry_within_its_bound(lat_deg):
    import random

    from navpy.core.approach.logs.navigation_snap_geometry import (
        closest_point_components_on_segment,
    )
    dock = SimulationObject(1, Location(lat_deg, 44.0, 300.0, is_absolute=True), 10)
    frame = DockFrame.at(dock)
    m_lat = frame.north_m_per_deg
    m_lon = frame.east_m_per_deg
    rng = random.Random(1234)
    for scale_m in (2.0, 20.0, 500.0):
        for _ in range(200):
            a, b = (
                Location(
                    lat_deg + rng.uniform(-scale_m, scale_m) / m_lat,
                    44.0 + rng.uniform(-scale_m, scale_m) / m_lon,
                    300.0 + rng.uniform(-scale_m, scale_m) / 4.0,
                    is_absolute=True,
                )
                for _ in range(2)
            )
            local_a, local_b = frame.local(a), frame.local(b)
            exact = closest_point_components_on_segment(a, b, dock.g_loc).slant
            flat = flat_segment_distance(local_a, local_b)
            reach = math.dist((0, 0, 0), local_a) + math.dist((0, 0, 0), local_b)
            assert abs(flat - exact) <= flat_earth_error_bound_m(reach, frame.abs_tan_lat)


def _count_exact(sensor):
    calls = []
    original = sensor._exact_contact

    def counting(*args):
        calls.append(args)
        return original(*args)

    sensor._exact_contact = counting
    return calls


def _in_frame(north_m: float, east_m: float) -> Location:
    """A point in the sensor's own flat frame at this UAV's dock."""
    frame = DockFrame.at(_docks()[1])
    return Location(
        LAT + north_m / frame.north_m_per_deg,
        LON + east_m / frame.east_m_per_deg,
        ALT,
        is_absolute=True,
    )


@pytest.mark.parametrize(("offset_m", "docks"), ((0.499, True), (0.5005, False)))
def test_the_coarse_filter_edge_still_runs_the_exact_test(offset_m, docks):
    """Both passes are inside radius + flat-earth bound (the bound is at
    least 1 mm), so both reach the exact test; contact needs exact closest
    + bow (~0.7 mm here) < 0.5 m, so only the first docks."""
    sensor, events, _ = _armed()
    calls = _count_exact(sensor)
    sensor.sample(_s(1.0, _in_frame(-0.3, offset_m), 25.0))
    sensor.sample(_s(1.024, _in_frame(0.3, offset_m), 25.0))
    assert len(calls) == 1
    assert len(events) == (1 if docks else 0)


def test_far_and_just_outside_samples_never_run_the_exact_test():
    sensor, events, _ = _armed()
    calls = _count_exact(sensor)
    for index in range(40):  # 25 m/s, 40 Hz, a kilometre out
        sensor.sample(_s(1.0 + index * 0.025, _at(-1000.0 + index * 0.625, 30.0)))
    sensor.sample(_s(3.0, _in_frame(-0.3, 0.502)))  # just past radius + bound
    sensor.sample(_s(3.024, _in_frame(0.3, 0.502)))
    assert calls == []
    assert events == []
