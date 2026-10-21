"""The sim closest-approach record (docs/design/gcs_sim_closest_approach.md).

The contact sensor keeps each episode's exact closest pass to the own dock and
reports it once when the lifecycle ends the episode, with the lifecycle's
accepted outcome. Display evidence only.
"""

from __future__ import annotations

import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.core.approach.dock_contact_episode import FinalApproachDockContact
from navpy.core.approach.logs.navigation_snap_geometry import (
    calc_distance,
    closest_point_components_on_segment,
)
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import DockContactIdentity
from navpy.core.ports.subscription import Subscription
from navpy.sim.world import closest_approach_record as record_module
from navpy.sim.world.closest_approach_record import (
    NAVPY_SIM_CLOSEST_APPROACH_MARKER,
    OUTCOME_DOCKED,
    OUTCOME_MISSED,
    encode,
    from_mapping,
    parse,
    to_mapping,
)
from navpy.sim.world.dock_contact_feed import (
    ATTITUDE_MESSAGE,
    POSITION_MESSAGE,
    SimDockContactFeed,
    attitude_sample,
)
from navpy.sim.world.dock_contact_sensor import (
    PositionSample,
    SimDockContactSensor,
    max_segment_s,
)
from navpy.core.ports.pose_stream_rate import POSE_STREAM_RATE_HZ
from navpy.sim.world.simulation_object import SimulationObject

LAT, LON, ALT = 40.0, 44.0, 100.0
HOME_AMSL = 60.0
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(LAT))
IDENTITY = DockContactIdentity(vehicle=7, task_id=3, dock_uid=1, episode=4)
DT = 0.025  # 40 Hz
MAX_AGE_S = max_segment_s(POSE_STREAM_RATE_HZ)


def _at(north_m: float, east_m: float = 0.0, up_m: float = 0.0) -> Location:
    return Location(
        LAT + north_m / M_PER_DEG_LAT,
        LON + east_m / M_PER_DEG_LON,
        ALT + up_m,
        is_absolute=True,
    )


def _s(time_s, location, *, attitude=None):
    return PositionSample(
        time_s, location, 25.0,
        relative_alt_m=location.alt - HOME_AMSL,
        attitude=attitude,
    )


def _world():
    # One set of dock objects: a new object is a world reload (never joined).
    docks = [SimulationObject(0, _at(0.0, 30.0), 10), SimulationObject(1, _at(0.0), 10)]
    return lambda: docks


def _flat_reports():
    """Reports read by their flat wire keys."""
    reports = []
    return reports, lambda record: reports.append(SimpleNamespace(**to_mapping(record)))


def _sensor(report=None):
    reports, flat = _flat_reports()
    sensor = SimDockContactSensor(_world(), report=report or flat)
    events = []
    sensor.arm(IDENTITY, events.append)
    return sensor, reports, events


def _fly(sensor, points, *, t0=10.0, attitude=None):
    """One sample per point, 25 ms apart (north, east, up in metres)."""
    for index, (north_m, east_m, up_m) in enumerate(points):
        sensor.sample(_s(t0 + index * DT, _at(north_m, east_m, up_m), attitude=attitude))


def _line(east_m, up_m=0.0, *, start=-4.0, step=0.625, count=13):
    return [(start + i * step, east_m, up_m) for i in range(count)]


def test_a_miss_reports_the_episode_minimum_once_at_disarm():
    sensor, reports, events = _sensor()
    _fly(sensor, _line(east_m=1.5, up_m=0.8))
    assert reports == [] and events == []  # nothing before the end

    sensor.disarm(docked=False)
    sensor.disarm(docked=False)  # once

    [record] = reports
    assert record.outcome == OUTCOME_MISSED
    assert record.d3_m == pytest.approx(math.hypot(1.5, 0.8), abs=0.01)
    assert record.h_m == pytest.approx(1.5, abs=0.01)
    assert record.v_m == pytest.approx(0.8, abs=0.01)
    assert (record.vehicle, record.task_id, record.dock_uid, record.episode) == (7, 3, 1, 4)


def test_the_reported_point_is_on_the_path_at_the_reported_distance():
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=2.0, up_m=-1.0, start=-3.9))
    sensor.disarm()

    [record] = reports
    point = Location(record.lat, record.lon, record.alt_amsl_m, is_absolute=True)
    dock = Location(record.dock_lat, record.dock_lon, record.dock_alt_amsl_m, is_absolute=True)
    assert record.dock_lat == pytest.approx(LAT) and record.dock_lon == pytest.approx(LON)
    assert calc_distance(point, dock) == pytest.approx(record.d3_m, abs=0.01)
    assert record.alt_rel_m == pytest.approx(record.alt_amsl_m - HOME_AMSL)
    assert record.dock_alt_rel_m == pytest.approx(ALT - HOME_AMSL)


def test_the_episode_minimum_is_the_exact_minimum_over_every_segment():
    """A curved pass: the reported distance equals the brute-force exact one."""
    points = [
        (-6.0 + i * 0.6, 3.0 * math.cos(i / 4.0) - 1.0, 0.3 * math.sin(i / 3.0))
        for i in range(21)
    ]
    sensor, reports, _ = _sensor()
    _fly(sensor, points)
    sensor.disarm()

    locations = [_at(*point) for point in points]
    exact = min(
        closest_point_components_on_segment(a, b, _at(0.0)).slant
        for a, b in zip(locations, locations[1:])
    )
    assert reports[0].d3_m == pytest.approx(exact, abs=1e-9)


def test_a_segment_flat_farther_but_exactly_nearer_is_still_kept(monkeypatch):
    """The flat distance only filters: a segment whose flat distance is larger
    than the kept one, but within the flat-earth bound, gets the exact test."""
    from navpy.sim.world import dock_contact_sensor as module

    near, far = 0.6, 0.7
    # Flat ordering opposite to the exact one, within a 0.06 m bound.
    flat_by_east = {far: (0.60, 0.06), near: (0.65, 0.06)}
    monkeypatch.setattr(
        module,
        "_flat_distance",
        lambda previous, current: flat_by_east[
            round(abs(current.local[1]), 1)
        ],
    )
    sensor, reports, _ = _sensor()
    _fly(sensor, [(-0.3, far, 0.0), (0.3, far, 0.0)], t0=1.0)
    _fly(sensor, [(-0.3, near, 0.0), (0.3, near, 0.0)], t0=2.0)
    sensor.disarm()

    exact_near = closest_point_components_on_segment(
        _at(-0.3, near), _at(0.3, near), _at(0.0),
    ).slant
    assert reports[0].d3_m == pytest.approx(exact_near, abs=1e-9)


def test_docked_reports_the_contact_segment_matching_the_docked_event():
    sensor, reports, events = _sensor()
    _fly(sensor, _line(east_m=0.2))
    [event] = events
    sensor.disarm(docked=True)

    [record] = reports
    assert record.outcome == OUTCOME_DOCKED
    assert record.d3_m == event.closest_3d_m
    assert record.h_m == event.closest_h_m and record.v_m == event.closest_v_m


def test_a_contact_the_lifecycle_rejected_reports_missed_with_the_episode_minimum():
    """Contact comes before the closest point; tracking goes on after it."""
    points = _line(east_m=0.2)
    sensor, reports, events = _sensor()
    _fly(sensor, points)
    [event] = events
    sensor.disarm(docked=False)

    locations = [_at(*point) for point in points]
    exact = min(
        closest_point_components_on_segment(a, b, _at(0.0)).slant
        for a, b in zip(locations, locations[1:])
    )
    assert reports[0].outcome == OUTCOME_MISSED
    assert reports[0].d3_m == pytest.approx(exact, abs=1e-9)
    assert event.closest_3d_m > exact + 0.05  # the contact segment was not the minimum


def test_no_joined_segment_reports_nothing():
    sensor, reports, _ = _sensor()
    sensor.sample(_s(1.0, _at(-1.0)))
    sensor.sample(_s(1.5, _at(1.0)))  # a gap: never joined
    sensor.disarm()
    assert reports == []


def test_rearm_reports_nothing_of_the_old_episode_by_itself():
    """The sensor reports on disarm; arm alone starts a clean episode."""
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=3.0))
    sensor.arm(DockContactIdentity(7, 3, 1, 5), lambda _event: None)
    _fly(sensor, _line(east_m=1.0), t0=30.0)
    sensor.disarm()
    [record] = reports
    assert record.episode == 5
    assert record.d3_m == pytest.approx(1.0, abs=0.01)


def test_a_throwing_sink_does_not_break_disarm(caplog):
    def boom(_record):
        raise RuntimeError("sink down")

    sensor, _, _ = _sensor(report=boom)
    _fly(sensor, _line(east_m=1.0))
    sensor.disarm()  # does not raise
    assert "closest-approach report failed" in caplog.text


def test_contact_and_distances_do_not_depend_on_attitude():
    with_attitude, reports_a, events_a = _sensor()
    without, reports_b, events_b = _sensor()
    _fly(with_attitude, _line(east_m=0.2), attitude=Attitude(pitch=2.0, yaw=90.0, roll=-5.0))
    _fly(without, _line(east_m=0.2))
    with_attitude.disarm(docked=True)
    without.disarm(docked=True)
    assert events_a == events_b
    assert reports_a[0].d3_m == reports_b[0].d3_m
    assert (reports_a[0].roll_deg, reports_a[0].pitch_deg, reports_a[0].yaw_deg) == (-5.0, 2.0, 90.0)
    assert reports_b[0].yaw_deg is None


def test_the_attitude_of_the_nearer_segment_end_is_used():
    sensor, reports, _ = _sensor()
    sensor.sample(_s(1.0, _at(-0.2, 1.0), attitude=Attitude(pitch=0.0, yaw=10.0, roll=0.0)))
    sensor.sample(_s(1.025, _at(0.9, 1.0), attitude=Attitude(pitch=0.0, yaw=20.0, roll=0.0)))
    sensor.disarm()
    assert reports[0].yaw_deg == 10.0
    assert reports[0].aircraft_time_s == 1.0


# -- feed: attitude labels positions only near their own time --------------

def _attitude_msg(boot_ms, roll=0.1, pitch=0.05, yaw=1.0):
    return SimpleNamespace(time_boot_ms=boot_ms, roll=roll, pitch=pitch, yaw=yaw)


def _position_msg(boot_ms, location):
    return SimpleNamespace(
        time_boot_ms=boot_ms,
        lat=int(round(location.lat * 1e7)),
        lon=int(round(location.lng * 1e7)),
        alt=int(round(location.alt * 1e3)),
        relative_alt=int(round((location.alt - HOME_AMSL) * 1e3)),
        vx=2500, vy=0, vz=0,
    )


class _Recorder:
    def __init__(self):
        self.samples = []

    def sample(self, sample):
        self.samples.append(sample)


def _feed():
    recorder = _Recorder()
    callbacks = {}

    def subscribe(name, callback):
        callbacks[name] = callback
        return Subscription(lambda: callbacks.pop(name))

    SimDockContactFeed(recorder, subscribe, attitude_max_age_s=MAX_AGE_S)
    return recorder, callbacks


@pytest.mark.parametrize(
    "attitude_ms, expect",
    [
        (1_000, True),                       # same time
        (1_000 - int(MAX_AGE_S * 1000), True),  # at the bound
        (1_000 - int(MAX_AGE_S * 1000) - 5, False),  # stale
        (1_000 + int(MAX_AGE_S * 1000) + 5, False),  # newer: a clock reset
        (None, False),                       # missing
    ],
)
def test_feed_labels_a_position_only_with_a_timely_attitude(attitude_ms, expect):
    recorder, callbacks = _feed()
    if attitude_ms is not None:
        callbacks[ATTITUDE_MESSAGE](_attitude_msg(attitude_ms))
    callbacks[POSITION_MESSAGE](_position_msg(1_000, _at(0.0)))
    [sample] = recorder.samples
    assert (sample.attitude is not None) is expect
    assert sample.relative_alt_m == pytest.approx(ALT - HOME_AMSL)
    if expect:
        assert sample.attitude.yaw == pytest.approx(math.degrees(1.0))


@pytest.mark.parametrize("bad", [math.nan, math.inf, True, "1"])
def test_a_non_finite_attitude_labels_nothing(bad):
    assert attitude_sample(_attitude_msg(1_000, yaw=bad)) is None
    recorder, callbacks = _feed()
    callbacks[ATTITUDE_MESSAGE](_attitude_msg(1_000, yaw=1.0))
    callbacks[ATTITUDE_MESSAGE](_attitude_msg(1_010, yaw=bad))  # replaces the good one
    callbacks[POSITION_MESSAGE](_position_msg(1_010, _at(0.0)))
    assert recorder.samples[0].attitude is None


# -- record ------------------------------------------------------------------

def _record(**overrides):
    return from_mapping(_values(**overrides))


def _values(**overrides):
    values = dict(
        vehicle=2, task_id=7, dock_uid=3, episode=1, outcome=OUTCOME_MISSED,
        aircraft_time_s=512.3, lat=40.0, lon=44.0, alt_amsl_m=101.0,
        d3_m=1.84, h_m=1.52, v_m=1.04, dock_lat=40.00001, dock_lon=44.00001,
        dock_alt_amsl_m=100.0, alt_rel_m=41.0, dock_alt_rel_m=40.0,
        roll_deg=-3.0, pitch_deg=2.0, yaw_deg=91.0,
    )
    values.update(overrides)
    return values


def test_record_round_trips():
    record = _record()
    assert parse(encode(record)) == record
    bare = _record(alt_rel_m=None, dock_alt_rel_m=None, roll_deg=None, pitch_deg=None, yaw_deg=None)
    assert parse(encode(bare)) == bare


def _line_with(**changes):
    payload = json.loads(encode(_record()).split(" ", 1)[1])
    payload.update(changes)
    return f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} {json.dumps(payload)}"


@pytest.mark.parametrize(
    "line",
    [
        "",
        "NAVPY_SIM_CLOSEST_APPROACH_V2 {}",
        f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER}{{}}",
        f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} not json",
        f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} []",
        _line_with(outcome="parked"),
        _line_with(vehicle=True),
        _line_with(vehicle=2.0),
        _line_with(d3_m=None),
        _line_with(d3_m="1.8"),
        _line_with(lat=float("nan")),
        _line_with(yaw_deg=float("inf")),
        _line_with(yaw_deg=None),  # attitude: all three or none
        _line_with(outcome=["missed"]),
        _line_with(outcome={"a": 1}),
        _line_with(d3_m=10 ** 400),
        _line_with(dock_alt_rel_m="40"),
    ],
)
def test_parse_ignores_anything_but_an_exact_record(line):
    assert parse(line) is None


def test_parse_ignores_json_nested_too_deep():
    assert parse(f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} " + "[" * 100_000) is None


def test_missing_field_is_ignored():
    payload = json.loads(encode(_record()).split(" ", 1)[1])
    del payload["h_m"]
    assert parse(f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} {json.dumps(payload)}") is None


def test_publish_goes_through_the_replaceable_publisher(capsys):
    record_module.publish(_record())
    assert parse(capsys.readouterr().out.strip()) == _record()
    got = []
    record_module.set_publisher(got.append)
    try:
        record_module.publish(_record())
    finally:
        record_module.set_publisher(record_module.print_record)
    assert got == [_record()]


# -- lifecycle: the outcome is the accepted latch, decided under its lock ----

def _lifecycle(*, latch=True):
    reports, flat = _flat_reports()
    sensor = SimDockContactSensor(_world(), report=flat)
    latched = []

    def latch_docked(event):
        latched.append(event)
        return latch

    lifecycle = FinalApproachDockContact(
        sensor, lambda: 7, lambda _identity, _s: True, latch_docked, Mock(),
    )
    poi = SimpleNamespace(identity=SimpleNamespace(obj_id=1, task_id=3))
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(
            "navpy.core.approach.dock_contact_episode.get_poi_task_id",
            lambda _poi: 3,
        )
        lifecycle.arm(poi)
    return lifecycle, sensor, reports, latched, poi


def test_an_accepted_contact_reports_docked_at_nav_exit():
    lifecycle, sensor, reports, latched, _ = _lifecycle()
    _fly(sensor, _line(east_m=0.2))
    assert lifecycle.disarm() is not None
    assert [r.outcome for r in reports] == [OUTCOME_DOCKED]
    assert len(latched) == 1


def test_a_refused_latch_reports_missed():
    lifecycle, sensor, reports, _, _ = _lifecycle(latch=False)
    _fly(sensor, _line(east_m=0.2))
    assert lifecycle.disarm() is None
    assert [r.outcome for r in reports] == [OUTCOME_MISSED]


@pytest.mark.parametrize("end", ["disarm", "rearm"])
def test_a_contact_held_past_the_episode_end_is_missed_and_unlatched(end):
    """The sensor saw contact, but its callback reaches the lifecycle only
    after NAV exit (or re-arm) detached the episode."""
    lifecycle, sensor, reports, latched, poi = _lifecycle()
    held = []
    sensor._on_docked = held.append  # hold the callback between the two locks
    _fly(sensor, _line(east_m=0.2))
    [event] = held

    if end == "disarm":
        assert lifecycle.disarm() is None
    else:
        with pytest.MonkeyPatch.context() as patch:
            patch.setattr(
                "navpy.core.approach.dock_contact_episode.get_poi_task_id",
                lambda _poi: 3,
            )
            lifecycle.arm(poi)
    lifecycle._on_docked(event)  # the held callback lands late

    assert latched == []
    assert [r.outcome for r in reports] == [OUTCOME_MISSED]
    assert reports[0].episode == event.identity.episode
