"""The sim closest-approach record (docs/design/gcs_sim_closest_approach.md).

The contact sensor keeps each episode's exact closest pass to the own dock and
reports it once when the lifecycle ends the episode, with the lifecycle's
accepted outcome. Display evidence only.
"""

from __future__ import annotations

import itertools
import json
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.core.approach.dock_contact_episode import (
    FinalApproachDockContact,
    with_dock_evidence_cut,
)
from navpy.core.approach.logs.navigation_snap_geometry import (
    calc_distance,
    closest_point_components_on_segment,
)
from navpy.core.approach.vision_nav.episode_latch import (
    FinalApproachEpisodeLatch,
    FinalApproachTerminal,
)
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import DockContactIdentity, DockEpisodeEnd
from navpy.core.ports.subscription import Subscription
from navpy.sim.world import closest_approach_record as record_module
from navpy.sim.world.closest_approach_tracker import ClosestApproachTracker
from navpy.sim.world.closest_approach_record import (
    NAVPY_SIM_CLOSEST_APPROACH_MARKER,
    OUTCOME_ABORTED,
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
# NAV's own failure after the dock left the image.
FAILED = DockEpisodeEnd(failed=True, abort_reason="no_detection")
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

    sensor.disarm(FAILED)
    sensor.disarm(FAILED)  # once

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
    sensor.disarm(DockEpisodeEnd(docked=True))

    [record] = reports
    assert record.outcome == OUTCOME_DOCKED
    assert record.d3_m == event.closest_3d_m
    assert record.h_m == event.closest_h_m and record.v_m == event.closest_v_m


@pytest.mark.parametrize(
    "end, outcome", [(DockEpisodeEnd(), OUTCOME_ABORTED), (FAILED, OUTCOME_MISSED)],
)
def test_a_contact_the_lifecycle_rejected_reports_the_episode_minimum(end, outcome):
    """Contact comes before the closest point; tracking goes on after it.
    Nothing latched is aborted; NAV's failure after the pass is missed."""
    points = _line(east_m=0.2)
    sensor, reports, events = _sensor()
    _fly(sensor, points)
    [event] = events
    sensor.disarm(end)

    locations = [_at(*point) for point in points]
    exact = min(
        closest_point_components_on_segment(a, b, _at(0.0)).slant
        for a, b in zip(locations, locations[1:])
    )
    assert reports[0].outcome == outcome
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
    with_attitude.disarm(DockEpisodeEnd(docked=True))
    without.disarm(DockEpisodeEnd(docked=True))
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


def test_a_refused_latch_with_nothing_latched_reports_aborted():
    lifecycle, sensor, reports, _, _ = _lifecycle(latch=False)
    _fly(sensor, _line(east_m=0.2))
    assert lifecycle.disarm() is None
    assert [(r.outcome, r.reason) for r in reports] == [(OUTCOME_ABORTED, None)]


@pytest.mark.parametrize("end", ["disarm", "rearm"])
def test_a_contact_held_past_the_episode_end_is_aborted_and_unlatched(end):
    """The sensor saw contact, but its callback reaches the lifecycle only
    after NAV exit (or re-arm) detached the episode: no terminal latched."""
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
    assert [r.outcome for r in reports] == [OUTCOME_ABORTED]
    assert reports[0].episode == event.identity.episode


# -- aborted: NAV ended the approach early (todo 108) -------------------------

def test_an_episode_ended_while_still_closing_is_aborted_without_a_reason():
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=1.0, start=-12.0))  # stops 4.5 m short, closing
    sensor.disarm(DockEpisodeEnd())
    [record] = reports
    assert (record.outcome, record.reason) == (OUTCOME_ABORTED, None)
    assert record.d3_m == pytest.approx(math.hypot(4.5, 1.0), abs=0.01)


def test_nav_failure_while_still_closing_is_aborted_with_its_reason():
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=1.0, start=-12.0))
    sensor.disarm(FAILED)
    assert [(r.outcome, r.reason) for r in reports] == [(OUTCOME_ABORTED, "no_detection")]


@pytest.mark.parametrize(
    "end, expected",
    [
        (DockEpisodeEnd(abort_reason="cancelled"), (OUTCOME_ABORTED, "cancelled")),
        (DockEpisodeEnd(), (OUTCOME_ABORTED, None)),
        (DockEpisodeEnd(failed=True, abort_reason="source_command_stale"), (OUTCOME_MISSED, None)),
    ],
)
def test_after_the_pass_only_navs_own_failure_is_missed(end, expected):
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=1.5))  # passed the dock
    sensor.disarm(end)
    assert [(r.outcome, r.reason) for r in reports] == [expected]


def test_a_latched_miss_is_missed_even_while_closing():
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=1.0, start=-12.0))
    sensor.disarm(DockEpisodeEnd(missed=True))
    assert [(r.outcome, r.reason) for r in reports] == [(OUTCOME_MISSED, None)]


def test_an_abort_reason_that_is_not_an_identifier_is_dropped():
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=1.5))
    sensor.disarm(DockEpisodeEnd(abort_reason="Bad reason!"))
    assert [(r.outcome, r.reason) for r in reports] == [(OUTCOME_ABORTED, None)]


def test_an_aborted_record_round_trips_with_its_reason():
    record = _record(outcome=OUTCOME_ABORTED, reason="cancelled")
    assert record is not None and parse(encode(record)) == record


@pytest.mark.parametrize(
    "outcome, reason",
    [(OUTCOME_MISSED, "cancelled"), (OUTCOME_DOCKED, "cancelled"),
     (OUTCOME_ABORTED, ""), (OUTCOME_ABORTED, "Bad reason"), (OUTCOME_ABORTED, 3),
     (OUTCOME_ABORTED, "x" * 65)],
)
def test_a_reason_is_only_an_identifier_on_an_aborted_record(outcome, reason):
    assert from_mapping(_values(outcome=outcome, reason=reason)) is None


def test_the_lifecycle_passes_how_nav_left_the_episode():
    reports, flat = _flat_reports()
    sensor = SimDockContactSensor(_world(), report=flat)
    ended = []

    def ending(identity, docked):
        ended.append((identity, docked))
        return DockEpisodeEnd(abort_reason="cancelled")

    lifecycle = FinalApproachDockContact(
        sensor, lambda: 7, lambda _identity, _s: True, lambda _e: True, Mock(),
        ending=ending,
    )
    poi = SimpleNamespace(identity=SimpleNamespace(obj_id=1, task_id=3))
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr("navpy.core.approach.dock_contact_episode.get_poi_task_id", lambda _poi: 3)
        lifecycle.arm(poi)
    _fly(sensor, _line(east_m=1.5))
    lifecycle.disarm()
    assert ended == [(DockContactIdentity(vehicle=7, task_id=3, dock_uid=1, episode=1), False)]
    assert [(r.outcome, r.reason) for r in reports] == [(OUTCOME_ABORTED, "cancelled")]


def test_a_failing_ending_falls_back_and_never_breaks_nav_exit():
    reports, flat = _flat_reports()
    sensor = SimDockContactSensor(_world(), report=flat)
    logger = Mock()

    def ending(_identity, _docked):
        raise RuntimeError("no mode session")

    lifecycle = FinalApproachDockContact(
        sensor, lambda: 7, lambda _identity, _s: True, lambda _e: True, logger,
        ending=ending,
    )
    poi = SimpleNamespace(identity=SimpleNamespace(obj_id=1, task_id=3))
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr("navpy.core.approach.dock_contact_episode.get_poi_task_id", lambda _poi: 3)
        lifecycle.arm(poi)
    _fly(sensor, _line(east_m=1.5))
    assert lifecycle.disarm() is None
    assert [r.outcome for r in reports] == [OUTCOME_ABORTED]
    logger.error.assert_called_once()


# -- pass evidence (v6): the dock clearly ahead, then the range opening -------

def _cut_after(sensor, points, count, *, t0=10.0):
    """Fly the first ``count`` points, NAV ends the approach, fly the rest."""
    _fly(sensor, points[:count], t0=t0)
    sensor.end_evidence(IDENTITY)
    _fly(sensor, points[count:], t0=t0 + count * DT)


def test_a_near_miss_cut_one_sample_after_its_closest_point_is_missed():
    sensor, reports, events = _sensor()
    _cut_after(sensor, _line(east_m=0.72), 8)  # the closest point is in 6 -> 7
    sensor.disarm(FAILED)
    [record] = reports
    assert events == []
    assert (record.outcome, record.reason) == (OUTCOME_MISSED, None)
    assert record.d3_m == pytest.approx(0.72, abs=0.01)


def test_a_near_miss_cut_at_its_closest_sample_is_aborted():
    """Later samples before the disarm pass the dock; they never count."""
    sensor, reports, _ = _sensor()
    _cut_after(sensor, _line(east_m=0.72, step=0.5, count=17), 9)  # ends at north 0
    sensor.disarm(FAILED)
    [record] = reports
    assert (record.outcome, record.reason) == (OUTCOME_ABORTED, "no_detection")
    assert record.d3_m == pytest.approx(0.72, abs=0.01)


def test_the_minimum_stops_at_the_cut():
    sensor, reports, _ = _sensor()
    _cut_after(sensor, _line(east_m=1.0, start=-12.0, count=25), 10)  # cut at north -6.375
    sensor.disarm(FAILED)
    [record] = reports
    assert record.outcome == OUTCOME_ABORTED
    assert record.d3_m == pytest.approx(math.hypot(6.375, 1.0), abs=0.01)


def test_a_cut_stored_before_the_next_sample_excludes_its_opening_and_minimum():
    sensor, reports, _ = _sensor()
    points = _line(east_m=0.72)
    _fly(sensor, points[:7])  # still closing at north -0.25
    sensor.end_evidence(IDENTITY)
    # The next sample opens the range and holds the lower minimum.
    sensor.sample(_s(10.0 + 7 * DT, _at(*points[7])))
    # A late sample (older aircraft time) past the dock.
    sensor.sample(_s(10.0 + 6.5 * DT, _at(1.0, 0.72)))
    sensor.disarm(FAILED)
    [record] = reports
    assert (record.outcome, record.reason) == (OUTCOME_ABORTED, "no_detection")
    assert record.d3_m == pytest.approx(math.hypot(0.25, 0.72), abs=0.01)


def _store_cut_inside_sample(monkeypatch, sensor, index):
    """NAV stores the cut while the sensor is inside its ``index``-th sample,
    after that sample's ingress read."""
    own_frame = sensor._own_frame
    calls = itertools.count()

    def frame(dock_uid):
        if next(calls) == index:
            sensor.end_evidence(IDENTITY)
        return own_frame(dock_uid)

    monkeypatch.setattr(sensor, "_own_frame", frame)


@pytest.mark.parametrize(
    "inside, outcome, d3_m",
    [
        (8, OUTCOME_MISSED, 0.72),  # 7 -> 8 holds the closest point and opens
        (7, OUTCOME_ABORTED, math.hypot(0.125, 0.72)),  # 7 -> 8 is after the cut
    ],
)
def test_a_sample_read_before_the_store_counts_and_the_next_does_not(
    monkeypatch, inside, outcome, d3_m,
):
    sensor, reports, _ = _sensor()
    _store_cut_inside_sample(monkeypatch, sensor, inside)
    _fly(sensor, _line(east_m=0.72, start=-4.5))  # 7 at north -0.125, 8 at 0.5
    sensor.disarm(FAILED)
    [record] = reports
    assert record.outcome == outcome
    assert record.d3_m == pytest.approx(d3_m, abs=0.01)


def test_a_cut_for_another_identity_is_ignored():
    sensor, reports, _ = _sensor()
    points = _line(east_m=0.72)
    _fly(sensor, points[:5])
    sensor.end_evidence(DockContactIdentity(vehicle=7, task_id=3, dock_uid=1, episode=3))
    _fly(sensor, points[5:], t0=10.0 + 5 * DT)
    sensor.disarm(FAILED)
    assert [r.outcome for r in reports] == [OUTCOME_MISSED]


def test_a_stale_cut_never_cuts_the_next_episode():
    sensor, reports, _ = _sensor()
    sensor.end_evidence(IDENTITY)
    sensor.arm(DockContactIdentity(7, 3, 1, 5), lambda _event: None)
    _fly(sensor, _line(east_m=0.72))
    sensor.disarm(FAILED)
    assert [(r.episode, r.outcome) for r in reports] == [(5, OUTCOME_MISSED)]


def test_a_contact_after_the_cut_still_publishes_and_the_record_keeps_the_cut():
    sensor, reports, events = _sensor()
    _cut_after(sensor, _line(east_m=0.2), 5)  # cut at north -1.5, closing
    assert len(events) == 1  # the lifecycle refuses it after FAILED
    sensor.disarm(FAILED)
    [record] = reports
    assert record.outcome == OUTCOME_ABORTED
    assert record.d3_m == pytest.approx(math.hypot(1.5, 0.2), abs=0.01)


def _turn():
    """A 100 m radius turn centred 316 m east of the dock at 30 m/s and
    40 Hz: the dock nearly dead ahead at first, abeam at 216 m (sample 209.4)."""
    step = 0.75 / 100.0
    return [
        (100.0 * math.sin(math.pi / 2 + i * step), 316.0 + 100.0 * math.cos(math.pi / 2 + i * step), 0.0)
        for i in range(213)  # to 0.075 s after the closest point
    ]


def test_a_cancel_just_before_a_far_turns_closest_point_is_aborted():
    """Codex design r3: evidence to the disarm would read missed 216 m."""
    sensor, reports, _ = _sensor()
    _fly(sensor, _turn())
    sensor.disarm(DockEpisodeEnd(abort_reason="cancelled"))
    [record] = reports
    assert (record.outcome, record.reason) == (OUTCOME_ABORTED, "cancelled")
    assert record.d3_m == pytest.approx(216.0, rel=0.005)  # the test's flat degrees


def test_a_nav_fail_just_before_a_far_turns_closest_point_is_aborted():
    sensor, reports, _ = _sensor()
    _cut_after(sensor, _turn(), 208)  # 0.05 s before the closest point
    sensor.disarm(FAILED)
    assert [(r.outcome, r.reason) for r in reports] == [(OUTCOME_ABORTED, "no_detection")]


def test_an_abeam_orbit_dip_then_an_inbound_leg_cut_while_closing_is_aborted():
    """The orbit sets the minimum and its range opens, but the dock was
    never ahead; the inbound leg had it ahead but never opened."""
    step = 0.025
    orbit = [
        (30.0 * math.sin(i * step), 2.0 + 30.0 * math.cos(i * step), 0.0)
        for i in range(round(2 * math.pi / step) + 1)
    ]
    inbound = [(0.0, 32.0 - 0.75 * i, 0.0) for i in range(1, 3)]
    sensor, reports, _ = _sensor()
    _fly(sensor, orbit + inbound)
    sensor.disarm(FAILED)
    [record] = reports
    assert record.outcome == OUTCOME_ABORTED
    assert record.d3_m == pytest.approx(28.0, rel=0.005)  # the test's flat degrees


def test_closing_then_a_repeated_position_is_aborted():
    sensor, reports, _ = _sensor()
    points = _line(east_m=0.72)[:7]
    _fly(sensor, points + [points[-1]])  # a new time, the same position
    sensor.disarm(FAILED)
    assert [r.outcome for r in reports] == [OUTCOME_ABORTED]


@pytest.mark.parametrize("cause", ["gap", "clock_reset", "invalid", "dock_change"])
def test_ahead_then_a_break_then_an_opening_segment_is_aborted(cause):
    docks = [SimulationObject(0, _at(0.0, 30.0), 10), SimulationObject(1, _at(0.0), 10)]
    reports, flat = _flat_reports()
    sensor = SimDockContactSensor(lambda: docks, report=flat)
    sensor.arm(IDENTITY, lambda _event: None)
    points = _line(east_m=0.72)
    _fly(sensor, points[:7])  # ahead, still closing at north -0.25
    t = 10.0 + 7 * DT
    if cause == "gap":
        t += 2 * DT
    elif cause == "clock_reset":
        t -= 1.0
    elif cause == "invalid":
        sensor.sample(PositionSample(t, _at(*points[7]), -1.0))
        t += DT
    else:
        docks[1] = SimulationObject(1, _at(0.0), 10)  # a world reload
    _fly(sensor, points[7:], t0=t)
    sensor.disarm(FAILED)
    assert [r.outcome for r in reports] == [OUTCOME_ABORTED]


@pytest.mark.parametrize(
    "start, end, passed",
    [
        ((0.72, -2.0, 0.0), (0.72, 0.5, 0.0), True),  # crosses, ends nearer than it started
        ((0.72, -2.0, 0.0), (0.72, 2.0, 0.0), True),
        ((0.72, -2.0, 0.0), (0.72, 0.0, 0.0), False),  # ends at its closest point
        ((0.72, -2.0, 0.0), (0.72, -2.0, 0.0), False),  # stationary
    ],
)
def test_one_segment_is_a_pass_only_if_the_range_opens_at_its_end(start, end, passed):
    tracker = ClosestApproachTracker()
    tracker.observe(start, end)
    assert tracker.take().passed is passed


def test_rearm_clears_the_pass_evidence():
    tracker = ClosestApproachTracker()
    tracker.observe((0.72, -4.0, 0.0), (0.72, -0.25, 0.0))  # ahead
    tracker.take()
    tracker.observe((0.72, 0.375, 0.0), (0.72, 1.0, 0.0))  # opening, dock behind
    assert tracker.take().passed is False


def test_the_pass_evidence_counts_segments_the_flat_filter_skips(monkeypatch):
    from navpy.sim.world import dock_contact_sensor as module

    # Only the first segment gets the exact geometry; every later one is
    # flat-farther than it.
    monkeypatch.setattr(module, "_flat_distance", lambda _previous, _current: (1e9, 0.0))
    sensor, reports, _ = _sensor()
    _fly(sensor, _line(east_m=0.72))
    sensor.disarm(FAILED)
    assert [r.outcome for r in reports] == [OUTCOME_MISSED]


# -- the cut: NAV's FAILED or MISSED latch, first observer, never raising ------

def test_a_handoff_that_blocks_while_samples_arrive_leaves_them_excluded():
    sensor, reports, _ = _sensor()
    points = _line(east_m=0.72)
    _fly(sensor, points[:7])  # the dock ahead, still closing
    stored = []

    def handoff(_terminal, _identity):
        stored.append(sensor._evidence_end)  # the cut ran first
        # The opening segment and the lower minimum arrive on another thread.
        feeder = threading.Thread(
            target=_fly, args=(sensor, points[7:]), kwargs={"t0": 10.0 + 7 * DT},
        )
        feeder.start()
        feeder.join()

    latch = FinalApproachEpisodeLatch(with_dock_evidence_cut(sensor, handoff))
    latch.begin(IDENTITY)
    assert latch.latch_failure("no_detection")
    sensor.disarm(FAILED)
    assert stored == [IDENTITY]
    [record] = reports
    assert (record.outcome, record.reason) == (OUTCOME_ABORTED, "no_detection")
    assert record.d3_m == pytest.approx(math.hypot(0.25, 0.72), abs=0.01)


def test_a_cut_that_raises_never_reaches_the_latch(caplog):
    sensor = Mock()
    sensor.end_evidence.side_effect = RuntimeError("cut down")
    handoff = Mock()
    latch = FinalApproachEpisodeLatch(with_dock_evidence_cut(sensor, handoff))
    latch.begin(IDENTITY)
    assert latch.latch_failure("no_detection")
    assert latch.commands_ended  # the caller retires commands next
    handoff.assert_called_once_with(FinalApproachTerminal.FAILED, IDENTITY)
    assert "dock evidence cut failed" in caplog.text


@pytest.mark.parametrize(
    "terminal, cut",
    [
        (FinalApproachTerminal.FAILED, True),
        (FinalApproachTerminal.MISSED, True),
        (FinalApproachTerminal.DOCKED, False),
    ],
)
def test_the_composed_observer_cuts_first_and_only_for_navs_own_end(terminal, cut):
    calls = []
    sensor = SimpleNamespace(end_evidence=lambda identity: calls.append(("cut", identity)))
    observer = with_dock_evidence_cut(
        sensor, lambda kind, identity: calls.append(("handoff", kind, identity)),
    )
    observer(terminal, IDENTITY)
    assert calls == [("cut", IDENTITY)] * cut + [("handoff", terminal, IDENTITY)]


def test_without_a_sensor_or_an_armed_identity_nothing_is_cut():
    handoff = Mock()
    assert with_dock_evidence_cut(None, handoff) is handoff
    assert with_dock_evidence_cut(None, None) is None
    sensor = Mock()
    with_dock_evidence_cut(sensor, None)(FinalApproachTerminal.FAILED, None)
    sensor.end_evidence.assert_not_called()
