from __future__ import annotations

import pytest

from navpy.modules.common.models.location import Location
from scripts import upload_north_line_mission as uploader


class _Vehicle:
    """Reports home as the caller scripted it, one reading per call.

    Uses the real Location type: a stand-in with invented field names would
    only prove the wait agrees with the test's own guess at the API.
    """

    def __init__(self, readings: list[object]) -> None:
        self._readings = list(readings)
        self.reads = 0

    @property
    def home_location(self):
        self.reads += 1
        if not self._readings:
            raise AssertionError("home_location read more times than scripted")
        return self._readings.pop(0)


def test_home_wait_accepts_an_established_home() -> None:
    vehicle = _Vehicle([Location(lat=43.0, lng=34.0, alt=0.0)])

    uploader._wait_for_home(vehicle, echo=lambda _line: None)

    assert vehicle.reads == 1


def test_home_wait_ignores_the_pre_fix_placeholder() -> None:
    """SITL reports 0,0 until it has a GPS fix; that is not a location."""
    vehicle = _Vehicle([
        None,
        Location(lat=0.0, lng=0.0, alt=0.0),
        Location(lat=43.0, lng=34.0, alt=0.0),
    ])

    uploader._wait_for_home(vehicle, echo=lambda _line: None)

    assert vehicle.reads == 3


def test_home_wait_gives_up_rather_than_uploading_against_no_fix() -> None:
    vehicle = _Vehicle([Location(lat=0.0, lng=0.0, alt=0.0)] * 200)

    with pytest.raises(RuntimeError, match="no GPS fix"):
        uploader._wait_for_home(vehicle, echo=lambda _line: None, timeout_s=0.6)


def test_route_must_run_northward() -> None:
    with pytest.raises(ValueError, match="doubles back"):
        uploader.check_offsets(2000.0, 1500.0)


def test_gate_south_of_the_loiter_is_refused() -> None:
    with pytest.raises(ValueError, match="gate offset .* must be north"):
        uploader.check_offsets(1000.0, 2700.0, 1000.0, 900.0)


def test_default_route_is_accepted() -> None:
    uploader.check_offsets(
        uploader.DEFAULT_TAKEOFF_OFFSET_M, uploader.DEFAULT_WAYPOINT_OFFSET_M
    )


def test_loader_orders_every_navigated_point_northward() -> None:
    """Loiter, gate and POI must increase in latitude or the route reverses."""
    loader = uploader.build_loader((43.0, 34.0), 1000.0, 2700.0, 400.0, 400.0, 109)

    latitudes = [loader.wp(index).x for index in range(2, loader.count())]
    assert latitudes == sorted(latitudes)


def test_loader_keeps_every_point_on_one_meridian() -> None:
    loader = uploader.build_loader((43.0, 34.0), 1000.0, 2700.0, 400.0, 400.0, 109)

    longitudes = {loader.wp(index).y for index in range(loader.count())}
    assert len(longitudes) == 1


def test_loader_emits_the_loiter_between_takeoff_and_the_gate() -> None:
    from pymavlink.dialects.v20.ardupilotmega import (
        MAV_CMD_NAV_LOITER_TO_ALT,
        MAV_CMD_NAV_TAKEOFF,
        MAV_CMD_NAV_WAYPOINT,
    )

    loader = uploader.build_loader((43.0, 34.0), 1000.0, 2700.0, 400.0, 400.0, 109)

    commands = [loader.wp(index).command for index in range(loader.count())]
    assert commands == [
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_TAKEOFF,
        MAV_CMD_NAV_LOITER_TO_ALT,
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_WAYPOINT,
    ]


def _resolved(ordinal: int):
    """Resolve an ordinal through the evaluator's own selector.

    Deliberately uses resolve_poi_expectation rather than re-counting
    NAV_WAYPOINTs here: a local reimplementation would only prove this test
    agrees with itself, which is exactly how the gate/POI mix-up survived.
    """
    from scripts.eval_navigation_mission import (
        mission_item_from_message,
        resolve_poi_expectation,
    )

    loader = uploader.build_loader(
        (43.0, 34.0),
        uploader.DEFAULT_TAKEOFF_OFFSET_M,
        uploader.DEFAULT_WAYPOINT_OFFSET_M,
        400.0,
        400.0,
        109,
    )
    class _AsVehicleReportsIt:
        """The loader stores 1e7 ints for mission_item_int_send, and the vehicle
        returns MISSION_ITEM_INT; say so rather than letting the converter read
        raw ints as degrees."""

        def __init__(self, waypoint):
            self._waypoint = waypoint

        def get_type(self):
            return "MISSION_ITEM_INT"

        def __getattr__(self, name):
            return getattr(self._waypoint, name)

    mission = [
        mission_item_from_message(_AsVehicleReportsIt(loader.wp(index)))
        for index in range(loader.count())
    ]
    return resolve_poi_expectation(
        mission, poi_wp=ordinal, poi_rel_alt_m=60.0, home_abs_alt_m=0.0
    )


def test_evaluator_defaults_select_the_poi_not_the_gate() -> None:
    """Ordinal 1 is the handover gate; scoring it would discard the scored leg."""
    from scripts import eval_direct_pixel_pn as evaluator

    args = evaluator._parser().parse_args([])
    poi = _resolved(args.poi_wp)
    run_navigation_episode = _resolved(args.scoring_start_wp)

    assert poi.mission_seq == 4, "POI must be the final NAV_WAYPOINT"
    assert run_navigation_episode.mission_seq == 4
    north_m = (poi.location.lat_deg - 43.0) * uploader.METRES_PER_DEG_LAT
    assert north_m == pytest.approx(uploader.DEFAULT_WAYPOINT_OFFSET_M, abs=1.0)


def test_gate_is_ordinal_one_and_loiter_is_not_counted() -> None:
    gate = _resolved(1)

    assert gate.mission_seq == 3
    north_m = (gate.location.lat_deg - 43.0) * uploader.METRES_PER_DEG_LAT
    assert north_m == pytest.approx(uploader.DEFAULT_GATE_OFFSET_M, abs=1.0)


def test_scoring_active_leg_is_gate_to_poi_and_fixed() -> None:
    leg_m = uploader.DEFAULT_WAYPOINT_OFFSET_M - uploader.DEFAULT_GATE_OFFSET_M

    assert leg_m == pytest.approx(900.0)


def test_poi_south_of_the_gate_is_refused() -> None:
    with pytest.raises(ValueError, match="must be north of the gate"):
        uploader.check_offsets(1000.0, 1500.0)
