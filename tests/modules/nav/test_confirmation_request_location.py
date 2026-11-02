"""The confirm request carries the POI position, or NaN when none is known."""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.kernel.location import Location
from navpy.core.link.task_confirmation_msg import TaskConfirmRequestMsg
from navpy.core.review.confirmation_manager import ConfirmationManager
from navpy.core.review.confirmation_round_runner import ConfirmationRoundRunner


def _request(location, truth=None, is_simulation=False):
    geo = SimpleNamespace(projected_poi_location=location, truth_poi_location=truth)
    poi = SimpleNamespace(geo=geo, classification=SimpleNamespace(class_id=0))
    runner = SimpleNamespace(_sys_id=5, _is_simulation=is_simulation)
    request = ConfirmationRoundRunner._request_message(runner, poi, 1)
    # The POI is read, never changed: the command path sees nothing new.
    assert geo.projected_poi_location is location
    assert geo.truth_poi_location is truth
    # Packed to bytes and parsed back, as the GCS receives it.
    link = mavlink.MAVLink(None, srcSystem=5, srcComponent=191)
    raw = request.to_mavlink().pack(link)
    return TaskConfirmRequestMsg.from_mavlink(link.parse_char(raw))


def test_unknown_position_is_sent_as_nan_not_zero() -> None:
    location = _request(None).task.location

    assert math.isnan(location.lat)
    assert math.isnan(location.lng)
    assert math.isnan(location.alt)


def test_known_position_is_sent_as_is() -> None:
    location = _request(Location(40.2885, 44.4737, 1293.5)).task.location

    assert math.isclose(location.lat, 40.2885, abs_tol=1e-4)
    assert math.isclose(location.lng, 44.4737, abs_tol=1e-4)
    assert math.isclose(location.alt, 1293.5, abs_tol=1e-2)


def test_simulation_sends_the_truth_position() -> None:
    location = _request(
        Location(40.0, 44.0, 1000.0),
        truth=Location(40.2885, 44.4737, 1293.5),
        is_simulation=True,
    ).task.location

    assert math.isclose(location.lat, 40.2885, abs_tol=1e-4)
    assert math.isclose(location.lng, 44.4737, abs_tol=1e-4)


def test_simulation_without_truth_falls_back_to_the_projected_position() -> None:
    location = _request(Location(40.0, 44.0, 1000.0), is_simulation=True).task.location

    assert math.isclose(location.lat, 40.0, abs_tol=1e-4)


def test_simulation_with_no_position_stays_unknown() -> None:
    assert math.isnan(_request(None, is_simulation=True).task.location.lat)


def test_real_build_never_sends_truth() -> None:
    location = _request(None, truth=Location(40.2885, 44.4737, 1293.5)).task.location

    assert math.isnan(location.lat)


def test_the_manager_passes_its_simulation_flag_to_the_round_runner() -> None:
    manager = ConfirmationManager(sys_id=5, args=SimpleNamespace(), logger=Mock(), is_simulation=True)
    geo = SimpleNamespace(projected_poi_location=None, truth_poi_location=Location(40.2885, 44.4737, 1293.5))
    poi = SimpleNamespace(geo=geo, classification=SimpleNamespace(class_id=0))

    request = manager._round_runner._request_message(poi, 1)

    assert math.isclose(request.task.location.lat, 40.2885, abs_tol=1e-4)
