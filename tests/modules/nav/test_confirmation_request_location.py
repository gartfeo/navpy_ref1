"""The confirm request carries the POI position, or NaN when none is known."""

from __future__ import annotations

import math
from types import SimpleNamespace

from navpy.core.kernel.location import Location
from navpy.core.link.task_confirmation_msg import TaskConfirmRequestMsg
from navpy.core.review.confirmation_round_runner import ConfirmationRoundRunner


def _request(location):
    poi = SimpleNamespace(
        geo=SimpleNamespace(projected_poi_location=location),
        classification=SimpleNamespace(class_id=0),
    )
    runner = SimpleNamespace(_sys_id=5)
    request = ConfirmationRoundRunner._request_message(runner, poi, 1)
    # Through the wire, as the GCS receives it.
    return TaskConfirmRequestMsg.from_mavlink(request.to_mavlink())


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
