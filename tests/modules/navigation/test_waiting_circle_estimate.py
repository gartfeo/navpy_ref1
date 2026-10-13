"""The waiting circle's ground estimate uses no sim truth (owner, 2026-10-07)."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from navpy.core.approach.geo.legacy_destination_resolver import poi_ground_location
from navpy.core.kernel.location import Location
from navpy.core.skills.geo_approach.self_detected_approach import (
    WAITING_CIRCLE_REFERENCE_HEIGHT_M,
)


def _detection(reference_height_m: float, truth: Location) -> SimpleNamespace:
    return SimpleNamespace(
        geo=SimpleNamespace(
            camera_location=Location(40.0, 44.0, 1500.0, is_absolute=True),
            reference_height_m=reference_height_m,
            truth_poi_location=truth,
        ),
    )


def _estimate(detection: SimpleNamespace, **kwargs: object) -> tuple:
    zc_util = Mock()
    zc_util.ray_to_terrain_ned.return_value = Location(40.01, 44.0, 1300.0)
    result = poi_ground_location(
        zc_util=zc_util,
        geo_ref=Mock(),
        detect_data=detection,
        poi_ned=np.array([1.0, 0.0, 0.3]),
        allow_fallback=False,
        **kwargs,
    )
    return result, zc_util.ray_to_terrain_ned.call_args


def test_waiting_circle_estimate_ignores_the_sim_poi_height_and_position():
    honest = _detection(0.0, Location(40.01, 44.0, 1300.0, is_absolute=True))
    poisoned = _detection(999.0, Location(41.0, 45.0, 9999.0, is_absolute=True))

    first, first_call = _estimate(
        honest, reference_height_m=WAITING_CIRCLE_REFERENCE_HEIGHT_M)
    second, second_call = _estimate(
        poisoned, reference_height_m=WAITING_CIRCLE_REFERENCE_HEIGHT_M)

    assert first == second
    assert first_call.args[0] == second_call.args[0]
    assert np.array_equal(first_call.args[1], second_call.args[1])
    assert first_call.kwargs["delta_alt"] == second_call.kwargs["delta_alt"] == 0.0


def test_other_callers_keep_the_detection_reference_height():
    _, call = _estimate(_detection(2.0, Location(40.01, 44.0, 1300.0)))

    assert call.kwargs["delta_alt"] == 2.0
