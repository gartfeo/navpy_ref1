"""Compatibility checks for the peer-orbit extraction."""

from unittest.mock import Mock, patch

from navpy.modules.common.models.location import Location
from navpy.modules.navigation import peer_offset
from navpy.modules.navigation import peer_orbit_plan
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan


def test_orbit_constants_remain_available_from_peer_offset() -> None:
    assert peer_offset.MIN_APPROACH_STANDOFF_M == 500.0
    assert peer_offset.MIN_ACQUIRE_STANDOFF_M == 300.0
    assert peer_offset._ORBIT_PIXEL_MARGIN == 0.9
    assert (
        peer_offset.MIN_APPROACH_STANDOFF_M
        == peer_orbit_plan.MIN_APPROACH_STANDOFF_M
    )
    assert (
        peer_offset.MIN_ACQUIRE_STANDOFF_M
        == peer_orbit_plan.MIN_ACQUIRE_STANDOFF_M
    )
    assert (
        peer_offset._ORBIT_PIXEL_MARGIN
        == peer_orbit_plan._ORBIT_PIXEL_MARGIN
    )
    assert (
        peer_offset.MIN_CONFIRM_PIXELS
        == peer_orbit_plan.MIN_CONFIRM_PIXELS
    )
    assert (
        peer_offset.MIN_DETECT_PIXELS
        == peer_orbit_plan.MIN_DETECT_PIXELS
    )
    assert peer_offset.r_nav_min is peer_orbit_plan.r_nav_min


def test_orbit_leaf_uses_the_original_logger_category() -> None:
    assert peer_orbit_plan._DEFAULT_LOGGER.name == (
        "navpy.modules.navigation.peer_offset"
    )


def test_private_facade_wrapper_preserves_arguments_and_offset_fallback() -> None:
    target = Location(32.0, 34.0, 200.0)
    mount = Mock()
    expected = ApproachPlan(
        kind=ApproachKind.ORBIT,
        approach_location=target,
        offset_distance=0.0,
        orbit_radius=500.0,
    )

    with patch.object(
        peer_offset,
        "compute_orbit_plan",
        return_value=expected,
    ) as planner:
        actual = peer_offset._compute_orbit_plan(
            target,
            mount,
            150.0,
            2,
            None,
            40.0,
        )

    assert actual is expected
    planner.assert_called_once_with(
        target,
        mount,
        150.0,
        2,
        None,
        40.0,
        fallback_radius_m=peer_offset.OFFSET_LOITER_RADIUS_M,
        logger=peer_offset._log,
    )
