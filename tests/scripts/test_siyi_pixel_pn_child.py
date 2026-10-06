import inspect
from unittest.mock import Mock, call, patch

from scripts import siyi_pixel_pn_child as child


def test_final_approach_reset_precedes_confirmation_that_seeds_siyi_handoff() -> None:
    navigation = Mock()
    source = Mock()
    order = Mock()
    navigation.init.side_effect = lambda: order("init")

    with patch.object(
        child,
        "_wait_for_visual_confirmation",
        side_effect=lambda *_args: order("confirm"),
    ):
        child._initialize_confirmed_navigation(navigation, source, 3.0)

    assert order.call_args_list == [call("init"), call("confirm")]


def test_pixel_navigation_isolation_does_not_command_a_geo_flight_path() -> None:
    run_source = inspect.getsource(child.run)

    assert "_command_poi_loiter" not in run_source
    assert "_command_poi_approach" not in run_source


def test_siyi_acquisition_precedes_guided_mode_handoff() -> None:
    run_source = inspect.getsource(child.run)

    acquired = run_source.index('if not source.ready:')
    guided = run_source.index('vehicle.set_mode(FlightMode.GUIDED)')
    assert acquired < guided
