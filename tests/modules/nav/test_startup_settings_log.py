"""The startup log names every setting that shapes the review and delivery flow."""

from argparse import Namespace
from unittest.mock import Mock

from navpy.app import runtime_composition
from navpy.core.flow.nav_status import NavigationStatusReporter
from navpy.core.kernel.config.nav_args import NavArgs


def _logged(logger):
    return [call.args[0] for call in logger.info.call_args_list]


def test_initial_status_logs_the_effective_flow_settings():
    args = Namespace(
        confirm_wait_time_sec=30.0,
        min_wp=10,
        min_alt=-100.0,
        is_auto_confirm=0.0,  # a MAVLink param arrives as a float
        is_confirm_on_fail=1.0,
        is_oneshot=True,
        confirm_gate_timeout_sec=15.0,
    )
    logger = Mock()

    NavigationStatusReporter(logger, args, lambda: None).initial()

    # The "N: <cwt>s" prefix is a readiness marker for eval scripts.
    assert _logged(logger) == [
        "N: 30.0s, min_wp: 10; min_alt: -100.0 m; auto_confirm: False; "
        "confirm_on_fail: True; oneshot: True; gate: 15.0s; home: N/A"
    ]


def test_an_explicit_cli_value_wins_over_the_param_in_the_status():
    # -tcfc false equals the default, so "Args (non-default)" omits it, but
    # it still overrides the AAS_NAV_CM_FL param: the status line shows it.
    params = {"AAS_NAV_CM_FL": 1.0, "AAS_NAV_AUTO_CM": 0.0}
    vehicle = Mock(mission_items_count=12)
    vehicle.get_param_or_default = lambda name, default: params.get(name, default)
    vehicle.get_mission_item.return_value = None
    args = Namespace(AAS_NAV_CM_FL=False, _cli_overrides={"AAS_NAV_CM_FL"})
    logger = Mock()

    NavigationStatusReporter(logger, NavArgs(args, vehicle), lambda: None).initial()

    (line,) = _logged(logger)
    assert "auto_confirm: False; confirm_on_fail: False; oneshot: False" in line


def test_non_default_params_include_the_nav_flow_params(monkeypatch):
    params = {
        "AAS_NAV_MIN_ALT": -100.0,
        "AAS_NAV_ONESHOT": 1.0,
        "AAS_DEL_DIR": 1.0,
        "AAS_TARG_WPS": 8.0,
    }
    vehicle = Mock()
    vehicle.get_param_or_default = lambda name, default: params.get(name, default)
    monkeypatch.setattr(
        runtime_composition,
        "collect_non_default_args",
        lambda args, defaults: {},
    )
    logger = Mock()

    runtime_composition._log_configuration(Namespace(), vehicle, logger)

    assert _logged(logger)[-1] == (
        "AAS params (non-default): AAS_DEL_DIR=1.0, AAS_NAV_MIN_ALT=-100.0, "
        "AAS_NAV_ONESHOT=1.0, AAS_TARG_WPS=8.0"
    )
