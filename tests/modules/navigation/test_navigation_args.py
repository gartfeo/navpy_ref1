import argparse

from navpy.args.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.args.pid_args import PIDArgs


class FakeVehicle:
    min_pitch = -60

    def __init__(self, params=None):
        self.params = dict(params or {})

    def get_param_or_default(self, name, default):
        return self.params.get(name, default)


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        self.warnings.append(args[0] if args else "")


def _parse_args(cli_args=None):
    parser = argparse.ArgumentParser()
    NavigationArgs.add_args(parser)
    PIDArgs.add_args(parser, "pitch")
    return parser.parse_args(cli_args or [])


def _navigation_args(cli_args=None, params=None):
    args = _parse_args(cli_args)
    return args, NavigationArgs(args, FakeVehicle(params), FakeLogger())


def test_pitch_kp_cli_override_feeds_pitch_args():
    args, navigation_args = _navigation_args(["-pkp", "2.3"], {"AAS_DEL_P_KP": 4.0})

    assert args.pitch_kp == 2.3
    assert navigation_args.pitch_args.kp == 2.3


def test_pitch_kp_uses_vehicle_param_without_cli_override():
    vehicle = FakeVehicle({"AAS_DEL_P_KP": 2.2})
    args = _parse_args()
    navigation_args = NavigationArgs(args, vehicle, FakeLogger())

    assert args.pitch_kp == 2.2
    assert navigation_args.pitch_args.kp == 2.2

    vehicle.params["AAS_DEL_P_KP"] = 2.4
    navigation_args.refresh()

    assert args.pitch_kp == 2.4
    assert navigation_args.pitch_args.kp == 2.4


def test_pitch_kp_missing_vehicle_param_uses_navigation_default():
    args, navigation_args = _navigation_args()
    default_kp = NavigationArgs.PARAMS["AAS_DEL_P_KP"]

    assert default_kp == 1.5
    assert args.pitch_kp == default_kp
    assert navigation_args.pitch_args.kp == default_kp


def test_pitch_kp_cli_override_survives_refresh_when_vehicle_param_changes():
    vehicle = FakeVehicle({"AAS_DEL_P_KP": 4.0})
    args = _parse_args(["-pkp", "2.3"])
    navigation_args = NavigationArgs(args, vehicle, FakeLogger())

    vehicle.params["AAS_DEL_P_KP"] = 5.0
    navigation_args.refresh()

    assert args.pitch_kp == 2.3
    assert navigation_args.pitch_args.kp == 2.3


def test_default_navigation_algorithm_is_legacy_pn():
    _, navigation_args = _navigation_args()

    assert navigation_args.navigation_algorithm == NavigationAlgorithm.PN.value
    assert navigation_args.pitch_controller == "pn"


def test_navigation_algorithm_maps_mavlink_param_values():
    expected = {
        0: (NavigationAlgorithm.PID.value, "pid"),
        1: (NavigationAlgorithm.PN.value, "pn"),
        2: (NavigationAlgorithm.VISION_NAV_PN.value, "pn"),
    }

    for param_value, (algorithm, pitch_controller) in expected.items():
        _, navigation_args = _navigation_args(params={"AAS_DEL_CTRL": param_value})

        assert navigation_args.navigation_algorithm == algorithm
        assert navigation_args.pitch_controller == pitch_controller


def test_navigation_algorithm_cli_selects_vision_nav():
    _, navigation_args = _navigation_args(["--pitch-controller", "vision-nav-pn"])

    assert navigation_args.navigation_algorithm == NavigationAlgorithm.VISION_NAV_PN.value
    assert navigation_args.pitch_controller == "pn"


def test_navigation_algorithm_cli_accepts_numeric_selection():
    _, navigation_args = _navigation_args(["--navigation-algorithm", "2"])

    assert navigation_args.navigation_algorithm == NavigationAlgorithm.VISION_NAV_PN.value
    assert navigation_args.pitch_controller == "pn"


def test_invalid_navigation_algorithm_param_warns_and_falls_back_to_pn():
    args = _parse_args()
    logger = FakeLogger()
    navigation_args = NavigationArgs(
        args,
        FakeVehicle({"AAS_DEL_CTRL": 99}),
        logger,
    )

    assert navigation_args.navigation_algorithm == NavigationAlgorithm.PN.value
    assert navigation_args.pitch_controller == "pn"
    assert logger.warnings


def test_removed_final_approach_calibration_and_roll_pid_are_not_configs():
    _, navigation_args = _navigation_args(params={"AAS_DEL_FPAOFF": 9.9})
    assert not hasattr(navigation_args, "final_approach_fpa_offset_deg")
    assert not hasattr(navigation_args, "roll_args")
    assert not hasattr(navigation_args, "pitch_kp")
    assert "AAS_DEL_FPAOFF" not in NavigationArgs.PARAMS

    # The operator CLI flags no longer exist.
    parser = argparse.ArgumentParser()
    NavigationArgs.add_args(parser)
    _, unknown = parser.parse_known_args(["-tfpo", "1.0"])
    assert "-tfpo" in unknown


def test_delivery_pitch_and_throttle_use_del_short_flags():
    args = _parse_args(["-da", "-15", "-dt", "40"])

    assert args.AAS_DEL_PITCH == -15
    assert args.AAS_DEL_THR == 40
