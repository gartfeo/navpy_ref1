import argparse
from unittest.mock import MagicMock

import pytest

from navpy.args.logger_args import LoggerArgs, LoggerArgsStub, LogStatusDest
from navpy.logger.cache_log_level import CacheLogLevel


def _parse(argv):
    parser = argparse.ArgumentParser(prog="navpy")
    LoggerArgs.add_args(parser)
    return parser.parse_args(argv)


class TestLogLevelArgParsing:
    """--log-level / --log-status-level accept case-insensitive level NAMES.

    Regression: the flags were declared ``type=CacheLogLevel``, which does a
    value-lookup keyed on the enum's int values (-1..3), so no CLI string ever
    matched and every ``-ll <level>`` invocation errored out.
    """

    def test_short_flag_parses_name(self):
        assert _parse(["-ll", "DEBUG"]).log_level is CacheLogLevel.DEBUG

    def test_long_flag_is_case_insensitive(self):
        assert _parse(["--log-level", "debug"]).log_level is CacheLogLevel.DEBUG

    def test_status_level_flag_parses_name(self):
        assert _parse(["-lsl", "warning"]).log_status_level is CacheLogLevel.WARNING

    def test_defaults_to_info_when_omitted(self):
        ns = _parse([])
        assert ns.log_level is CacheLogLevel.INFO
        assert ns.log_status_level is CacheLogLevel.INFO

    def test_flows_into_logger_args(self):
        args = LoggerArgs(_parse(["-ll", "verbose"]))
        assert args.log_level is CacheLogLevel.VERBOSE

    def test_invalid_level_exits(self):
        with pytest.raises(SystemExit):
            _parse(["-ll", "bogus"])

    def test_numeric_value_string_rejected(self):
        # The old broken form (an int-valued string) is not a name; reject it.
        with pytest.raises(SystemExit):
            _parse(["-ll", "1"])


def _make_args(**overrides):
    defaults = {
        "log_status_interval": 2,
        "log_status_dest": [LogStatusDest.DRONE.value, LogStatusDest.NETWORK.value],
        "log_level": CacheLogLevel.INFO,
        "log_status_level": CacheLogLevel.INFO,
    }
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def _make_vehicle(param_values=None):
    vehicle = MagicMock()
    param_values = param_values or {}

    def get_param(name, default):
        return param_values.get(name, default)

    vehicle.get_param_or_default.side_effect = get_param
    return vehicle


class TestLoggerArgs:
    def test_init_default_interval(self):
        args = LoggerArgs(_make_args())
        assert args.status_update_interval == 0.5  # 1/2 Hz

    def test_refresh_uses_vehicle_param(self):
        args = LoggerArgs(_make_args())
        vehicle = _make_vehicle({"AAS_LOG_RATE": 5, "AAS_LOG_DEFER": 0})
        args.set_vehicle(vehicle)
        assert args.status_update_interval == 0.2  # 1/5 Hz

    def test_refresh_zero_rate_no_crash(self):
        """AAS_LOG_RATE=0 should not cause ZeroDivisionError."""
        args = LoggerArgs(_make_args())
        original_interval = args.status_update_interval
        vehicle = _make_vehicle({"AAS_LOG_RATE": 0, "AAS_LOG_DEFER": 0})
        args.set_vehicle(vehicle)
        # interval unchanged when rate is 0
        assert args.status_update_interval == original_interval

    def test_refresh_no_vehicle_is_noop(self):
        args = LoggerArgs(_make_args())
        original = args.status_update_interval
        args.refresh()
        assert args.status_update_interval == original


class TestLoggerArgsStub:
    def test_stub_creates_valid_instance(self):
        stub = LoggerArgsStub()
        assert stub.status_update_interval == 1.0 / 0.2
        assert stub.vehicle is None
