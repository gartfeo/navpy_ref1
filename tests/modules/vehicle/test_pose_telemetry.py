from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from navpy.plugins.vehicle.ardupilot.pose_telemetry import (
    position_sample_from_message,
    simulator_truth_pose_from_sample,
    simulator_truth_source_time_s,
)


def _sim_state_message(**overrides: object) -> SimpleNamespace:
    fields: dict[str, object] = {
        "lat": 40.0,
        "lon": 44.0,
        "alt": 1000.0,
        "roll": 0.05,
        "pitch": -0.07,
        "yaw": 2.1,
        "lat_int": 0,
        "lon_int": 0,
    }
    fields.update(overrides)
    return SimpleNamespace(**fields)


def _sample(message: SimpleNamespace) -> SimpleNamespace:
    return SimpleNamespace(message=message, receipt_time_s=123.456)


def test_source_time_reads_the_time_us_extension_as_seconds() -> None:
    message = _sim_state_message(time_us=12_500_000)

    assert simulator_truth_source_time_s(message) == pytest.approx(12.5)


def test_source_time_is_none_without_the_extension() -> None:
    """Stock firmware: the field simply does not exist on the message."""
    assert simulator_truth_source_time_s(_sim_state_message()) is None


@pytest.mark.parametrize(
    "time_us", [0, -1, None, True, "12", float("nan"), float("inf")]
)
def test_source_time_rejects_invalid_values(time_us: object) -> None:
    message = _sim_state_message(time_us=time_us)

    assert simulator_truth_source_time_s(message) is None


def test_truth_pose_carries_the_source_stamp() -> None:
    pose = simulator_truth_pose_from_sample(
        _sample(_sim_state_message(time_us=12_500_000))
    )

    assert pose is not None
    assert pose.source_time_s == pytest.approx(12.5)
    assert pose.receipt_time_s == pytest.approx(123.456)
    assert pose.attitude.yaw == pytest.approx(math.degrees(2.1))


def test_truth_pose_without_stamp_has_none_source_time() -> None:
    pose = simulator_truth_pose_from_sample(_sample(_sim_state_message()))

    assert pose is not None
    assert pose.source_time_s is None


def _global_position(**overrides: object) -> SimpleNamespace:
    fields: dict[str, object] = {
        "lat": 401234567,
        "lon": 441234567,
        "alt": 1250500,
        "relative_alt": 50000,
        "time_boot_ms": 12345,
    }
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_position_sample_is_absolute_and_stamped_from_integer_boot_ms() -> None:
    sample = position_sample_from_message(_global_position(), 77.5)

    assert sample is not None
    assert sample.lat == pytest.approx(40.1234567)
    assert sample.lon == pytest.approx(44.1234567)
    assert sample.alt == pytest.approx(1250.5)
    assert sample.time_boot_s == float(12345) * 1e-3
    assert sample.receipt_time_s == 77.5


@pytest.mark.parametrize("time_boot_ms", [None, True, 12345.0, "12345"])
def test_position_sample_rejects_a_non_integer_boot_stamp(
    time_boot_ms: object,
) -> None:
    message = _global_position(time_boot_ms=time_boot_ms)

    assert position_sample_from_message(message, 1.0) is None
