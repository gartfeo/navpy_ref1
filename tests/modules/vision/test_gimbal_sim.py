import time
from dataclasses import dataclass, field
from unittest.mock import Mock

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.sim.gimbal_sim import GimbalSim
from navpy.modules.vision.sim.gimbal_stabilizer import GimbalStabilizer


@dataclass
class _AttitudeReader:
    values: list[Attitude | None] = field(
        default_factory=lambda: [Attitude(0.0, 0.0, 0.0)]
    )

    def read(self) -> Attitude | None:
        if len(self.values) > 1:
            return self.values.pop(0)
        return self.values[0]


def test_public_sim_preserves_initial_data():
    data = GimbalData(att=Attitude(-45.0, 10.0, 5.0))
    gimbal = GimbalSim(data, _AttitudeReader())

    assert gimbal.get_data() == data


def test_set_attitude_updates_value_and_disables_stabilization():
    data = GimbalData(
        att=Attitude(-45.0, 0.0, 0.0),
        roll_stabilize=True,
        pitch_stabilize=True,
    )
    gimbal = GimbalSim(data, _AttitudeReader())

    gimbal.set_att(Attitude(-30.0, 15.0, 5.0))

    result = gimbal.get_data()
    assert result.att == Attitude(-30.0, 15.0, 5.0)
    assert not result.roll_stabilize
    assert not result.pitch_stabilize


def test_stop_before_start_is_safe():
    gimbal = GimbalSim(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        _AttitudeReader(),
    )
    assert gimbal.stop() is True
    assert not gimbal.is_alive()


def test_stop_propagates_incomplete_runner_quiescence():
    gimbal = GimbalSim(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        _AttitudeReader(),
    )
    gimbal._runner = Mock()
    gimbal._runner.stop.return_value = False

    assert gimbal.stop() is False
    gimbal._runner.stop.assert_called_once_with()


def test_start_is_idempotent_and_stop_joins_worker():
    gimbal = GimbalSim(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        _AttitudeReader(),
    )

    gimbal.start()
    gimbal.start()
    assert gimbal.is_alive()
    gimbal.stop()

    assert not gimbal.is_alive()


def test_thread_safe_reads_during_updates():
    gimbal = GimbalSim(
        GimbalData(
            att=Attitude(-45.0, 0.0, 0.0),
            roll_stabilize=True,
            pitch_stabilize=True,
        ),
        _AttitudeReader([Attitude(10.0, 5.0, 15.0)]),
    )
    gimbal.start()
    try:
        deadline_s = time.monotonic() + 0.1
        while time.monotonic() < deadline_s:
            assert isinstance(gimbal.get_data(), GimbalData)
    finally:
        gimbal.stop()


def test_stabilizer_noops_when_aircraft_attitude_is_unavailable():
    data = GimbalData(
        att=Attitude(-45.0, 0.0, 0.0),
        roll_stabilize=True,
        pitch_stabilize=True,
    )
    stabilizer = GimbalStabilizer(data, _AttitudeReader([None]), "ZYX")

    stabilizer.advance()

    assert stabilizer.get_data() == data


def test_stabilizer_noops_when_both_axes_are_disabled():
    data = GimbalData(
        att=Attitude(-45.0, 0.0, 0.0),
        roll_stabilize=False,
        pitch_stabilize=False,
    )
    stabilizer = GimbalStabilizer(
        data,
        _AttitudeReader([Attitude(10.0, 5.0, 15.0)]),
        "ZYX",
    )

    stabilizer.advance()

    assert stabilizer.get_data() == data


@pytest.mark.parametrize(
    ("roll_stabilize", "pitch_stabilize"),
    [(True, False), (False, True), (True, True)],
)
def test_enabled_stabilization_axes_produce_valid_attitude(
    roll_stabilize: bool,
    pitch_stabilize: bool,
):
    stabilizer = GimbalStabilizer(
        GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            roll_stabilize=roll_stabilize,
            pitch_stabilize=pitch_stabilize,
            g_seq="XYZ",
        ),
        _AttitudeReader([Attitude(10.0, 5.0, 15.0)]),
        "ZYX",
    )

    stabilizer.advance()

    assert isinstance(stabilizer.get_data().att, Attitude)


def test_mount_setup_attitude_does_not_contaminate_camera_attitude():
    stabilizer = GimbalStabilizer(
        GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            roll_stabilize=True,
            pitch_stabilize=False,
            g_seq="XYZ",
            setup=GimbalMountSetup(att=Attitude(90.0, 0.0, 90.0)),
        ),
        _AttitudeReader([Attitude(0.0, 0.0, 15.0)]),
        "ZYX",
    )

    stabilizer.advance()

    assert stabilizer.get_data().att.pitch == pytest.approx(0.0, abs=2.0)
