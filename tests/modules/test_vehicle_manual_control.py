"""Public VehicleMav manual-control contract."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch
import threading

import pytest

from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.vehicle_commands import VehicleCommands
from navpy.modules.vehicle.vehicle_mav import VehicleMav


@pytest.fixture
def vehicle_and_connection():
    connection = SimpleNamespace(mav=MagicMock())
    identity = SimpleNamespace(target_system=1, source_system=1)
    commands = VehicleCommands(
        identity, MavTransport(connection, threading.RLock()),
    )
    with patch(
        "navpy.modules.vehicle.vehicle_mav.build_vehicle_parts",
        return_value=SimpleNamespace(
            identity=identity,
            commands=commands,
        ),
    ):
        vehicle = VehicleMav(
            "unused",
            1,
            wait_heartbeat=False,
            send_heartbeat=False,
            skip_mission_download=True,
        )
    return vehicle, connection


@pytest.mark.parametrize(
    ("values", "expected"),
    [
        ((100, -200, 500, 300), (1, 100, -200, 500, 300, 0)),
        ((0, 0, 500, 0, 3), (1, 0, 0, 500, 0, 3)),
        ((0, 0, 500, 0), (1, 0, 0, 500, 0, 0)),
    ],
)
def test_send_manual_control(vehicle_and_connection, values, expected):
    vehicle, connection = vehicle_and_connection

    vehicle.send_manual_control(*values)

    connection.mav.manual_control_send.assert_called_once_with(*expected)


def test_vehicle_identity_is_public_without_transport_access(vehicle_and_connection):
    vehicle, _connection = vehicle_and_connection

    assert vehicle.target_system == 1
    assert vehicle.source_system == 1
