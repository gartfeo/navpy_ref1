"""One shared MAVLink link, two systems, only one connected (UniRC, 2026-11-08).

The GCS backend opens one ``MavBus`` per device. On a shared UDP link the
reader hears every vehicle; a system nobody attached is broadcast to every
receiver (the companion relies on that for peer navlink traffic). The
connected vehicle must still keep only its own system's telemetry, or the
operator sees a parked, unconnected vehicle's position, speed and mission
item on the flying one.
"""
from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

CONNECTED = 4
UNCONNECTED = 5


def _decoded(system_id: int, encode) -> mavlink.MAVLink_message:
    """``encode`` packed by ``system_id``'s autopilot, then decoded as a reader would."""
    sender = mavlink.MAVLink(None, srcSystem=system_id, srcComponent=1)
    packed = encode(sender).pack(sender)
    return mavlink.MAVLink(None).parse_buffer(packed)[0]


def _telemetry(system_id: int, *, lat_e7: int, groundspeed: float, seq: int):
    return [
        _decoded(system_id, lambda m: m.global_position_int_encode(
            1000, lat_e7, 340000000, 150000, 100000, 2500, 0, 0, 0)),
        _decoded(system_id, lambda m: m.vfr_hud_encode(
            groundspeed, groundspeed, 0, 50, 100.0, 0.0)),
        _decoded(system_id, lambda m: m.mission_current_encode(seq)),
    ]


class _ScriptedConnection:
    """A link that delivers the packets handed to it, then idles."""

    def __init__(self) -> None:
        self._packets = []
        self._lock = threading.Lock()
        self.drained = threading.Event()
        self.mav = MagicMock()

    def deliver(self, packets) -> None:
        with self._lock:
            self.drained.clear()
            self._packets.extend(packets)

    def recv_match(self, blocking=False, timeout=None):
        with self._lock:
            if self._packets:
                return self._packets.pop(0)
        self.drained.set()
        if blocking:
            time.sleep(0.005)
        return None

    def close(self) -> None:
        pass


@pytest.mark.parametrize("unconnected_first", [False, True])
def test_connected_vehicle_keeps_its_own_telemetry_on_a_shared_link(unconnected_first):
    own = _telemetry(CONNECTED, lat_e7=320000000, groundspeed=25.0, seq=4)
    foreign = _telemetry(UNCONNECTED, lat_e7=321000000, groundspeed=0.0, seq=0)
    packets = foreign + own if unconnected_first else own + foreign
    connection = _ScriptedConnection()
    bus = MavBus(connection, "test:shared_bus_isolation")
    vehicle = VehicleMav(
        device="test:shared_bus_isolation",
        target_system=CONNECTED,
        logger=MagicMock(),
        skip_mission_download=True,
        wait_heartbeat=False,
        send_heartbeat=False,
        mav_type=mavlink.MAV_TYPE_GCS,
        mav_comp_id=mavlink.MAV_COMP_ID_MISSIONPLANNER,
        bus=bus,
    )
    try:
        connection.deliver(packets)
        # Two drained observations: the second proves the reader finished
        # dispatching the last packet it took before the first one.
        assert connection.drained.wait(5.0)
        connection.drained.clear()
        assert connection.drained.wait(5.0)

        location = vehicle.location(is_relative=False)
        assert location is not None
        assert location.lat == pytest.approx(32.0)
        assert vehicle.ground_speed == pytest.approx(25.0)
        assert vehicle.mission_items_next == 4
    finally:
        vehicle.close()
        bus.close()
