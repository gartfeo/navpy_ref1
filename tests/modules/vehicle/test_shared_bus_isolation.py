"""One shared MAVLink link, two systems, only one connected (UniRC, 2026-11-08).

The GCS backend opens one ``MavBus`` per device. On a shared UDP link the
reader hears every vehicle; a system nobody attached is broadcast to every
receiver (the companion relies on that for peer navlink traffic). The
connected vehicle must still keep only its own system's telemetry, or the
operator sees a parked, unconnected vehicle's position, speed and mission
item on the flying one. Swarm navlink traffic from other systems must still
reach NAVLINK subscribers.
"""
from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.link.task_availability_msg import AvailableTaskRequestMsg
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

CONNECTED = 4
UNCONNECTED = 5


def _packed_by(system_id: int, component_id: int, message) -> mavlink.MAVLink_message:
    """``message`` packed by ``system_id``/``component_id``, decoded as a reader would."""
    sender = mavlink.MAVLink(None, srcSystem=system_id, srcComponent=component_id)
    return mavlink.MAVLink(None).parse_buffer(message.pack(sender))[0]


def _telemetry(system_id: int, *, lat_e7: int, groundspeed: float, seq: int):
    encoder = mavlink.MAVLink(None)
    return [
        _packed_by(system_id, 1, encoder.global_position_int_encode(
            1000, lat_e7, 340000000, 150000, 100000, 2500, 0, 0, 0)),
        _packed_by(system_id, 1, encoder.vfr_hud_encode(
            groundspeed, groundspeed, 0, 50, 100.0, 0.0)),
        _packed_by(system_id, 1, encoder.mission_current_encode(seq)),
    ]


def _own():
    return _telemetry(CONNECTED, lat_e7=320000000, groundspeed=25.0, seq=4)


def _foreign():
    return _telemetry(UNCONNECTED, lat_e7=321000000, groundspeed=0.0, seq=0)


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


class _SharedLink:
    """A real ``MavBus`` on a scripted link, one GCS vehicle attached."""

    def __init__(self, device: str) -> None:
        self.connection = _ScriptedConnection()
        self.bus = MavBus(self.connection, device)
        self.vehicle = VehicleMav(
            device=device,
            target_system=CONNECTED,
            logger=MagicMock(),
            skip_mission_download=True,
            wait_heartbeat=False,
            send_heartbeat=False,
            mav_type=mavlink.MAV_TYPE_GCS,
            mav_comp_id=mavlink.MAV_COMP_ID_MISSIONPLANNER,
            bus=self.bus,
        )

    def deliver(self, packets) -> None:
        """Hand ``packets`` to the reader; return once all are dispatched."""
        self.connection.deliver(packets)
        # Two drained observations: the second proves the reader finished
        # dispatching the last packet it took before the first one.
        assert self.connection.drained.wait(5.0)
        self.connection.drained.clear()
        assert self.connection.drained.wait(5.0)

    def close(self) -> None:
        self.vehicle.close()
        self.bus.close()


@pytest.fixture
def link(request):
    shared = _SharedLink(f"test:shared_bus_isolation:{request.node.name}")
    yield shared
    shared.close()


def _assert_own_telemetry(vehicle: VehicleMav) -> None:
    location = vehicle.location(is_relative=False)
    assert location is not None
    assert location.lat == pytest.approx(32.0)
    assert vehicle.ground_speed == pytest.approx(25.0)
    assert vehicle.mission_items_next == 4


def test_foreign_telemetry_after_own_does_not_overwrite_it(link):
    link.deliver(_own())
    link.deliver(_foreign())

    _assert_own_telemetry(link.vehicle)


def test_foreign_telemetry_before_own_is_never_taken(link):
    link.deliver(_foreign())

    assert link.vehicle.location(is_relative=False) is None
    assert link.vehicle.ground_speed is None
    assert link.vehicle.mission_items_next is None

    link.deliver(_own())
    _assert_own_telemetry(link.vehicle)


def test_foreign_navlink_still_reaches_navlink_subscribers(link):
    """A GCS swarm listener (``scripts/python/send_navlink.py``) hears a
    peer's target-less AVAILABLE_TASK_REQUEST broadcast, while that peer's
    telemetry stays out."""
    heard = []
    link.vehicle.on_message("NAVLINK", heard.append)
    broadcast = _packed_by(
        UNCONNECTED, 191, AvailableTaskRequestMsg(UNCONNECTED, []).to_mavlink(),
    )

    link.deliver([broadcast] + _foreign())

    assert [(m.get_type(), m.get_srcSystem()) for m in heard] == [
        (broadcast.get_type(), UNCONNECTED),
    ]
    assert link.vehicle.location(is_relative=False) is None
