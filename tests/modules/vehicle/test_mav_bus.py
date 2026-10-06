"""Tests for MavBus — shared MAVLink connection."""
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_TYPE_FIXED_WING, MAV_TYPE_GCS, MAV_TYPE_ONBOARD_CONTROLLER,
)


def _make_heartbeat(sys_id, mav_type=MAV_TYPE_FIXED_WING):
    msg = MagicMock()
    msg.get_srcSystem.return_value = sys_id
    msg.get_type.return_value = "HEARTBEAT"
    msg.type = mav_type
    return msg


def _make_data(sys_id, mtype="GLOBAL_POSITION_INT"):
    msg = MagicMock()
    msg.get_srcSystem.return_value = sys_id
    msg.get_type.return_value = mtype
    return msg


def _idle_connection():
    """Return a connection whose reader mock cannot spin and retain GBs of calls."""
    conn = MagicMock()
    conn.recv_match.side_effect = lambda **_kwargs: time.sleep(0.001)
    return conn


class TestMavBusRegistry(unittest.TestCase):
    """MavBus.get_or_create shares buses by device string."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_same_device_returns_same_bus(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus
        conn = _idle_connection()
        mock_mavutil.mavlink_connection.return_value = conn

        bus1 = MavBus.get_or_create("test:registry1", source_system=1)
        bus2 = MavBus.get_or_create("test:registry1", source_system=1)
        self.assertIs(bus1, bus2)
        self.assertEqual(mock_mavutil.mavlink_connection.call_count, 1)
        bus1.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_different_device_returns_different_bus(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus
        conn1, conn2 = _idle_connection(), _idle_connection()
        mock_mavutil.mavlink_connection.side_effect = [conn1, conn2]

        bus1 = MavBus.get_or_create("test:registryA", source_system=1)
        bus2 = MavBus.get_or_create("test:registryB", source_system=1)
        self.assertIsNot(bus1, bus2)
        bus1.close()
        bus2.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_close_removes_from_registry(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus, _buses
        conn = _idle_connection()
        mock_mavutil.mavlink_connection.return_value = conn

        bus = MavBus.get_or_create("test:close_reg", source_system=1)
        self.assertIn("test:close_reg", _buses)
        bus.close()
        self.assertNotIn("test:close_reg", _buses)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_get_or_create_waits_for_close_before_replacement(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus, _buses

        device = "test:close_replace_wait"
        close_entered = threading.Event()
        release_close = threading.Event()
        result = {}

        conn1 = _idle_connection()
        conn1.close.side_effect = lambda: (close_entered.set(), release_close.wait(timeout=3.0))
        conn2 = _idle_connection()
        mock_mavutil.mavlink_connection.side_effect = [conn1, conn2]

        bus1 = MavBus.get_or_create(device, source_system=1)
        closer = threading.Thread(target=bus1.close)
        closer.start()
        self.assertTrue(close_entered.wait(timeout=2.0))

        creator = threading.Thread(
            target=lambda: result.setdefault("bus", MavBus.get_or_create(device, source_system=1))
        )
        creator.start()
        time.sleep(0.2)

        self.assertTrue(creator.is_alive())
        self.assertEqual(mock_mavutil.mavlink_connection.call_count, 1)

        release_close.set()
        closer.join(timeout=2.0)
        creator.join(timeout=2.0)

        self.assertFalse(closer.is_alive())
        self.assertFalse(creator.is_alive())
        bus2 = result["bus"]
        self.assertIsNot(bus2, bus1)
        self.assertIs(_buses.get(device), bus2)
        self.assertEqual(mock_mavutil.mavlink_connection.call_count, 2)
        bus2.close()

    def test_close_only_removes_self_from_registry(self):
        from navpy.modules.vehicle.mav_bus import MavBus, _buses

        device = "test:close_identity"
        conn1 = _idle_connection()
        conn2 = _idle_connection()

        bus1 = MavBus(conn1, device)
        bus2 = MavBus(conn2, device)
        _buses[device] = bus2

        try:
            bus1.close()
            self.assertIs(_buses.get(device), bus2)
        finally:
            bus2.close()


class TestMavBusDispatch(unittest.TestCase):
    """MavBus dispatches messages to registered consumers."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_attach_and_dispatch(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        ready = threading.Event()
        msg = _make_data(1)
        sent = False

        def receive(**_kwargs):
            nonlocal sent
            ready.wait(timeout=1.0)
            if not sent:
                sent = True
                return msg
            time.sleep(0.001)

        conn = MagicMock()
        conn.recv_match.side_effect = receive
        mock_mavutil.mavlink_connection.return_value = conn

        bus = MavBus.get_or_create("test:dispatch1", source_system=1)

        consumer = MagicMock()
        bus.attach(1, consumer)

        ready.set()
        deadline = time.monotonic() + 1.0
        while not consumer.feed_message.called and time.monotonic() < deadline:
            time.sleep(0.01)
        consumer.feed_message.assert_called_once_with(msg)

        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_detach_removes_consumer(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        conn = _idle_connection()
        mock_mavutil.mavlink_connection.return_value = conn

        bus = MavBus.get_or_create("test:dispatch2", source_system=1)

        consumer = MagicMock()
        bus.attach(1, consumer)
        bus.detach(1)
        self.assertFalse(bus.has_targets)
        self.assertTrue(bus.is_closed)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_attach_rejects_closed_bus(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        conn = _idle_connection()
        mock_mavutil.mavlink_connection.return_value = conn

        bus = MavBus.get_or_create("test:attach_closed", source_system=1)
        bus.close()

        with self.assertRaises(RuntimeError):
            bus.attach(1, MagicMock())
        self.assertFalse(bus.has_targets)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_get_or_create_does_not_return_bus_while_detach_closes_last_target(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus, _buses

        device = "test:detach_attach_close_race"
        close_entered = threading.Event()
        release_close = threading.Event()
        result = {}

        conn1 = _idle_connection()
        conn1.close.side_effect = lambda: (close_entered.set(), release_close.wait(timeout=3.0))
        conn2 = _idle_connection()
        mock_mavutil.mavlink_connection.side_effect = [conn1, conn2]

        bus1 = MavBus.get_or_create(device, source_system=1)
        bus1.attach(1, MagicMock())
        detacher = threading.Thread(target=lambda: bus1.detach(1))
        detacher.start()
        self.assertTrue(close_entered.wait(timeout=2.0))

        creator = threading.Thread(
            target=lambda: result.setdefault("bus", MavBus.get_or_create(device, source_system=1))
        )
        creator.start()
        time.sleep(0.2)

        self.assertTrue(creator.is_alive())
        self.assertEqual(mock_mavutil.mavlink_connection.call_count, 1)

        release_close.set()
        detacher.join(timeout=2.0)
        creator.join(timeout=2.0)

        self.assertFalse(detacher.is_alive())
        self.assertFalse(creator.is_alive())
        bus2 = result["bus"]
        self.assertIsNot(bus2, bus1)
        self.assertIs(_buses.get(device), bus2)
        self.assertFalse(bus2.has_targets)
        bus2.close()


class TestMavBusReaderQuietPaths(unittest.TestCase):
    """Windows-UDP WSAECONNRESET is a quiet path, not a reader error."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_connection_reset_is_quiet_and_reader_survives(self, mock_mavutil):
        from navpy.modules.vehicle import mav_bus as bus_mod
        from navpy.modules.vehicle.mav_bus import MavBus

        call_count = 0
        hb = _make_heartbeat(1, MAV_TYPE_FIXED_WING)

        def recv_side_effect(**kwargs):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                # ICMP port-unreachable from a vanished UDP peer surfaces as
                # WSAECONNRESET on the next recv of the same (healthy) socket.
                raise ConnectionResetError(10054, "peer gone")
            return hb if call_count == 2 else None

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        with patch("navpy.modules.vehicle.mav_bus_reader.log.exception") as mock_exc, \
             patch("navpy.modules.vehicle.mav_bus_reader.log.debug") as mock_dbg:
            bus = MavBus(conn, "udp:0.0.0.0:5992")
            time.sleep(0.3)
            # Reader survived the reset and kept consuming messages after it.
            self.assertIn(1, bus.heartbeats)
            mock_exc.assert_not_called()
            self.assertTrue(
                any("unreachable" in str(c) for c in mock_dbg.call_args_list))
            bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_connection_reset_on_tcp_stays_loud(self, mock_mavutil):
        # On TCP a reset means the link is genuinely dead — the quiet path is
        # UDP-only, so a TCP reset must keep the loud reader-error log.
        from navpy.modules.vehicle import mav_bus as bus_mod
        from navpy.modules.vehicle.mav_bus import MavBus

        call_count = 0

        def recv_side_effect(**kwargs):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise ConnectionResetError(10054, "link dead")
            return None

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        with patch("navpy.modules.vehicle.mav_bus_reader.log.exception") as mock_exc:
            bus = MavBus(conn, "tcp:127.0.0.1:5760")
            time.sleep(0.3)
            mock_exc.assert_called()
            bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_other_oserrors_still_logged_as_reader_errors(self, mock_mavutil):
        from navpy.modules.vehicle import mav_bus as bus_mod
        from navpy.modules.vehicle.mav_bus import MavBus

        call_count = 0

        def recv_side_effect(**kwargs):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise OSError(10049, "address not available")
            return None

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        with patch("navpy.modules.vehicle.mav_bus_reader.log.exception") as mock_exc:
            bus = MavBus(conn, "test:other_oserror")
            time.sleep(0.3)
            mock_exc.assert_called()
            bus.close()


class TestMavBusReaderCooperativeScheduling(unittest.TestCase):
    """A continuous MAVLink backlog must not monopolize the Python runtime."""

    def _run_messages(self, messages, dispatch=None):
        from navpy.modules.vehicle.mav_bus_heartbeats import MavBusHeartbeats
        from navpy.modules.vehicle.mav_bus_membership import MavBusMembership
        from navpy.modules.vehicle.mav_bus_reader import MavBusReader

        stop = threading.Event()
        connection = MagicMock()
        pending = list(messages)

        def receive(**_kwargs):
            if pending:
                return pending.pop(0)
            stop.set()
            return None

        connection.recv_match.side_effect = receive
        events = []
        reader = MavBusReader(
            connection,
            "udp:0.0.0.0:5992",
            MavBusMembership(),
            MavBusHeartbeats(),
            stop,
            cooperative_yield=lambda: events.append("yield"),
        )
        reader._dispatch = dispatch or (lambda _message: events.append("dispatch"))
        reader._run()
        return events

    def test_yields_after_each_decoded_packet(self):
        self.assertEqual(
            self._run_messages([_make_data(1), _make_data(1)]),
            ["dispatch", "yield", "dispatch", "yield"],
        )

    def test_does_not_yield_when_receive_times_out(self):
        self.assertEqual(self._run_messages([]), [])

    def test_dispatch_failure_still_yields_before_reader_continues(self):
        def fail(_message):
            raise RuntimeError("receiver failed")

        self.assertEqual(self._run_messages([_make_data(1)], fail), ["yield"])


class TestMavBusHeartbeats(unittest.TestCase):
    """MavBus tracks vehicle and companion heartbeats."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_vehicle_heartbeat_tracked(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        call_count = 0
        hb = _make_heartbeat(1, MAV_TYPE_FIXED_WING)
        def recv_side_effect(**kwargs):
            nonlocal call_count
            call_count += 1
            return hb if call_count == 1 else None

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        bus = MavBus(conn, "test:hb_vehicle")
        time.sleep(0.2)
        self.assertIn(1, bus.heartbeats)
        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_companion_heartbeat_tracked(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        call_count = 0
        hb = _make_heartbeat(1, MAV_TYPE_ONBOARD_CONTROLLER)
        def recv_side_effect(**kwargs):
            nonlocal call_count
            call_count += 1
            return hb if call_count == 1 else None

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        bus = MavBus(conn, "test:hb_companion")
        time.sleep(0.2)
        self.assertIn(1, bus.companion_heartbeats)
        self.assertNotIn(1, bus.heartbeats)
        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_gcs_heartbeat_ignored(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        call_count = 0
        hb = _make_heartbeat(255, MAV_TYPE_GCS)
        def recv_side_effect(**kwargs):
            nonlocal call_count
            call_count += 1
            return hb if call_count == 1 else None

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        bus = MavBus(conn, "test:hb_gcs")
        time.sleep(0.2)
        self.assertNotIn(255, bus.heartbeats)
        self.assertNotIn(255, bus.companion_heartbeats)
        bus.close()


class TestMavBusBroadcast(unittest.TestCase):
    """MavBus broadcasts messages from unknown sources to all targets."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_unknown_source_broadcasts_to_all_targets(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        # Gate message delivery until consumers are attached
        ready = threading.Event()
        gcs_msg = _make_data(255, "NAMED_VALUE_INT")
        delivered = False

        def recv_side_effect(**kwargs):
            nonlocal delivered
            ready.wait()
            if not delivered:
                delivered = True
                return gcs_msg
            time.sleep(0.001)

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        bus = MavBus(conn, "test:broadcast_all")

        consumer1 = MagicMock()
        consumer2 = MagicMock()
        bus.attach(1, consumer1)
        bus.attach(2, consumer2)
        ready.set()

        time.sleep(0.3)

        # Both consumers should receive the message from the unknown source
        consumer1.feed_message.assert_called_with(gcs_msg)
        consumer2.feed_message.assert_called_with(gcs_msg)

        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_known_source_dispatches_to_single_target(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        # Gate message delivery until consumers are attached
        ready = threading.Event()
        vehicle_msg = _make_data(1, "GLOBAL_POSITION_INT")
        delivered = False

        def recv_side_effect(**kwargs):
            nonlocal delivered
            ready.wait()
            if not delivered:
                delivered = True
                return vehicle_msg
            time.sleep(0.001)

        conn = MagicMock()
        conn.recv_match.side_effect = recv_side_effect
        bus = MavBus(conn, "test:dispatch_single")

        consumer1 = MagicMock()
        consumer2 = MagicMock()
        bus.attach(1, consumer1)
        bus.attach(2, consumer2)
        ready.set()

        time.sleep(0.3)

        # Only consumer1 (target for sys_id 1) should receive it
        consumer1.feed_message.assert_called_with(vehicle_msg)
        consumer2.feed_message.assert_not_called()

        bus.close()


class TestMavBusWaitHeartbeat(unittest.TestCase):
    """MavBus.wait_heartbeat polls the heartbeat table."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_wait_heartbeat_success(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        conn = _idle_connection()
        mock_mavutil.mavlink_connection.return_value = conn

        bus = MavBus.get_or_create("test:wait_hb1", source_system=1)
        bus._heartbeats.observe(_make_heartbeat(1))
        self.assertTrue(bus.wait_heartbeat(1, timeout=0.5))
        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_wait_heartbeat_timeout(self, mock_mavutil):
        from navpy.modules.vehicle.mav_bus import MavBus

        conn = _idle_connection()
        mock_mavutil.mavlink_connection.return_value = conn

        bus = MavBus.get_or_create("test:wait_hb2", source_system=1)
        self.assertFalse(bus.wait_heartbeat(99, timeout=0.3))
        bus.close()


class TestEnlargeRecvBuffer(unittest.TestCase):
    """SO_RCVBUF hardening is best-effort and transport-aware."""

    def test_socket_backed_conn_gets_rcvbuf_bumped(self):
        import socket as _socket
        from navpy.modules.vehicle.mav_bus import _enlarge_recv_buffer, _RCVBUF_BYTES
        sock = MagicMock()
        sock.getsockopt.return_value = _RCVBUF_BYTES
        conn = MagicMock()
        conn.port = sock
        _enlarge_recv_buffer(conn, "udp:test")
        sock.setsockopt.assert_called_once_with(
            _socket.SOL_SOCKET, _socket.SO_RCVBUF, _RCVBUF_BYTES
        )

    def test_serial_like_conn_without_setsockopt_is_skipped(self):
        from navpy.modules.vehicle.mav_bus import _enlarge_recv_buffer

        class _SerialPort:  # pyserial-like: no setsockopt
            pass

        conn = MagicMock()
        conn.port = _SerialPort()
        _enlarge_recv_buffer(conn, "COM3")  # must not raise

    def test_none_port_is_skipped(self):
        from navpy.modules.vehicle.mav_bus import _enlarge_recv_buffer
        conn = MagicMock()
        conn.port = None
        _enlarge_recv_buffer(conn, "x")  # must not raise

    def test_setsockopt_oserror_is_swallowed(self):
        from navpy.modules.vehicle.mav_bus import _enlarge_recv_buffer
        sock = MagicMock()
        sock.setsockopt.side_effect = OSError("no buffer space")
        conn = MagicMock()
        conn.port = sock
        _enlarge_recv_buffer(conn, "udp:test")  # must not raise


if __name__ == "__main__":
    unittest.main()
