"""Tests for MavBus shared connection and VehicleManager integration."""
from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock, patch

from gcs.backend.vehicle_manager import VehicleManager
from navpy.modules.vehicle.mav_bus import MavBus, MavBusClosedError


class _FakeBus:
    def __init__(self, heartbeats=None, has_targets=False, is_closed=False):
        self.heartbeats = heartbeats or {}
        self.has_targets = has_targets
        self.is_closed = is_closed
        self.close = MagicMock(side_effect=self._close)

    def _close(self):
        self.is_closed = True


class _ClosesAfterHeartbeatBus(_FakeBus):
    @property
    def heartbeats(self):
        self.is_closed = True
        return {1: time.time() + 60.0}

    @heartbeats.setter
    def heartbeats(self, value):
        pass


class _ConnectsDuringScanBus(_FakeBus):
    def __init__(self, mgr):
        super().__init__()
        self._mgr = mgr

    @property
    def heartbeats(self):
        self._mgr._vehicles[1] = MagicMock()
        return {1: time.time() + 60.0}

    @heartbeats.setter
    def heartbeats(self, value):
        pass


class TestMavBusSendLock:
    """Verify that vehicles on the same MavBus share one send lock."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_vehicles_on_same_device_share_lock(self, mock_mavutil):
        """The public bus registry returns one serialized link per device."""
        conn = MagicMock()
        conn.recv_match = MagicMock(return_value=None)
        mock_mavutil.mavlink_connection.return_value = conn

        bus = None
        try:
            bus = MavBus.get_or_create("test:shared-link-same", source_system=255)
            same_bus = MavBus.get_or_create(
                "test:shared-link-same", source_system=255,
            )
            assert bus is same_bus
            assert bus.send_lock is same_bus.send_lock
            assert mock_mavutil.mavlink_connection.call_count == 1
        finally:
            if bus is not None:
                bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_vehicles_on_different_devices_have_separate_locks(self, mock_mavutil):
        """Distinct public bus components own distinct serialization locks."""
        conn1, conn2 = MagicMock(), MagicMock()
        conn1.recv_match = MagicMock(return_value=None)
        conn2.recv_match = MagicMock(return_value=None)
        mock_mavutil.mavlink_connection.side_effect = [conn1, conn2]

        bus1 = bus2 = None
        try:
            bus1 = MavBus.get_or_create("test:shared-link-a", source_system=255)
            bus2 = MavBus.get_or_create("test:shared-link-b", source_system=255)
            assert bus1.send_lock is not bus2.send_lock
            assert mock_mavutil.mavlink_connection.call_count == 2
        finally:
            if bus1 is not None:
                bus1.close()
            if bus2 is not None:
                bus2.close()

    def test_vehicle_mav_uses_bus_lock(self):
        """A public vehicle send waits behind its supplied bus lock."""
        from navpy.modules.vehicle.vehicle_mav import VehicleMav

        conn = MagicMock()
        conn.recv_match = MagicMock(return_value=None)
        bus = MavBus(conn, "test:vehicle-send-lock")
        v = VehicleMav(
            device="unused",
            target_system=1,
            wait_heartbeat=False,
            send_heartbeat=False,
            skip_mission_download=True,
            bus=bus,
        )
        started = threading.Event()
        completed = threading.Event()

        def send():
            started.set()
            v.send_manual_control(0, 0, 500, 0)
            completed.set()

        bus.send_lock.acquire()
        try:
            sender = threading.Thread(target=send, daemon=True)
            sender.start()
            assert started.wait(timeout=0.5)
            assert not completed.wait(timeout=0.05)
        finally:
            bus.send_lock.release()
        sender.join(timeout=0.5)
        try:
            assert completed.is_set()
            conn.mav.manual_control_send.assert_called_once()
        finally:
            v.close()

    def test_add_vehicle_same_sysid_is_atomic(self):
        """Concurrent connects for one sys_id must not create duplicate VehicleMav."""
        mgr = VehicleManager()
        first_constructor_started = threading.Event()
        constructed = []

        def fake_vehicle(*_args, **_kwargs):
            constructed.append(object())
            if len(constructed) == 1:
                first_constructor_started.set()
                time.sleep(0.1)
            vehicle = MagicMock()
            vehicle.on_message = MagicMock()
            vehicle.close = MagicMock()
            return vehicle

        results = []

        def worker():
            results.append(mgr.add_vehicle("udp:127.0.0.1:14550", sys_id=1, name="v1"))

        try:
            with patch.object(mgr, "_get_or_create_bus", return_value=MagicMock()), \
                 patch.object(mgr, "_probe_mission_async"), \
                 patch("gcs.backend.vehicle_manager.VehicleMav", side_effect=fake_vehicle) as ctor:
                t1 = threading.Thread(target=worker)
                t1.start()
                assert first_constructor_started.wait(timeout=2.0)
                t2 = threading.Thread(target=worker)
                t2.start()
                t1.join(timeout=2.0)
                t2.join(timeout=2.0)

                assert not t1.is_alive()
                assert not t2.is_alive()
                assert ctor.call_count == 1
                assert len(results) == 2
                assert results[0] is results[1]
        finally:
            mgr.shutdown()

    def test_add_vehicle_retries_once_when_bus_closes_before_attach(self):
        """A connect racing last-target disconnect should retry on a fresh bus."""
        mgr = VehicleManager()
        vehicles = []

        def fake_vehicle(*_args, **_kwargs):
            if not vehicles:
                vehicles.append(None)
                raise MavBusClosedError("closing")
            vehicle = MagicMock()
            vehicle.on_message = MagicMock()
            vehicle.close = MagicMock()
            vehicles.append(vehicle)
            return vehicle

        try:
            with patch.object(mgr, "_get_or_create_bus", return_value=MagicMock()) as get_bus, \
                 patch.object(mgr, "_drop_stale_bus") as drop_stale, \
                 patch.object(mgr, "_probe_mission_async"), \
                 patch("gcs.backend.vehicle_manager.VehicleMav", side_effect=fake_vehicle) as ctor:
                entry = mgr.add_vehicle("udp:127.0.0.1:14550", sys_id=1, name="v1")

            assert entry.sys_id == 1
            assert ctor.call_count == 2
            assert get_bus.call_count == 2
            drop_stale.assert_called_once_with("udp:127.0.0.1:14550")
        finally:
            mgr.shutdown()

    def test_add_vehicle_binds_vehicle_to_manager_selected_bus(self):
        """The manager must track the same bus the constructed vehicle uses."""
        mgr = VehicleManager()
        device = "udp:127.0.0.1:14550"
        selected_bus = _FakeBus(has_targets=True)
        mgr._buses[device] = selected_bus

        def fake_vehicle(*_args, **_kwargs):
            vehicle = MagicMock()
            vehicle.on_message = MagicMock()
            vehicle.close = MagicMock()
            return vehicle

        try:
            with patch.object(mgr, "_probe_mission_async"), \
                 patch("gcs.backend.vehicle_manager.VehicleMav", side_effect=fake_vehicle) as ctor:
                entry = mgr.add_vehicle(device, sys_id=1, name="v1")

            assert ctor.call_args.kwargs["bus"] is selected_bus
            assert mgr._buses[device] is selected_bus
        finally:
            mgr.shutdown()


class TestVehicleManagerScanRace:
    def test_scan_discards_heartbeat_from_bus_closed_during_scan(self):
        """A scan that overlaps disconnect must not reconnect from stale heartbeats."""
        mgr = VehicleManager()
        bus = _ClosesAfterHeartbeatBus()

        with patch.object(mgr, "_get_or_create_bus", return_value=bus):
            found = mgr.scan("test:stale-scan", timeout=0.01)

        assert found == []

    def test_scan_does_not_pop_replacement_bus_when_stale_bus_finds_nothing(self):
        """An old no-result scan must not remove a newer bus from manager state."""
        mgr = VehicleManager()
        stale_bus = _FakeBus()
        replacement_bus = _FakeBus(has_targets=True)
        mgr._buses["test:replace"] = replacement_bus

        with patch.object(mgr, "_get_or_create_bus", return_value=stale_bus):
            found = mgr.scan("test:replace", timeout=0.01)

        assert found == []
        stale_bus.close.assert_not_called()
        assert mgr._buses["test:replace"] is replacement_bus

    def test_scan_filters_vehicle_connected_before_scan_returns(self):
        """Concurrent scans should not report vehicles another request connected."""
        mgr = VehicleManager()
        bus = _ConnectsDuringScanBus(mgr)

        with patch.object(mgr, "_get_or_create_bus", return_value=bus):
            found = mgr.scan("test:already-connected", timeout=0.01)

        assert found == []
