"""Tests for VehicleMav preflight telemetry: sensor_health, ekf_status, snapshot."""
import threading
import types
import unittest
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_SYS_STATUS_SENSOR_3D_GYRO,
    MAV_SYS_STATUS_SENSOR_3D_ACCEL,
    MAV_SYS_STATUS_SENSOR_3D_MAG,
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
    MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
    MAV_COMP_ID_AUTOPILOT1,
)

from navpy.modules.vehicle.vehicle_mav import VehicleMav


def _make_vehicle(test_case: unittest.TestCase | None = None):
    """Construct through the public composition root on a connection-free bus."""
    connection = types.SimpleNamespace(mav=MagicMock())
    bus = MagicMock(
        conn=connection,
        send_lock=threading.RLock(),
        heartbeats={},
    )
    bus.reserve.return_value = MagicMock()
    vehicle = VehicleMav(
        "unused",
        target_system=1,
        logger=MagicMock(),
        skip_mission_download=True,
        wait_heartbeat=False,
        send_heartbeat=False,
        bus=bus,
    )
    if test_case is not None:
        test_case.addCleanup(vehicle.close)
    return vehicle


def _feed(vehicle, message_type: str, message) -> None:
    message.get_msgId = lambda: 0
    message.get_srcSystem = lambda: 1
    message.get_srcComponent = lambda: MAV_COMP_ID_AUTOPILOT1
    message.get_seq = lambda: 0
    message.get_type = lambda: message_type
    vehicle.feed_message(message)


def _sys_status(present=0, enabled=0, health=0):
    return types.SimpleNamespace(
        onboard_control_sensors_present=present,
        onboard_control_sensors_enabled=enabled,
        onboard_control_sensors_health=health,
        voltage_battery=0,
        current_battery=-1,
    )


def _ekf_report(flags=0, vel=0.0, ph=0.0, pv=0.0, comp=0.0, terr=0.0):
    return types.SimpleNamespace(
        flags=flags,
        velocity_variance=vel,
        pos_horiz_variance=ph,
        pos_vert_variance=pv,
        compass_variance=comp,
        terrain_alt_variance=terr,
    )


_ALL_SENSORS = (
    MAV_SYS_STATUS_SENSOR_3D_GYRO
    | MAV_SYS_STATUS_SENSOR_3D_ACCEL
    | MAV_SYS_STATUS_SENSOR_3D_MAG
    | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    | MAV_SYS_STATUS_SENSOR_RC_RECEIVER
)


class TestSensorHealth(unittest.TestCase):
    def test_all_none_when_no_sys_status(self):
        v = _make_vehicle()
        self.assertEqual(
            v.sensor_health,
            {"gyro": None, "accel": None, "mag": None,
             "abs_pressure": None, "rc": None},
        )

    def test_none_when_not_present(self):
        v = _make_vehicle()
        _feed(v, "SYS_STATUS", _sys_status(present=0, enabled=0, health=0))
        self.assertIsNone(v.sensor_health["gyro"])

    def test_none_when_present_but_not_enabled(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_SENSOR_3D_GYRO
        _feed(v, "SYS_STATUS", _sys_status(present=bit, enabled=0, health=bit))
        self.assertIsNone(v.sensor_health["gyro"])

    def test_true_when_present_enabled_healthy(self):
        v = _make_vehicle()
        _feed(v, "SYS_STATUS", _sys_status(
            present=_ALL_SENSORS, enabled=_ALL_SENSORS, health=_ALL_SENSORS,
        ))
        sh = v.sensor_health
        self.assertTrue(all(sh[k] is True for k in sh))

    def test_false_when_enabled_but_unhealthy(self):
        v = _make_vehicle()
        gyro = MAV_SYS_STATUS_SENSOR_3D_GYRO
        # gyro present+enabled but unhealthy; others healthy
        _feed(v, "SYS_STATUS", _sys_status(
            present=_ALL_SENSORS, enabled=_ALL_SENSORS,
            health=_ALL_SENSORS & ~gyro,
        ))
        sh = v.sensor_health
        self.assertIs(sh["gyro"], False)
        self.assertIs(sh["accel"], True)

    def test_rc_isolated(self):
        v = _make_vehicle()
        rc = MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        _feed(v, "SYS_STATUS", _sys_status(present=rc, enabled=rc, health=rc))
        sh = v.sensor_health
        self.assertIs(sh["rc"], True)
        self.assertIsNone(sh["gyro"])


class TestEkfStatus(unittest.TestCase):
    def test_none_when_no_report(self):
        v = _make_vehicle()
        self.assertIsNone(v.ekf_status)

    def test_decodes_fields(self):
        v = _make_vehicle()
        _feed(v, "EKF_STATUS_REPORT", _ekf_report(
            flags=831, vel=0.1, ph=0.2, pv=0.3, comp=0.4, terr=0.9,
        ))
        ekf = v.ekf_status
        self.assertEqual(ekf["flags"], 831)
        self.assertAlmostEqual(ekf["velocity_variance"], 0.1)
        self.assertAlmostEqual(ekf["pos_horiz_variance"], 0.2)
        self.assertAlmostEqual(ekf["pos_vert_variance"], 0.3)
        self.assertAlmostEqual(ekf["compass_variance"], 0.4)
        self.assertAlmostEqual(ekf["terrain_alt_variance"], 0.9)

    def test_missing_fields_default_zero(self):
        v = _make_vehicle()
        _feed(v, "EKF_STATUS_REPORT", types.SimpleNamespace(flags=1))
        ekf = v.ekf_status
        self.assertEqual(ekf["flags"], 1)
        self.assertEqual(ekf["velocity_variance"], 0.0)


class TestSnapshotPreflightFields(unittest.TestCase):
    """VehicleEntry.snapshot() carries ekf + sensors."""

    def _entry_with(self, sys_status=None, ekf=None):
        # Exercise the real snapshot against a normally composed VehicleMav.
        from gcs.backend.vehicle_manager import VehicleEntry

        v = _make_vehicle(self)
        if sys_status is not None:
            _feed(v, "SYS_STATUS", sys_status)
        if ekf is not None:
            _feed(v, "EKF_STATUS_REPORT", ekf)

        return VehicleEntry(1, v, "test")

    def test_snapshot_includes_sensors_and_ekf(self):
        entry = self._entry_with(
            sys_status=_sys_status(
                present=_ALL_SENSORS, enabled=_ALL_SENSORS, health=_ALL_SENSORS,
            ),
            ekf=_ekf_report(flags=831, vel=0.05),
        )
        snap = entry.snapshot()
        self.assertIn("sensors", snap)
        self.assertIn("ekf", snap)
        self.assertIs(snap["sensors"]["gyro"], True)
        self.assertEqual(snap["ekf"]["flags"], 831)

    def test_snapshot_ekf_none_when_absent(self):
        entry = self._entry_with()
        snap = entry.snapshot()
        self.assertIsNone(snap["ekf"])
        self.assertIsNone(snap["sensors"]["gyro"])


if __name__ == "__main__":
    unittest.main()
