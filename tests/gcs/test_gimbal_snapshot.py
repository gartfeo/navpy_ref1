from gcs.backend import vehicle_camera
"""Tests for GCS gimbal telemetry snapshot caching."""
import math
from types import SimpleNamespace

import orjson
import pytest
from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_CAMERA, MAV_COMP_ID_CAMERA2

from gcs.backend import vehicle_manager as vehicle_manager_mod
from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.vehicle_manager import VehicleEntry
from navpy.modules.vision.mavlink_camera_components import (
    CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID,
)


class FakeVehicle:
    def __init__(self):
        self.callbacks = {}
        self.battery_level = 80
        self.battery_voltage = 24.0
        self.battery_current = 3.0
        self.get_mode = None
        self.attitude = None
        self.is_armed = False
        self.heading = 90
        self.air_speed = 22.0
        self.ground_speed = 20.0
        self.gps_fix_type = 3
        self.gps_satellites = 12
        self.gps_hacc = 0.8
        self.link_ok = True
        self.link_quality = 100
        self.mission_items_next = 1
        self.mission_items_count = 10
        self.prearm_ok = True
        self.prearm_check_state = "ok"
        self.ekf_status = None
        self.sensor_health = {}
        self.airspeed_present = None
        self.airspeed_healthy = None
        self.rc3_raw = 1500

    def on_message(self, name, callback):
        self.callbacks.setdefault(name, []).append(callback)

    def location(self, is_relative):
        if is_relative:
            return SimpleNamespace(alt=120.0)
        return SimpleNamespace(lat=32.0, lng=34.8, alt=500.0)

    def emit(self, name, msg):
        for callback in self.callbacks.get(name, []):
            callback(msg)


def _make_entry(monkeypatch, *, wall=1000.0, monotonic=10.0):
    clock = {"wall": wall, "monotonic": monotonic}
    monkeypatch.setattr(vehicle_manager_mod.time, "time", lambda: clock["wall"])
    monkeypatch.setattr(
        vehicle_manager_mod.time,
        "monotonic",
        lambda: clock["monotonic"],
    )
    vehicle = FakeVehicle()
    entry = VehicleEntry(1, vehicle, "uav1")
    return entry, vehicle, clock


# Default src_system 1 = the shared system id of vehicle sys_id 1, which every
# test VehicleEntry uses. The companion is told apart from the autopilot by
# COMPONENT id: 191 for its own traffic, the camera components for optics.
AUTOPILOT_COMPONENT_ID = 1


def _gimbal_msg(device_id=1, q=None, flags=32, failure_flags=0, time_boot_ms=100,
                src_system=1, src_component=COMPANION_COMPONENT_ID):
    return SimpleNamespace(
        gimbal_device_id=device_id,
        q=[1.0, 0.0, 0.0, 0.0] if q is None else q,
        flags=flags,
        failure_flags=failure_flags,
        time_boot_ms=time_boot_ms,
        get_srcSystem=lambda: src_system,
        get_srcComponent=lambda: src_component,
    )


def _camera_fov_msg(component=MAV_COMP_ID_CAMERA, hfov=40.0, vfov=25.0, time_boot_ms=101,
                    src_system=1):
    return SimpleNamespace(
        get_srcSystem=lambda: src_system,
        get_srcComponent=lambda: component,
        hfov=hfov,
        vfov=vfov,
        time_boot_ms=time_boot_ms,
    )


def _camera_settings_msg(component=MAV_COMP_ID_CAMERA, zoom=2.5, time_boot_ms=102,
                         src_system=1):
    return SimpleNamespace(
        get_srcSystem=lambda: src_system,
        get_srcComponent=lambda: component,
        zoomLevel=zoom,
        time_boot_ms=time_boot_ms,
    )


def test_registers_gimbal_status_callback(monkeypatch):
    _, vehicle, _ = _make_entry(monkeypatch)

    assert "GIMBAL_DEVICE_ATTITUDE_STATUS" in vehicle.callbacks
    assert len(vehicle.callbacks["GIMBAL_DEVICE_ATTITUDE_STATUS"]) == 1
    assert "CAMERA_FOV_STATUS" in vehicle.callbacks
    assert len(vehicle.callbacks["CAMERA_FOV_STATUS"]) == 1
    assert "CAMERA_SETTINGS" in vehicle.callbacks
    assert len(vehicle.callbacks["CAMERA_SETTINGS"]) == 1


def test_snapshot_includes_gimbal_payload(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit(
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        _gimbal_msg(device_id=2, q=[0.5, 0.5, 0.5, 0.5], flags=32, failure_flags=1),
    )

    snap = entry.snapshot()
    assert snap["gimbals"] == {
        "2": {
            "device_id": 2,
            "q": [0.5, 0.5, 0.5, 0.5],
            "flags": 32,
            "failure_flags": 1,
            "time_boot_ms": 100,
            "updated_at": 1000.0,
            "stale": False,
        }
    }


def test_multi_device_gimbals_are_cached_independently(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))
    vehicle.emit(
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        _gimbal_msg(device_id=2, q=[0.0, 1.0, 0.0, 0.0]),
    )

    gimbals = entry.snapshot()["gimbals"]
    assert sorted(gimbals.keys()) == ["1", "2"]
    assert gimbals["1"]["q"] == [1.0, 0.0, 0.0, 0.0]
    assert gimbals["2"]["q"] == [0.0, 1.0, 0.0, 0.0]


def test_camera_optics_merge_into_matching_device(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=2))
    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(component=MAV_COMP_ID_CAMERA2))
    vehicle.emit("CAMERA_SETTINGS", _camera_settings_msg(component=MAV_COMP_ID_CAMERA2, zoom=3.0))

    gimbal = entry.snapshot()["gimbals"]["2"]
    assert gimbal["stale"] is False
    assert gimbal["optics_stale"] is False
    assert gimbal["fov_h_rad"] == pytest.approx(math.radians(40.0))
    assert gimbal["fov_v_rad"] == pytest.approx(math.radians(25.0))
    assert gimbal["optics_time_boot_ms"] == 101
    assert gimbal["zoom_level"] == 3.0
    assert gimbal["settings_time_boot_ms"] == 102


def test_all_camera_components_map_to_gimbal_device_ids(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch)

    for device_id, component_id in CAMERA_COMPONENT_BY_GIMBAL_DEVICE_ID.items():
        vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(component=component_id))

    assert sorted(entry.snapshot()["gimbals"].keys()) == ["1", "2", "3", "4", "5", "6"]


def test_camera_optics_do_not_refresh_attitude_staleness(monkeypatch):
    entry, vehicle, clock = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))
    clock["monotonic"] += vehicle_manager_mod.GIMBAL_TELEMETRY_STALE_S + 0.1
    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(component=MAV_COMP_ID_CAMERA))

    gimbal = entry.snapshot()["gimbals"]["1"]
    assert gimbal["stale"] is True
    assert gimbal["optics_stale"] is False


def test_camera_optics_can_be_stale_independently(monkeypatch):
    entry, vehicle, clock = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))
    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(component=MAV_COMP_ID_CAMERA))
    clock["monotonic"] += vehicle_manager_mod.CAMERA_OPTICS_STALE_S + 0.1
    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))

    gimbal = entry.snapshot()["gimbals"]["1"]
    assert gimbal["stale"] is False
    assert gimbal["optics_stale"] is True


@pytest.mark.parametrize("hfov,vfov", [(0.0, 25.0), (40.0, 180.0), (math.nan, 25.0)])
def test_invalid_camera_fov_is_ignored(monkeypatch, hfov, vfov):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(hfov=hfov, vfov=vfov))

    assert entry.snapshot()["gimbals"] == {}


def test_unknown_camera_component_is_ignored(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(component=99))
    vehicle.emit("CAMERA_SETTINGS", _camera_settings_msg(component=99))

    assert entry.snapshot()["gimbals"] == {}


def test_stale_uses_monotonic_clock_and_refreshes(monkeypatch):
    entry, vehicle, clock = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))
    clock["wall"] += 3600.0
    clock["monotonic"] += 1.0
    assert entry.snapshot()["gimbals"]["1"]["stale"] is False

    clock["monotonic"] += vehicle_manager_mod.GIMBAL_TELEMETRY_STALE_S + 0.1
    assert entry.snapshot()["gimbals"]["1"]["stale"] is True

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))
    assert entry.snapshot()["gimbals"]["1"]["stale"] is False


@pytest.mark.parametrize("device_id", [0, 7, True, "1"])
def test_ignores_invalid_gimbal_device_ids(monkeypatch, device_id):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=device_id))

    assert entry.snapshot()["gimbals"] == {}


@pytest.mark.parametrize(
    "q",
    [
        [1.0, 0.0, 0.0],
        [math.nan, 0.0, 0.0, 0.0],
        [math.inf, 0.0, 0.0, 0.0],
    ],
)
def test_ignores_invalid_quaternions(monkeypatch, q):
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(q=q))

    assert entry.snapshot()["gimbals"] == {}


def test_gimbal_snapshot_is_orjson_serializable(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch)
    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))

    payload = {"vehicles": [entry.snapshot()]}

    encoded = orjson.dumps(payload)
    assert b'"gimbals"' in encoded


def test_gimbal_telem_log_off_by_default(monkeypatch, caplog):
    """The opt-in footprint-driver telemetry log stays silent unless the
    GCS_LOG_GIMBAL_TELEM flag is set — no 5 Hz spam in the default backend log."""
    monkeypatch.setattr(vehicle_camera, "_LOG_GIMBAL_TELEM", False)
    entry, vehicle, _ = _make_entry(monkeypatch)

    with caplog.at_level("INFO", logger="gcs.backend.vehicle_manager"):
        vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1))

    assert "GIMBAL_TELEM" not in caplog.text


def test_gimbal_telem_log_when_enabled(monkeypatch, caplog):
    """With the flag on, each gimbal attitude sample logs sys_id, device_id and
    the raw quaternion so footprint smoothness is measurable offline."""
    monkeypatch.setattr(vehicle_camera, "_LOG_GIMBAL_TELEM", True)
    entry, vehicle, _ = _make_entry(monkeypatch)

    with caplog.at_level("INFO", logger="gcs.backend.vehicle_manager"):
        vehicle.emit(
            "GIMBAL_DEVICE_ATTITUDE_STATUS",
            _gimbal_msg(device_id=2, q=[0.5, 0.5, 0.5, 0.5], time_boot_ms=100,
                       src_system=1, src_component=COMPANION_COMPONENT_ID),
        )

    assert "GIMBAL_TELEM" in caplog.text
    assert "sys=1" in caplog.text
    assert "dev=2" in caplog.text
    assert "tb=100" in caplog.text
    # source identity distinguishes each companion's stream (cross-delivery)
    assert "srcsys=1" in caplog.text
    assert "srccomp=191" in caplog.text


def test_gimbal_telem_from_other_vehicle_companion_is_dropped(monkeypatch):
    """Cross-delivered gimbal telemetry from ANOTHER vehicle's companion (a
    different MAVLink source system) must NOT pollute this vehicle's gimbal
    snapshot — otherwise the map footprint flickers between vehicles' poses."""
    entry, vehicle, _ = _make_entry(monkeypatch)   # VehicleEntry sys_id 1

    # sys_id 2's companion (system id 2) cross-delivering onto sys 1's link.
    vehicle.emit(
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        _gimbal_msg(device_id=1, q=[0.5, 0.5, 0.5, 0.5], src_system=2),
    )
    assert entry.snapshot()["gimbals"] == {}

    # This vehicle's own companion (system id 1) is still accepted.
    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1, src_system=1))
    assert "1" in entry.snapshot()["gimbals"]


def test_cross_delivered_camera_optics_are_dropped(monkeypatch):
    """FOV/settings from another vehicle's companion are also filtered by
    source, so a vehicle's footprint size/zoom isn't set by a peer's optics."""
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1, src_system=1))
    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(src_system=2))   # peer companion
    vehicle.emit("CAMERA_SETTINGS", _camera_settings_msg(src_system=2))

    gimbal = entry.snapshot()["gimbals"]["1"]
    assert "fov_h_rad" not in gimbal      # peer FOV rejected
    assert "zoom_level" not in gimbal      # peer settings rejected


def test_own_autopilot_component_is_not_treated_as_companion_gimbal(monkeypatch):
    """Under the shared sysid the aircraft's OWN autopilot carries the same
    srcSystem as its companion, so system id alone no longer discriminates.
    Gimbal telemetry from component 1 (the autopilot, e.g. its own mount) must
    NOT be attributed to the companion's camera."""
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit(
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        _gimbal_msg(device_id=1, src_system=1,
                    src_component=AUTOPILOT_COMPONENT_ID),
    )
    assert entry.snapshot()["gimbals"] == {}

    # Same system id, companion component -> accepted.
    vehicle.emit(
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        _gimbal_msg(device_id=1, src_system=1,
                    src_component=COMPANION_COMPONENT_ID),
    )
    assert "1" in entry.snapshot()["gimbals"]


def test_companion_camera_components_are_accepted_for_optics(monkeypatch):
    """The companion publishes CAMERA_FOV_STATUS / CAMERA_SETTINGS from the
    per-camera component ids, not 191, so the telemetry filter must accept the
    whole companion-owned component set or footprint optics never arrive."""
    entry, vehicle, _ = _make_entry(monkeypatch)

    vehicle.emit("GIMBAL_DEVICE_ATTITUDE_STATUS", _gimbal_msg(device_id=1, src_system=1))
    vehicle.emit("CAMERA_FOV_STATUS", _camera_fov_msg(src_system=1))
    vehicle.emit("CAMERA_SETTINGS", _camera_settings_msg(src_system=1))

    gimbal = entry.snapshot()["gimbals"]["1"]
    assert gimbal["fov_h_rad"] == pytest.approx(math.radians(40.0))
    assert gimbal["zoom_level"] == pytest.approx(2.5)
