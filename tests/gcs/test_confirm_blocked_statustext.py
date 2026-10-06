"""Tests for the CONF-03 recognition-gate blocked-state STATUSTEXT parser
(D-15/D-16/D-17) — source-filtered per-UAV field + snapshot exposure.
"""
from types import SimpleNamespace

from gcs.backend import vehicle_manager as vehicle_manager_mod
from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.vehicle_manager import (
    CONFIRM_BLOCKED_STALE_S,
    VehicleEntry,
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


def _make_entry(monkeypatch, sys_id=1, *, wall=1000.0, monotonic=10.0):
    clock = {"wall": wall, "monotonic": monotonic}
    monkeypatch.setattr(vehicle_manager_mod.time, "time", lambda: clock["wall"])
    monkeypatch.setattr(
        vehicle_manager_mod.time,
        "monotonic",
        lambda: clock["monotonic"],
    )
    vehicle = FakeVehicle()
    entry = VehicleEntry(sys_id, vehicle, f"uav{sys_id}")
    return entry, vehicle, clock


# A companion STATUSTEXT carries its aircraft's system id and the companion
# component id (191); the autopilot's own STATUSTEXT shares that system id
# but uses component 1.
AUTOPILOT_COMPONENT_ID = 1


def _statustext_msg(text, src_system=1, severity=6,
                    src_component=COMPANION_COMPONENT_ID):
    return SimpleNamespace(
        text=text,
        severity=severity,
        get_srcSystem=lambda: src_system,
        get_srcComponent=lambda: src_component,
    )


def test_confirm_blocked_pixels_reason_parsed_with_task_id(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch, sys_id=1)

    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:pixels|12/20|7", src_system=1))

    assert entry._confirm_blocked == {"reason": "pixels", "detail": "12/20", "task_id": 7}


def test_confirm_blocked_zoom_reason_no_detail(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch, sys_id=1)

    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:zoom||7", src_system=1))

    assert entry._confirm_blocked == {"reason": "zoom", "detail": "", "task_id": 7}


def test_confirm_blocked_clear_resets_field(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch, sys_id=1)
    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:pixels|12/20|7", src_system=1))
    assert entry._confirm_blocked is not None

    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:clear", src_system=1))

    assert entry._confirm_blocked is None


def test_confirm_blocked_cross_delivery_source_filter_two_entries(monkeypatch):
    """Pitfall 2 (load-bearing): a CONFIRM_BLOCKED STATUSTEXT sourced from
    UAV A's companion must only update UAV A's entry, never UAV B's, even
    though both entries receive the same cross-delivered message over the
    shared link."""
    entry_a, vehicle_a, clock = _make_entry(monkeypatch, sys_id=1)
    entry_b, vehicle_b, _ = _make_entry(monkeypatch, sys_id=2)
    # Reuse the same clock for both so monkeypatch state stays consistent.
    monkeypatch.setattr(vehicle_manager_mod.time, "time", lambda: clock["wall"])
    monkeypatch.setattr(vehicle_manager_mod.time, "monotonic", lambda: clock["monotonic"])

    msg = _statustext_msg("CONFIRM_BLOCKED:pixels|12/20|7", src_system=1)  # UAV 1's companion
    vehicle_a.emit("STATUSTEXT", msg)
    vehicle_b.emit("STATUSTEXT", msg)

    assert entry_a._confirm_blocked == {"reason": "pixels", "detail": "12/20", "task_id": 7}
    assert entry_b._confirm_blocked is None


def test_confirm_blocked_ignored_when_not_from_own_companion(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch, sys_id=1)

    # src_system=2 is UAV 2's companion, not UAV 1's.
    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:pixels|12/20|7", src_system=2))

    assert entry._confirm_blocked is None


def test_snapshot_includes_confirm_blocked_field(monkeypatch):
    entry, vehicle, _ = _make_entry(monkeypatch, sys_id=1)
    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:zoom||3", src_system=1))

    snap = entry.snapshot()

    assert snap["confirm_blocked"] == {"reason": "zoom", "detail": "", "task_id": 3}


def test_snapshot_confirm_blocked_defaults_to_none(monkeypatch):
    entry, _, _ = _make_entry(monkeypatch, sys_id=1)

    assert entry.snapshot()["confirm_blocked"] is None


def test_stale_confirm_blocked_clears_as_safety_net(monkeypatch):
    entry, vehicle, clock = _make_entry(monkeypatch, sys_id=1, monotonic=10.0)
    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:pixels|12/20|7", src_system=1))
    assert entry._confirm_blocked is not None

    # Advance monotonic clock well past CONFIRM_BLOCKED_STALE_S without an
    # explicit clear signal -- the staleness safety net must self-heal.
    clock["monotonic"] = 10.0 + CONFIRM_BLOCKED_STALE_S + 1.0

    snap = entry.snapshot()

    assert snap["confirm_blocked"] is None
    assert entry._confirm_blocked is None


def test_fresh_confirm_blocked_survives_snapshot(monkeypatch):
    entry, vehicle, clock = _make_entry(monkeypatch, sys_id=1, monotonic=10.0)
    vehicle.emit("STATUSTEXT", _statustext_msg("CONFIRM_BLOCKED:pixels|12/20|7", src_system=1))

    clock["monotonic"] = 10.0 + 1.0  # well within CONFIRM_BLOCKED_STALE_S

    snap = entry.snapshot()

    assert snap["confirm_blocked"] == {"reason": "pixels", "detail": "12/20", "task_id": 7}
