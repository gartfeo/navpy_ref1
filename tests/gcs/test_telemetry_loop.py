"""Tests for telemetry_loop change detection."""
from gcs.backend.telemetry_loop import _has_changed


def _make_snap(**overrides):
    base = {
        "battery": 75.0,
        "mode": "AUTO",
        "armed": True,
        "lat": 32.0,
        "lon": 34.0,
        "alt": 100.0,
        "heading": 180.0,
        "ground_speed": 25.0,
        "link_ok": True,
        "companion_ok": True,
        "mission_progress": 0.5,
        "roll": 0.0,
        "pitch": 0.0,
        "gimbals": {},
    }
    base.update(overrides)
    return base


class TestHasChanged:
    def test_identical_snapshots_no_change(self):
        snap = _make_snap()
        assert _has_changed(snap, snap.copy()) is False

    def test_mode_change_detected(self):
        prev = _make_snap()
        curr = _make_snap(mode="GUIDED")
        assert _has_changed(prev, curr) is True

    def test_status_texts_forces_change(self):
        """status_texts in curr must always trigger a broadcast."""
        prev = _make_snap()
        curr = _make_snap(status_texts=[{"severity": 2, "text": "Engine fail"}])
        assert _has_changed(prev, curr) is True

    def test_status_texts_alone_forces_change(self):
        """Even with identical telemetry, status_texts triggers broadcast."""
        snap = _make_snap()
        curr = snap.copy()
        curr["status_texts"] = [{"severity": 0, "text": "CRITICAL"}]
        assert _has_changed(snap, curr) is True

    def test_empty_status_texts_no_false_positive(self):
        """Empty status_texts list should not force a broadcast."""
        prev = _make_snap()
        curr = _make_snap(status_texts=[])
        assert _has_changed(prev, curr) is False

    def test_lat_small_change_ignored(self):
        prev = _make_snap(lat=32.0000000)
        curr = _make_snap(lat=32.0000001)
        assert _has_changed(prev, curr) is False

    def test_lat_large_change_detected(self):
        prev = _make_snap(lat=32.000000)
        curr = _make_snap(lat=32.000010)
        assert _has_changed(prev, curr) is True

    def test_voltage_change_detected(self):
        prev = _make_snap(voltage=12.0)
        curr = _make_snap(voltage=11.0)
        assert _has_changed(prev, curr) is True

    def test_voltage_small_change_ignored(self):
        prev = _make_snap(voltage=12.0)
        curr = _make_snap(voltage=12.3)
        assert _has_changed(prev, curr) is False

    def test_current_change_detected(self):
        prev = _make_snap(current=5.0)
        curr = _make_snap(current=6.0)
        assert _has_changed(prev, curr) is True

    def test_voltage_none_to_value_detected(self):
        prev = _make_snap()
        curr = _make_snap(voltage=12.4)
        assert _has_changed(prev, curr) is True

    def test_gps_fix_change_detected(self):
        prev = _make_snap(gps_fix=2)
        curr = _make_snap(gps_fix=3)
        assert _has_changed(prev, curr) is True

    def test_gps_fix_none_to_value_detected(self):
        prev = _make_snap()
        curr = _make_snap(gps_fix=3)
        assert _has_changed(prev, curr) is True

    def test_gps_sats_change_detected(self):
        prev = _make_snap(gps_sats=8)
        curr = _make_snap(gps_sats=12)
        assert _has_changed(prev, curr) is True

    def test_gps_fix_identical_no_change(self):
        prev = _make_snap(gps_fix=3)
        curr = _make_snap(gps_fix=3)
        assert _has_changed(prev, curr) is False

    def test_prearm_ok_none_to_true_detected(self):
        prev = _make_snap()
        curr = _make_snap(prearm_ok=True)
        assert _has_changed(prev, curr) is True

    def test_prearm_ok_true_to_false_detected(self):
        prev = _make_snap(prearm_ok=True)
        curr = _make_snap(prearm_ok=False)
        assert _has_changed(prev, curr) is True

    def test_prearm_ok_identical_no_change(self):
        prev = _make_snap(prearm_ok=True)
        curr = _make_snap(prearm_ok=True)
        assert _has_changed(prev, curr) is False

    def test_prearm_check_state_none_to_value_detected(self):
        prev = _make_snap()
        curr = _make_snap(prearm_check_state="checks_disabled")
        assert _has_changed(prev, curr) is True

    def test_prearm_check_state_transition_detected(self):
        # prearm_ok stays None across no_sys_status -> checks_disabled, so the
        # state field is what carries the transition to the UI.
        prev = _make_snap(prearm_ok=None, prearm_check_state="no_sys_status")
        curr = _make_snap(prearm_ok=None, prearm_check_state="checks_disabled")
        assert _has_changed(prev, curr) is True

    def test_prearm_check_state_identical_no_change(self):
        prev = _make_snap(prearm_check_state="ok")
        curr = _make_snap(prearm_check_state="ok")
        assert _has_changed(prev, curr) is False

    def test_gimbal_q_change_detected(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [0, 1, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 101, "updated_at": 1001.0},
        })
        assert _has_changed(prev, curr) is True

    def test_gimbal_stale_change_detected(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": True,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        assert _has_changed(prev, curr) is True

    def test_gimbal_flags_change_detected(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 16,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        assert _has_changed(prev, curr) is True

    def test_gimbal_failure_flags_change_detected(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 1, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        assert _has_changed(prev, curr) is True

    def test_gimbal_fov_change_detected(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "fov_h_rad": 0.4, "fov_v_rad": 0.2,
                  "optics_stale": False},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "fov_h_rad": 0.2, "fov_v_rad": 0.1,
                  "optics_stale": False},
        })
        assert _has_changed(prev, curr) is True

    def test_gimbal_zoom_change_detected(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "fov_h_rad": 0.4, "fov_v_rad": 0.2,
                  "optics_stale": False, "zoom_level": 1.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "fov_h_rad": 0.4, "fov_v_rad": 0.2,
                  "optics_stale": False, "zoom_level": 2.0},
        })
        assert _has_changed(prev, curr) is True

    def test_gimbal_optics_timestamp_only_change_ignored(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "fov_h_rad": 0.4, "fov_v_rad": 0.2,
                  "optics_stale": False,
                  "optics_time_boot_ms": 100, "optics_updated_at": 1000.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "fov_h_rad": 0.4, "fov_v_rad": 0.2,
                  "optics_stale": False,
                  "optics_time_boot_ms": 101, "optics_updated_at": 1001.0},
        })
        assert _has_changed(prev, curr) is False

    def test_gimbal_timestamp_only_change_ignored(self):
        prev = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 101, "updated_at": 1001.0},
        })
        assert _has_changed(prev, curr) is False

    def test_gimbal_device_add_remove_detected(self):
        prev = _make_snap(gimbals={})
        curr = _make_snap(gimbals={
            "1": {"device_id": 1, "q": [1, 0, 0, 0], "flags": 32,
                  "failure_flags": 0, "stale": False,
                  "time_boot_ms": 100, "updated_at": 1000.0},
        })
        assert _has_changed(prev, curr) is True
