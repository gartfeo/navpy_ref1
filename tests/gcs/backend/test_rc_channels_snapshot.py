"""Tests for RC_CHANNELS capture in VehicleEntry and live-change detection."""
import unittest
from unittest.mock import MagicMock

from gcs.backend.vehicle_manager import VehicleEntry
from gcs.backend.telemetry_loop import _rc_channels_changed, _has_changed


def _make_mock_vehicle():
    """Mock VehicleMav that records on_message callbacks (like test_statustext)."""
    v = MagicMock()
    v.location.return_value = None
    v.get_mode = None
    v.attitude = None
    v.is_armed = False
    v.heading = 0
    v.air_speed = 0
    v.ground_speed = 0
    v.link_ok = True
    v.mission_items_next = 0
    v.mission_items_count = 0
    v._cbs = {}

    def _on_message(name, cb):
        v._cbs.setdefault(name, []).append(cb)

    v.on_message = _on_message
    return v


def _rc_msg(values, chancount=None):
    """Build a fake RC_CHANNELS message from a list of PWM values."""
    msg = MagicMock()
    msg.chancount = len(values) if chancount is None else chancount
    for i, val in enumerate(values, start=1):
        setattr(msg, f"chan{i}_raw", val)
    return msg


class TestRcChannelsSnapshot(unittest.TestCase):
    def test_rc_callback_registered(self):
        v = _make_mock_vehicle()
        VehicleEntry(1, v, "uav1")
        self.assertIn("RC_CHANNELS", v._cbs)
        self.assertEqual(len(v._cbs["RC_CHANNELS"]), 1)

    def test_no_rc_no_key(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        snap = entry.snapshot()
        self.assertNotIn("rc_channels", snap)

    def test_rc_appears_in_snapshot(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["RC_CHANNELS"][0]
        cb(_rc_msg([1500, 1490, 1100, 1900]))
        snap = entry.snapshot()
        self.assertIn("rc_channels", snap)
        self.assertEqual(snap["rc_channels"]["count"], 4)
        self.assertEqual(snap["rc_channels"]["channels"], [1500, 1490, 1100, 1900])

    def test_uint16_max_becomes_none(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["RC_CHANNELS"][0]
        cb(_rc_msg([1500, 65535, 1100]))
        snap = entry.snapshot()
        self.assertEqual(snap["rc_channels"]["channels"], [1500, None, 1100])

    def test_capped_at_16_channels(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["RC_CHANNELS"][0]
        # 18 channels reported, but only 16 map to writable RCn_* params.
        cb(_rc_msg([1500] * 18, chancount=18))
        snap = entry.snapshot()
        self.assertEqual(snap["rc_channels"]["count"], 16)

    def test_latest_message_wins(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["RC_CHANNELS"][0]
        cb(_rc_msg([1500, 1500]))
        cb(_rc_msg([1000, 2000]))
        snap = entry.snapshot()
        self.assertEqual(snap["rc_channels"]["channels"], [1000, 2000])


class TestRcChangeDetection(unittest.TestCase):
    def test_none_to_none_no_change(self):
        self.assertFalse(_rc_channels_changed(None, None))

    def test_appearance_is_change(self):
        self.assertTrue(_rc_channels_changed(None, {"channels": [1500], "count": 1}))

    def test_small_jitter_ignored(self):
        prev = {"channels": [1500, 1500], "count": 2}
        curr = {"channels": [1501, 1499], "count": 2}  # within ±2 µs
        self.assertFalse(_rc_channels_changed(prev, curr))

    def test_real_movement_detected(self):
        prev = {"channels": [1500, 1500], "count": 2}
        curr = {"channels": [1500, 1900], "count": 2}
        self.assertTrue(_rc_channels_changed(prev, curr))

    def test_count_change_detected(self):
        prev = {"channels": [1500], "count": 1}
        curr = {"channels": [1500, 1500], "count": 2}
        self.assertTrue(_rc_channels_changed(prev, curr))

    def test_has_changed_picks_up_rc(self):
        prev = {"rc_channels": {"channels": [1500], "count": 1}}
        curr = {"rc_channels": {"channels": [1900], "count": 1}}
        self.assertTrue(_has_changed(prev, curr))


if __name__ == "__main__":
    unittest.main()
