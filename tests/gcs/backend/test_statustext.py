"""Tests for STATUSTEXT collection in VehicleEntry."""
import unittest
from unittest.mock import MagicMock

from gcs.backend.vehicle_manager import VehicleEntry


def _make_mock_vehicle():
    """Create a mock VehicleMav with on_message support."""
    v = MagicMock()
    v.location.return_value = None
    v.get_mode = None
    v.attitude = None
    v.is_armed = False
    v.battery_level = 80
    v.heading = 0
    v.air_speed = 0
    v.ground_speed = 0
    v.link_ok = True
    v.mission_items_next = 0
    v.mission_items_count = 0
    # Track callbacks registered via on_message
    v._cbs = {}
    def _on_message(name, cb):
        v._cbs.setdefault(name, []).append(cb)
    v.on_message = _on_message
    return v


def _make_statustext_msg(text, severity=6):
    msg = MagicMock()
    msg.text = text
    msg.severity = severity
    return msg


class TestStatusTextCollection(unittest.TestCase):
    def test_statustext_callback_registered(self):
        v = _make_mock_vehicle()
        VehicleEntry(1, v, "uav1")
        self.assertIn("STATUSTEXT", v._cbs)
        self.assertEqual(len(v._cbs["STATUSTEXT"]), 1)

    def test_statustext_appears_in_snapshot(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        # Simulate a STATUSTEXT arriving
        cb = v._cbs["STATUSTEXT"][0]
        cb(_make_statustext_msg("Low battery", severity=4))
        snap = entry.snapshot()
        self.assertIn("status_texts", snap)
        self.assertEqual(len(snap["status_texts"]), 1)
        self.assertEqual(snap["status_texts"][0]["text"], "Low battery")
        self.assertEqual(snap["status_texts"][0]["label"], "WARN")
        self.assertEqual(snap["status_texts"][0]["severity"], 4)

    def test_snapshot_drains_queue(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["STATUSTEXT"][0]
        cb(_make_statustext_msg("msg1"))
        cb(_make_statustext_msg("msg2"))
        snap1 = entry.snapshot()
        self.assertEqual(len(snap1["status_texts"]), 2)
        # Second snapshot should have no status_texts
        snap2 = entry.snapshot()
        self.assertNotIn("status_texts", snap2)

    def test_no_statustext_no_key(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        snap = entry.snapshot()
        self.assertNotIn("status_texts", snap)

    def test_severity_labels(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["STATUSTEXT"][0]
        for sev, label in [(0, "EMERG"), (3, "ERR"), (4, "WARN"), (6, "INFO")]:
            cb(_make_statustext_msg(f"sev{sev}", severity=sev))
        snap = entry.snapshot()
        labels = [t["label"] for t in snap["status_texts"]]
        self.assertEqual(labels, ["EMERG", "ERR", "WARN", "INFO"])

    def test_null_terminated_text_stripped(self):
        v = _make_mock_vehicle()
        entry = VehicleEntry(1, v, "uav1")
        cb = v._cbs["STATUSTEXT"][0]
        cb(_make_statustext_msg("hello\x00\x00\x00"))
        snap = entry.snapshot()
        self.assertEqual(snap["status_texts"][0]["text"], "hello")
