"""Tests for compass-cal telemetry payload builders (compass_cal_listener)."""
import unittest
from unittest.mock import MagicMock

from gcs.backend.compass_cal_listener import _progress_payload, _report_payload


class TestProgressPayload(unittest.TestCase):
    def test_progress_fields(self):
        msg = MagicMock(compass_id=1, completion_pct=42, cal_status=2)
        payload = _progress_payload(7, msg)
        self.assertEqual(payload, {
            "type": "compass_cal_progress",
            "sys_id": 7,
            "compass_id": 1,
            "pct": 42,
            "cal_status": 2,
            "report": False,
        })


class TestReportPayload(unittest.TestCase):
    def test_report_pins_pct_to_100_and_carries_result(self):
        msg = MagicMock(compass_id=0, cal_status=4, fitness=3.5, autosaved=1)
        payload = _report_payload(2, msg)
        self.assertEqual(payload["type"], "compass_cal_progress")
        self.assertEqual(payload["sys_id"], 2)
        self.assertEqual(payload["compass_id"], 0)
        self.assertEqual(payload["pct"], 100)
        self.assertEqual(payload["cal_status"], 4)
        self.assertTrue(payload["report"])
        self.assertEqual(payload["fitness"], 3.5)
        self.assertEqual(payload["autosaved"], 1)


if __name__ == "__main__":
    unittest.main()
