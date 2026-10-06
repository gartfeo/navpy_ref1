"""Mission-download progress: [current, total] waypoint counts reach the UI.

Previously a download showed a static "Reading mission…" with no numbers, even
though the MAVLink download protocol knows the total count (MISSION_COUNT) and
each item's sequence number as it arrives. This threads an on_progress callback
from the protocol layer (mav_mission.py) through VehicleMav.download_mission,
mission_validator.validate_vehicle_mission, and VehicleManager.probe_existing_mission
onto VehicleEntry.mission_download_progress, exposed in the telemetry snapshot.
"""
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from gcs.backend.mission_validator import MissionProbe
from gcs.backend.vehicle_manager import VehicleEntry, VehicleManager


class TestProbeExistingMissionProgress(unittest.TestCase):
    def setUp(self):
        self.mgr = VehicleManager()
        # Register a bare entry directly (no add_vehicle) so the async
        # connect-time probe thread never runs and can't race this test's
        # explicit, synchronous probe_existing_mission call.
        self.entry = VehicleEntry(sys_id=6, vehicle=MagicMock(), name="u6")
        self.mgr._vehicles[6] = self.entry

    def tearDown(self):
        try:
            self.mgr.shutdown()
        except Exception:
            pass

    def test_progress_updates_during_download_and_clears_after(self):
        captured = []

        def fake_validate(vehicle, sys_id, on_progress=None):
            for i in range(1, 4):
                if on_progress:
                    on_progress(i, 3)
                captured.append(
                    list(self.entry.mission_download_progress)
                    if self.entry.mission_download_progress else None
                )
            return (
                MissionProbe(valid=True, item_count=3, track_count=3, search_pattern="distributed", polygon=[]),
                {"waypoints": [{}, {}, {}]},
            )

        self.assertIsNone(self.entry.mission_download_progress)
        with patch("gcs.backend.mission_validator.validate_vehicle_mission", side_effect=fake_validate):
            probe = self.mgr.probe_existing_mission(6)

        self.assertTrue(probe.valid)
        self.assertEqual(captured, [[1, 3], [2, 3], [3, 3]])
        # Cleared once the download settles — a ready card must not show a
        # stale progress count.
        self.assertIsNone(self.entry.mission_download_progress)

    def test_clear_does_not_wipe_a_concurrent_writers_value(self):
        """Our finally must only null OUR own progress. If a concurrent download
        of the same vehicle (e.g. the /mission GET) takes over the field after our
        last item, our clear must leave its live count alone — not flicker it to
        None."""
        other = [2, 9]  # a value "another writer" published

        def fake_validate(vehicle, sys_id, on_progress=None):
            on_progress(1, 3)  # we publish our progress
            self.entry.mission_download_progress = other  # someone else takes over
            return (
                MissionProbe(valid=True, item_count=3, track_count=3, search_pattern="distributed", polygon=[]),
                {"waypoints": [{}, {}, {}]},
            )

        with patch("gcs.backend.mission_validator.validate_vehicle_mission", side_effect=fake_validate):
            self.mgr.probe_existing_mission(6)

        # The field is not ours anymore, so it was left untouched.
        self.assertEqual(self.entry.mission_download_progress, other)

    def test_progress_cleared_even_on_failed_probe(self):
        def fake_validate(vehicle, sys_id, on_progress=None):
            if on_progress:
                on_progress(1, 5)
            raise RuntimeError("download boom")

        with patch("gcs.backend.mission_validator.validate_vehicle_mission", side_effect=fake_validate):
            with self.assertRaises(RuntimeError):
                self.mgr.probe_existing_mission(6)

        self.assertIsNone(self.entry.mission_download_progress)

    def test_exposed_in_snapshot(self):
        snap = self.entry.snapshot()
        self.assertIn("mission_download_progress", snap)
        self.assertIsNone(snap["mission_download_progress"])
        self.entry.mission_download_progress = [2, 10]
        self.assertEqual(self.entry.snapshot()["mission_download_progress"], [2, 10])


class TestMavMissionDownloadProgress(unittest.TestCase):
    """The protocol-layer on_progress callback (mirrors upload's on_wp_sent)."""

    def test_download_invokes_on_progress_per_item(self):
        from navpy.modules.vehicle.mission_download import MissionDownloader

        transport = MagicMock()
        store = MagicMock()
        messages = MagicMock()
        logger_ref = MagicMock()
        logger_ref.value = MagicMock()
        messages.cursor.return_value = 0
        count_msg = MagicMock(count=2)
        item_msgs = [MagicMock(seq=0), MagicMock(seq=1)]
        messages.wait_after.side_effect = [
            SimpleNamespace(message=count_msg),
            *(SimpleNamespace(message=item) for item in item_msgs),
        ]
        downloader = MissionDownloader(
            6,
            transport,
            store,
            messages,
            logger_ref,
        )
        progress_calls = []
        result = downloader.download(
            timeout=1.0,
            retries=1,
            on_progress=lambda current, total: progress_calls.append(
                (current, total)
            ),
        )

        self.assertEqual(result, 2)
        self.assertEqual(progress_calls, [(1, 2), (2, 2)])
        store.replace_items.assert_called_once_with(item_msgs)


if __name__ == "__main__":
    unittest.main()
