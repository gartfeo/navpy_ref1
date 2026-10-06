"""The mission gateway serializes all table transactions."""
from __future__ import annotations

from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import MagicMock

from navpy.modules.vehicle.mav_mission import MavMission


def _mission(downloader, uploader, store=None):
    return MavMission(
        target_system=7,
        transport=MagicMock(),
        logger_ref=SimpleNamespace(value=MagicMock()),
        message_store=MagicMock(),
        inbox=MagicMock(),
        store=store or MagicMock(),
        downloader=downloader,
        uploader=uploader,
    )


class TestMissionLock(unittest.TestCase):
    def test_status_reads_do_not_wait_for_active_download(self):
        entered = threading.Event()
        release = threading.Event()
        finished = threading.Event()
        store = MagicMock()
        store.count = 7
        store.next_sequence = 3

        def blocked_download(*_args, **_kwargs):
            entered.set()
            release.wait(timeout=2.0)
            return 7

        downloader = MagicMock()
        downloader.download.side_effect = blocked_download
        mission = _mission(downloader, MagicMock(), store)
        transfer = threading.Thread(target=mission.download)
        result = []

        def read_status():
            result.append((mission.next_seq, mission.count))
            finished.set()

        reader = threading.Thread(target=read_status)
        try:
            transfer.start()
            self.assertTrue(entered.wait(timeout=1.0))
            reader.start()
            self.assertTrue(
                finished.wait(timeout=1.0),
                "mission status read blocked behind the wire transaction",
            )
            self.assertEqual(result, [(3, 7)])
        finally:
            release.set()
            transfer.join(timeout=2.0)
            reader.join(timeout=2.0)

    def test_concurrent_downloads_do_not_overlap(self):
        state = {"active": 0, "overlap": False, "runs": 0}
        guard = threading.Lock()

        def busy_download(*_args, **_kwargs):
            with guard:
                state["active"] += 1
                state["runs"] += 1
                state["overlap"] |= state["active"] > 1
            time.sleep(0.02)
            with guard:
                state["active"] -= 1
            return 0

        downloader = MagicMock()
        downloader.download.side_effect = busy_download
        mission = _mission(downloader, MagicMock())
        threads = [threading.Thread(target=mission.download) for _ in range(5)]

        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)

        self.assertEqual(state["runs"], 5)
        self.assertFalse(state["overlap"])
        self.assertTrue(all(not thread.is_alive() for thread in threads))

    def test_download_and_upload_do_not_overlap(self):
        state = {"active": 0, "overlap": False}
        guard = threading.Lock()

        def busy(*_args, **_kwargs):
            with guard:
                state["active"] += 1
                state["overlap"] |= state["active"] > 1
            time.sleep(0.02)
            with guard:
                state["active"] -= 1
            return 0

        downloader = MagicMock()
        downloader.download.side_effect = busy
        uploader = MagicMock()
        uploader.upload.side_effect = busy
        mission = _mission(downloader, uploader)
        threads = [
            threading.Thread(target=mission.download),
            threading.Thread(target=mission.upload),
            threading.Thread(target=mission.download),
        ]

        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)

        self.assertFalse(state["overlap"])
        self.assertTrue(all(not thread.is_alive() for thread in threads))


if __name__ == "__main__":
    unittest.main()
