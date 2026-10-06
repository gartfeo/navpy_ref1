"""An unrelated invalid vision profile must not reset the telemetry interval."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend.settings_model import GcsSettings
from gcs.backend.telemetry_loop import TelemetryLoop


def test_telemetry_interval_does_not_read_vision_profiles():
    async def run_once():
        settings = GcsSettings()
        settings.simulation.sim_mode = False
        settings.connection.ws_broadcast_interval_s = 0.37
        settings.camera.vision_profile = "fixture"
        manager = MagicMock()
        manager.get_all_snapshots.return_value = []
        manager.get_seen_ids.return_value = []
        loop = TelemetryLoop(manager)
        intervals = []

        async def stop_after_sleep(interval):
            intervals.append(interval)
            loop._stop.set()

        with patch("gcs.backend.telemetry_loop.settings_store.get", return_value=settings), \
             patch("navpy.modules.vision.vision_profiles.load_profiles", side_effect=ValueError("invalid profile")) as profiles, \
             patch("gcs.backend.telemetry_loop.ws_manager.broadcast", new_callable=AsyncMock), \
             patch("gcs.backend.telemetry_loop.asyncio.sleep", side_effect=stop_after_sleep):
            await loop._loop()
        assert intervals == [0.37]
        profiles.assert_not_called()

    asyncio.run(run_once())
