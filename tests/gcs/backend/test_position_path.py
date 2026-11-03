"""Every position fix between two broadcasts reaches the map trail as ``path``."""
import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend.position_path import PositionPath
from gcs.backend.telemetry_loop import TelemetryLoop
from gcs.backend.vehicle_manager import VehicleEntry


def _fix(lat, lon, alt_m=1300.0, rel_m=20.0):
    return SimpleNamespace(lat=round(lat * 1e7), lon=round(lon * 1e7),
                           alt=round(alt_m * 1e3), relative_alt=round(rel_m * 1e3))


def _sourced_fix(lat, src_system, src_component=1):
    msg = _fix(lat, 44.0)
    msg.get_srcSystem = lambda: src_system
    msg.get_srcComponent = lambda: src_component
    return msg


class TestPositionPath(unittest.TestCase):
    def test_drain_returns_every_fix_once_oldest_first(self):
        path = PositionPath(2)
        path.on_global_position(_fix(40.1, 44.1, rel_m=10.0))
        path.on_global_position(_fix(40.2, 44.2, rel_m=12.5))
        self.assertEqual(path.drain(), [
            {"lat": 40.1, "lon": 44.1, "alt": 1300.0, "alt_rel": 10.0},
            {"lat": 40.2, "lon": 44.2, "alt": 1300.0, "alt_rel": 12.5},
        ])
        self.assertEqual(path.drain(), [])

    def test_undrained_buffer_keeps_the_newest_fixes(self):
        path = PositionPath(2, max_fixes=2)
        for i in range(3):
            path.on_global_position(_fix(40.0 + i, 44.0))
        self.assertEqual([f["lat"] for f in path.drain()], [41.0, 42.0])

    def test_only_this_vehicles_own_autopilot_is_kept(self):
        path = PositionPath(2)
        path.on_global_position(_sourced_fix(40.1, 2))
        path.on_global_position(_sourced_fix(49.0, 3))  # another UAV on the shared link
        path.on_global_position(_sourced_fix(49.1, 2, src_component=191))  # its companion
        path.on_global_position(_sourced_fix(40.2, 2))
        self.assertEqual([f["lat"] for f in path.drain()], [40.1, 40.2])

    def test_vehicle_entry_records_its_global_position_messages(self):
        vehicle = MagicMock()
        callbacks = {}
        vehicle.on_message = lambda name, cb: callbacks.setdefault(name, []).append(cb)
        entry = VehicleEntry(2, vehicle, "s1-u2")
        for cb in callbacks["GLOBAL_POSITION_INT"]:
            cb(_fix(40.3, 44.4))
        self.assertEqual([f["lat"] for f in entry.position_path.drain()], [40.3])


class TestTelemetryLoopPath(unittest.TestCase):
    def test_fixes_are_sent_with_the_vehicles_next_broadcast(self):
        real = PositionPath(2)
        snaps = iter([
            [{"sys_id": 2, "lat": 40.0}],
            # Unchanged (an out-and-back): not broadcast, its fixes are kept.
            [{"sys_id": 2, "lat": 40.0}],
            [{"sys_id": 2, "lat": 40.1}],
        ])
        fixes_before_tick = iter([[39.9, 40.0], [40.05, 40.0], [40.1]])

        def next_snapshots():
            for lat in next(fixes_before_tick):
                real.on_global_position(_fix(lat, 44.0))
            return next(snaps)

        async def run_ticks():
            mgr = MagicMock()
            mgr.get_all_snapshots.side_effect = next_snapshots
            mgr.drain_position_path.side_effect = lambda sid: real.drain()
            mgr.get_seen_ids.return_value = []
            loop = TelemetryLoop(mgr)
            settings = SimpleNamespace(simulation=SimpleNamespace(sim_mode=False),
                                       connection=SimpleNamespace(ws_broadcast_interval_s=0.0))
            left = iter(range(2))

            async def stop_at_the_last(_interval):
                if next(left, None) is None:
                    loop._stop.set()

            with patch("gcs.backend.telemetry_loop.settings_store.get", return_value=settings), \
                 patch("gcs.backend.telemetry_loop.ws_manager.broadcast",
                       new_callable=AsyncMock) as broadcast, \
                 patch("gcs.backend.telemetry_loop.emit"), \
                 patch("gcs.backend.telemetry_loop.asyncio.sleep", new=stop_at_the_last):
                await loop._loop()
            return broadcast

        broadcast = asyncio.run(run_ticks())
        sent = [call.args[0]["vehicles"] for call in broadcast.call_args_list
                if call.args[0].get("type") == "telemetry"]
        self.assertEqual([[[f["lat"] for f in v["path"]] for v in vehicles] for vehicles in sent], [
            [[39.9, 40.0]],
            [[40.05, 40.0, 40.1]],
        ])


class TestInitialState(unittest.TestCase):
    def test_a_new_subscriber_gets_empty_paths_and_nothing_is_drained(self):
        from gcs.backend.routes import telemetry

        ws = SimpleNamespace(send_bytes=AsyncMock())
        mgr = MagicMock()
        mgr.get_all_snapshots.return_value = [{"sys_id": 2, "lat": 40.0}]
        with patch.object(telemetry, "vehicle_mgr", mgr):
            asyncio.run(telemetry.send_initial_state(ws))
        import orjson
        sent = orjson.loads(ws.send_bytes.call_args.args[0])
        self.assertEqual(sent["vehicles"], [{"sys_id": 2, "lat": 40.0, "path": []}])
        mgr.drain_position_path.assert_not_called()


if __name__ == "__main__":
    unittest.main()
