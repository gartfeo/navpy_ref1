"""Messages window history: source attribution, value changes, drain and backlog."""
import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

from fastapi import HTTPException

from gcs.backend.telemetry_loop import TelemetryLoop, _has_changed
from gcs.backend.vehicle_entry import VehicleEntry
from gcs.backend.vehicle_message_log import (
    VehicleMessageLog,
    source_label,
)


class _Msg(SimpleNamespace):
    """A MAVLink message with a source, like pymavlink's."""

    def __init__(self, src_system, src_component, **fields):
        super().__init__(**fields)
        self._src = (src_system, src_component)

    def get_srcSystem(self):
        return self._src[0]

    def get_srcComponent(self):
        return self._src[1]


def _text(src_system, src_component, text, severity=6):
    return _Msg(src_system, src_component, severity=severity, text=text)


def _value(src_system, src_component, name, value):
    return _Msg(src_system, src_component, name=name, value=value)


class TestAttribution(unittest.TestCase):
    def test_only_this_uavs_source_system_is_kept(self):
        log = VehicleMessageLog(2)
        log.on_statustext(_text(2, 1, "PreArm: Throttle below failsafe"))
        # Another UAV's companion text crossing over on the shared link.
        log.on_statustext(_text(3, 191, "NavPy: task accepted"))
        log.on_named_value(_value(3, 1, "NAVWD", 1.0))
        entries = log.backlog()["entries"]
        self.assertEqual([e["text"] for e in entries], ["PreArm: Throttle below failsafe"])
        self.assertEqual(log.named_values(), {})

    def test_component_names_the_sender(self):
        self.assertEqual(source_label(1), "autopilot")
        self.assertEqual(source_label(191), "companion")
        self.assertEqual(source_label(250), "comp 250")
        log = VehicleMessageLog(2)
        log.on_statustext(_text(2, 191, b"NavPy ready\x00\x00", severity=4))
        entry = log.backlog()["entries"][0]
        self.assertEqual((entry["source"], entry["label"], entry["text"]),
                         ("companion", "WARN", "NavPy ready"))


class TestNamedValues(unittest.TestCase):
    def test_history_records_changes_and_latest_keeps_arrival_time(self):
        log = VehicleMessageLog(2)
        with patch("gcs.backend.vehicle_message_log.rt.time", side_effect=[10.0, 11.0, 12.0]):
            log.on_named_value(_value(2, 1, b"NAVWD\x00\x00\x00\x00\x00", 1.0))
            log.on_named_value(_value(2, 1, "NAVWD", 1.0))
            log.on_named_value(_value(2, 1, "NAVWD", 3.0))
        texts = [e["text"] for e in log.backlog()["entries"]]
        self.assertEqual(texts, ["NAVWD = 1", "NAVWD = 3"])
        self.assertEqual(log.named_values(),
                         {"NAVWD": {"value": 3.0, "source": "autopilot", "ts": 12.0}})


class TestDrainAndBacklog(unittest.TestCase):
    def test_drain_returns_each_entry_once_and_backlog_keeps_them(self):
        log = VehicleMessageLog(2)
        log.on_statustext(_text(2, 1, "a"))
        self.assertEqual([e["text"] for e in log.drain_new()], ["a"])
        self.assertEqual(log.drain_new(), [])
        log.on_statustext(_text(2, 1, "b"))
        self.assertEqual([e["seq"] for e in log.drain_new()], [2])
        backlog = log.backlog()
        self.assertEqual([e["text"] for e in backlog["entries"]], ["a", "b"])
        self.assertEqual(backlog["last_seq"], 2)
        self.assertEqual([e["text"] for e in log.backlog(after=1)["entries"]], ["b"])

    def test_capacity_bounds_the_history(self):
        log = VehicleMessageLog(2, capacity=3)
        for i in range(5):
            log.on_statustext(_text(2, 1, str(i)))
        self.assertEqual([e["text"] for e in log.backlog()["entries"]], ["2", "3", "4"])


class TestVehicleEntryWiring(unittest.TestCase):
    def test_statustext_and_named_values_reach_the_log_and_snapshot(self):
        vehicle = MagicMock()
        handlers = {}
        vehicle.on_message.side_effect = lambda name, cb: handlers.setdefault(name, cb)
        entry = VehicleEntry(2, vehicle, "uav2")
        handlers["STATUSTEXT"](_text(2, 1, "Mission: 1 WP"))
        handlers["NAMED_VALUE_FLOAT"](_value(2, 1, "NAVWD", 1.0))
        handlers["NAMED_VALUE_INT"](_value(2, 191, "TASK", 4))
        texts = [e["text"] for e in entry.message_log.backlog()["entries"]]
        self.assertEqual(texts, ["Mission: 1 WP", "NAVWD = 1", "TASK = 4"])
        self.assertEqual(set(entry.snapshot()["named_values"]), {"NAVWD", "TASK"})


class TestBroadcast(unittest.TestCase):
    def test_messages_force_a_broadcast(self):
        self.assertTrue(_has_changed({"mode": "FBWA"}, {"mode": "FBWA", "messages": [{"seq": 1}]}))
        self.assertFalse(_has_changed({"mode": "FBWA"}, {"mode": "FBWA"}))

    def test_named_value_update_is_a_change(self):
        prev = {"named_values": {"NAVWD": {"value": 1.0, "source": "autopilot", "ts": 1.0}}}
        curr = {"named_values": {"NAVWD": {"value": 1.0, "source": "autopilot", "ts": 2.0}}}
        self.assertTrue(_has_changed(prev, curr))

    def test_new_entries_ride_the_unchanged_vehicles_next_tick(self):
        log = VehicleMessageLog(2)
        ticks = iter([["first"], [], ["second"]])

        def snapshots():
            for text in next(ticks):
                log.on_statustext(_text(2, 1, text))
            return [{"sys_id": 2, "lat": 40.0}]

        async def run():
            mgr = MagicMock()
            mgr.get_all_snapshots.side_effect = snapshots
            mgr.drain_position_path.return_value = []
            mgr.drain_messages.side_effect = lambda sid: log.drain_new()
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

        broadcast = asyncio.run(run())
        sent = [call.args[0]["vehicles"] for call in broadcast.call_args_list
                if call.args[0].get("type") == "telemetry"]
        # Tick 2 has nothing new and an unchanged snapshot: not broadcast.
        self.assertEqual([[[m["text"] for m in v.get("messages", [])] for v in vs] for vs in sent],
                         [[["first"]], [["second"]]])


class TestRoute(unittest.TestCase):
    def test_route_returns_backlog_or_404(self):
        from gcs.backend.routes import vehicles

        entry = SimpleNamespace(message_log=VehicleMessageLog(2))
        entry.message_log.on_statustext(_text(2, 1, "hello"))
        mgr = MagicMock()
        mgr.get_vehicle.side_effect = lambda sid: entry if sid == 2 else None
        with patch.object(vehicles, "vehicle_mgr", mgr):
            body = asyncio.run(vehicles.vehicle_messages(2))
            self.assertEqual([e["text"] for e in body["entries"]], ["hello"])
            with self.assertRaises(HTTPException) as caught:
                asyncio.run(vehicles.vehicle_messages(9))
        self.assertEqual(caught.exception.status_code, 404)


if __name__ == "__main__":
    unittest.main()
