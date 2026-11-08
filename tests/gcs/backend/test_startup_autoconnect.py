"""Backend startup auto-connect: in launcher-managed sim mode, the backend
connects to THIS chat's running SITL (its monitor UDP port) on startup, so
vehicles appear without a browser."""
import asyncio
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend import startup_autoconnect as backend_main
from gcs.backend.routes import control as ctrl_mod
from gcs.backend.settings_model import GcsSettings


class _Entry:
    def __init__(self, sys_id):
        self.sys_id = sys_id


def _settings(*, sim_mode: bool, auto_connect: bool, launch_type: str = "bungee") -> GcsSettings:
    """A settings snapshot built from the REAL model, not a MagicMock.

    Both gates are keyword-required because production reads them in ONE
    condition (``startup_autoconnect.py``: ``not sim_mode or not auto_connect``), so a test that
    leaves either implicit cannot say which one it is exercising. A MagicMock
    fabricates both as truthy children, which would hide production dropping or
    renaming either read; the real model raises on a renamed field.
    """
    settings = GcsSettings()
    settings.simulation.sim_mode = sim_mode
    settings.connection.auto_connect = auto_connect
    settings.launch.launch_type = launch_type
    return settings


class TestStartupAutoconnect(unittest.TestCase):
    def test_connects_on_chat_monitor_port(self):
        disc = MagicMock(return_value=[_Entry(16), _Entry(17), _Entry(18)])
        auto = MagicMock()
        with patch("gcs.backend.instance_ports.chat_index", return_value=5), \
             patch("gcs.backend.settings_store.settings_store.get",
                   return_value=_settings(sim_mode=True, auto_connect=True)), \
             patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc), \
             patch("gcs.backend.routes.navpy_sim.auto_start_navpy_sim", auto):
            asyncio.run(backend_main.startup_autoconnect())
        # Discovered on the chat's monitor UDP port = the running SITL's port.
        self.assertEqual(disc.call_args.args[0], "udp:0.0.0.0:15555")
        # NavPy auto-started for each discovered vehicle.
        self.assertEqual({c.args[0] for c in auto.call_args_list}, {16, 17, 18})

    def test_retries_until_all_demo_uavs_are_connected(self):
        disc = MagicMock(side_effect=[
            [_Entry(1)],
            [_Entry(2)],
            [_Entry(3)],
        ])
        auto = MagicMock()
        with patch("gcs.backend.instance_ports.chat_index", return_value=0), \
             patch("gcs.backend.settings_store.settings_store.get",
                   return_value=_settings(sim_mode=True, auto_connect=True)), \
             patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc), \
             patch("gcs.backend.routes.navpy_sim.auto_start_navpy_sim", auto), \
             patch.object(backend_main.asyncio, "sleep", new_callable=AsyncMock):
            asyncio.run(backend_main.startup_autoconnect())
        self.assertEqual(disc.call_count, 3)
        self.assertEqual({c.args[0] for c in auto.call_args_list}, {1, 2, 3})

    def test_sim_mode_params_checked_once_per_vehicle_before_navpy(self):
        """Switch position 6 and the start-up mode go to AUTO before NavPy
        starts (2026-11-05 AUTOTUNE), once per vehicle across retries."""
        order = []
        vehicles = {sid: MagicMock(name=f"vehicle{sid}") for sid in (1, 2, 3)}
        entry_for = lambda sid: MagicMock(vehicle=vehicles[sid]) if sid in vehicles else None
        ensure = MagicMock(side_effect=lambda vehicle, sid: order.append(("params", sid)) or [])
        auto = MagicMock(side_effect=lambda sid: order.append(("navpy", sid)) or True)
        disc = MagicMock(side_effect=[[_Entry(1), _Entry(2)], [_Entry(2), _Entry(3)]])
        with patch("gcs.backend.instance_ports.chat_index", return_value=0),              patch("gcs.backend.settings_store.settings_store.get",
                   return_value=_settings(sim_mode=True, auto_connect=True)),              patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc),              patch.object(backend_main.vehicle_mgr, "get_vehicle", side_effect=entry_for),              patch.object(backend_main, "ensure_sim_mode_params", ensure),              patch("gcs.backend.routes.navpy_sim.auto_start_navpy_sim", auto),              patch.object(backend_main.asyncio, "sleep", new_callable=AsyncMock):
            asyncio.run(backend_main.startup_autoconnect())
        self.assertEqual([c.args for c in ensure.call_args_list],
                         [(vehicles[1], 1), (vehicles[2], 2), (vehicles[3], 3)])
        for sid in (1, 2, 3):
            self.assertLess(order.index(("params", sid)), order.index(("navpy", sid)))

    def test_sim_mode_params_error_does_not_stop_navpy_start(self):
        auto = MagicMock(return_value=True)
        with patch("gcs.backend.instance_ports.chat_index", return_value=0),              patch("gcs.backend.settings_store.settings_store.get",
                   return_value=_settings(sim_mode=True, auto_connect=True)),              patch.object(backend_main.vehicle_mgr, "discover_and_connect",
                          MagicMock(return_value=[_Entry(1), _Entry(2), _Entry(3)])),              patch.object(backend_main.vehicle_mgr, "get_vehicle", return_value=MagicMock()),              patch.object(backend_main, "ensure_sim_mode_params",
                          MagicMock(side_effect=RuntimeError("link"))),              patch("gcs.backend.routes.navpy_sim.auto_start_navpy_sim", auto):
            asyncio.run(backend_main.startup_autoconnect())
        self.assertEqual({c.args[0] for c in auto.call_args_list}, {1, 2, 3})

    def test_noop_without_chat_index(self):
        disc = MagicMock(return_value=[])
        with patch("gcs.backend.instance_ports.chat_index", return_value=None), \
             patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc):
            asyncio.run(backend_main.startup_autoconnect())
        disc.assert_not_called()

    def test_noop_when_sim_mode_off(self):
        # auto_connect is ON, so sim_mode is the only thing that can stop it.
        # asyncio.sleep is faked AND asserted: faked so a broken gate fails in
        # milliseconds instead of sitting through the whole retry budget, asserted
        # so the fake proves the early return rather than hiding a loop that runs.
        disc = MagicMock(return_value=[])
        with patch("gcs.backend.instance_ports.chat_index", return_value=5), \
             patch("gcs.backend.settings_store.settings_store.get",
                   return_value=_settings(sim_mode=False, auto_connect=True)), \
             patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc), \
             patch.object(backend_main.asyncio, "sleep",
                          new_callable=AsyncMock) as sleep:
            asyncio.run(backend_main.startup_autoconnect())
        disc.assert_not_called()
        sleep.assert_not_awaited()

    def test_noop_when_auto_connect_off(self):
        # The operator turned auto-connect off. Sim mode and a chat index are both
        # present, so the only thing that can stop the scan is that setting — and
        # nothing must connect behind their back on startup.
        disc = MagicMock(return_value=[])
        auto = MagicMock()
        with patch("gcs.backend.instance_ports.chat_index", return_value=5), \
             patch("gcs.backend.settings_store.settings_store.get",
                   return_value=_settings(sim_mode=True, auto_connect=False)), \
             patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc), \
             patch("gcs.backend.routes.navpy_sim.auto_start_navpy_sim", auto), \
             patch.object(backend_main.asyncio, "sleep",
                          new_callable=AsyncMock) as sleep:
            asyncio.run(backend_main.startup_autoconnect())
        disc.assert_not_called()
        auto.assert_not_called()
        # Returned before the retry loop, not merely idled through it.
        sleep.assert_not_awaited()


class TestStartupStartsEsp32Sim(unittest.TestCase):
    """Startup auto-connect starts the ESP32 sim as the connect route does, so a
    hand demo's Launch is enabled with no manual connect (todo 164)."""

    def setUp(self):
        ctrl_mod._esp32_sim = None

    def tearDown(self):
        ctrl_mod._esp32_sim = None

    def _run(self, settings, disc, sim_cls, auto=None):
        with patch("gcs.backend.instance_ports.chat_index", return_value=0),              patch("gcs.backend.settings_store.settings_store.get", return_value=settings),              patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc),              patch("gcs.backend.routes.navpy_sim.auto_start_navpy_sim", auto or MagicMock()),              patch("gcs.backend.esp32_simulator.Esp32Simulator", sim_cls),              patch.object(backend_main.asyncio, "sleep", new_callable=AsyncMock):
            asyncio.run(backend_main.startup_autoconnect())

    def test_starts_once_before_the_first_scan(self):
        order = MagicMock()
        disc = MagicMock(side_effect=[[_Entry(1)], [_Entry(2)], [_Entry(3)]])
        order.attach_mock(disc, "scan")
        order.attach_mock(MagicMock(), "sim_cls")
        sim_cls = order.sim_cls
        self._run(_settings(sim_mode=True, auto_connect=True, launch_type="container"), disc, sim_cls)
        sim_cls.assert_called_once()
        sim_cls.return_value.start.assert_called_once()
        self.assertIs(ctrl_mod._esp32_sim, sim_cls.return_value)
        # Started before the scan loop, not after a retry finds the vehicles.
        names = [c[0] for c in order.mock_calls]
        self.assertLess(names.index("sim_cls().start"), names.index("scan"))
        self.assertEqual(disc.call_count, 3)

    def test_start_failure_does_not_stop_the_connect(self):
        disc = MagicMock(return_value=[_Entry(1), _Entry(2), _Entry(3)])
        sim_cls = MagicMock()
        sim_cls.return_value.start.side_effect = OSError("address in use")
        auto = MagicMock()
        self._run(_settings(sim_mode=True, auto_connect=True, launch_type="container"), disc, sim_cls, auto)
        self.assertIsNone(ctrl_mod._esp32_sim)
        disc.assert_called_once()
        self.assertEqual({c.args[0] for c in auto.call_args_list}, {1, 2, 3})

    def test_not_started_when_auto_connect_off(self):
        disc = MagicMock(return_value=[])
        sim_cls = MagicMock()
        self._run(_settings(sim_mode=True, auto_connect=False, launch_type="container"), disc, sim_cls)
        sim_cls.assert_not_called()
        self.assertIsNone(ctrl_mod._esp32_sim)

    def test_not_started_when_sim_mode_off(self):
        disc = MagicMock(return_value=[])
        sim_cls = MagicMock()
        self._run(_settings(sim_mode=False, auto_connect=True, launch_type="container"), disc, sim_cls)
        sim_cls.assert_not_called()


if __name__ == "__main__":
    unittest.main()
