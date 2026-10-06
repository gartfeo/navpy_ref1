"""Backend startup auto-connect: in launcher-managed sim mode, the backend
connects to THIS chat's running SITL (its monitor UDP port) on startup, so
vehicles appear without a browser."""
import asyncio
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend import main as backend_main
from gcs.backend.settings_model import GcsSettings


class _Entry:
    def __init__(self, sys_id):
        self.sys_id = sys_id


def _settings(*, sim_mode: bool, auto_connect: bool) -> GcsSettings:
    """A settings snapshot built from the REAL model, not a MagicMock.

    Both gates are keyword-required because production reads them in ONE
    condition (``main.py``: ``not sim_mode or not auto_connect``), so a test that
    leaves either implicit cannot say which one it is exercising. A MagicMock
    fabricates both as truthy children, which would hide production dropping or
    renaming either read; the real model raises on a renamed field.
    """
    settings = GcsSettings()
    settings.simulation.sim_mode = sim_mode
    settings.connection.auto_connect = auto_connect
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
            asyncio.run(backend_main._startup_autoconnect())
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
            asyncio.run(backend_main._startup_autoconnect())
        self.assertEqual(disc.call_count, 3)
        self.assertEqual({c.args[0] for c in auto.call_args_list}, {1, 2, 3})

    def test_noop_without_chat_index(self):
        disc = MagicMock(return_value=[])
        with patch("gcs.backend.instance_ports.chat_index", return_value=None), \
             patch.object(backend_main.vehicle_mgr, "discover_and_connect", disc):
            asyncio.run(backend_main._startup_autoconnect())
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
            asyncio.run(backend_main._startup_autoconnect())
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
            asyncio.run(backend_main._startup_autoconnect())
        disc.assert_not_called()
        auto.assert_not_called()
        # Returned before the retry loop, not merely idled through it.
        sleep.assert_not_awaited()


if __name__ == "__main__":
    unittest.main()
