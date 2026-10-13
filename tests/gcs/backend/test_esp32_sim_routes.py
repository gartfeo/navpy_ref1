"""Tests for the ESP32 simulator start/stop route helpers."""
import os
import unittest
from unittest.mock import patch, MagicMock

from gcs.backend import instance_ports as ip
from gcs.backend.esp32_simulator import Esp32Simulator
from gcs.backend.routes import control as ctrl_mod
from gcs.backend.settings_model import GcsSettings


def _settings(*, sim_mode: bool, launch_type: str, port: int = 80) -> GcsSettings:
    """A settings snapshot built from the REAL model, not a MagicMock.

    ``launch_type`` is REQUIRED, deliberately. Defaulting it here would plant a
    test-local default opposite production's own (``"bungee"``), and this test
    class exists to check which launch type reaches the simulator. Each caller
    therefore states the gate it means to reach.

    A MagicMock is the wrong fixture for exactly that check: if production stopped
    reading ``launch.launch_type``, a fabricated child mock is also != "container",
    so the "bungee skips" test would keep passing while covering nothing. The real
    model raises on a renamed field instead (these are plain attribute reads, not
    ``getattr`` with a default).
    """
    settings = GcsSettings()
    settings.simulation.sim_mode = sim_mode
    settings.launch.launch_type = launch_type
    settings.launch.esp32_port = port
    return settings


class TestEsp32SimManagement(unittest.TestCase):
    """Test the module-level _esp32_sim management functions."""

    def setUp(self):
        # Reset module state before each test
        ctrl_mod._esp32_sim = None

    def tearDown(self):
        ctrl_mod._esp32_sim = None

    def test_stop_when_not_running_is_noop(self):
        ctrl_mod.stop_esp32_sim_if_running()
        self.assertIsNone(ctrl_mod._esp32_sim)

    def test_stop_when_running_calls_stop(self):
        mock_sim = MagicMock()
        ctrl_mod._esp32_sim = mock_sim
        ctrl_mod.stop_esp32_sim_if_running()
        mock_sim.stop.assert_called_once()
        self.assertIsNone(ctrl_mod._esp32_sim)

    def test_sim_status_reflects_state(self):
        self.assertIsNone(ctrl_mod._esp32_sim)
        mock_sim = MagicMock()
        ctrl_mod._esp32_sim = mock_sim
        self.assertIsNotNone(ctrl_mod._esp32_sim)


class TestAutoStartEsp32Sim(unittest.TestCase):
    """Test auto_start_esp32_sim behavior."""

    def setUp(self):
        ctrl_mod._esp32_sim = None

    def tearDown(self):
        ctrl_mod._esp32_sim = None

    @patch.object(ctrl_mod.settings_store, "get")
    def test_skips_when_sim_mode_off(self, mock_get):
        # Container launch type, so only sim_mode can be the reason it skips.
        mock_get.return_value = _settings(sim_mode=False, launch_type="container")
        self.assertFalse(ctrl_mod.auto_start_esp32_sim())
        self.assertIsNone(ctrl_mod._esp32_sim)

    @patch.object(ctrl_mod.settings_store, "get")
    def test_skips_when_launch_type_bungee(self, mock_get):
        mock_get.return_value = _settings(sim_mode=True, launch_type="bungee")
        self.assertFalse(ctrl_mod.auto_start_esp32_sim())
        self.assertIsNone(ctrl_mod._esp32_sim)

    @patch.object(ctrl_mod.settings_store, "get")
    def test_skips_when_already_running(self, mock_get):
        # Both gates open, so only the already-running check can stop it.
        mock_get.return_value = _settings(sim_mode=True, launch_type="container")
        ctrl_mod._esp32_sim = MagicMock()  # already running
        self.assertFalse(ctrl_mod.auto_start_esp32_sim())

    @patch.dict(os.environ, {}, clear=False)
    @patch("gcs.backend.esp32_simulator.Esp32Simulator")
    @patch.object(ctrl_mod.settings_store, "get")
    def test_starts_simulator_in_sim_container_mode(self, mock_get, mock_cls):
        # A backend started without the isolated launcher keeps the settings port.
        os.environ.pop("GCS_CHAT_INDEX", None)
        mock_get.return_value = _settings(
            sim_mode=True, launch_type="container", port=8080)
        mock_instance = MagicMock()
        mock_cls.return_value = mock_instance
        self.assertTrue(ctrl_mod.auto_start_esp32_sim())
        mock_cls.assert_called_once_with(port=8080)
        mock_instance.start.assert_called_once()
        self.assertIs(ctrl_mod._esp32_sim, mock_instance)

    @patch.dict(os.environ, {"GCS_CHAT_INDEX": "7"})
    @patch("gcs.backend.esp32_simulator.Esp32Simulator")
    @patch.object(ctrl_mod.settings_store, "get")
    def test_launcher_slot_owns_the_simulator_port(self, mock_get, mock_cls):
        # The slot's port, not the persisted one: concurrent stacks never
        # contend for it and a run's port does not depend on the OS.
        mock_get.return_value = _settings(
            sim_mode=True, launch_type="container", port=8080)
        self.assertTrue(ctrl_mod.auto_start_esp32_sim())
        mock_cls.assert_called_once_with(port=ip.esp32_sim_port(7))

    @patch("gcs.backend.esp32_simulator.Esp32Simulator")
    @patch.object(ctrl_mod.settings_store, "get")
    def test_failed_start_leaves_no_running_simulator(self, mock_get, mock_cls):
        mock_get.return_value = _settings(sim_mode=True, launch_type="container")
        mock_cls.return_value.start.side_effect = OSError("address in use")
        with self.assertRaises(OSError):
            ctrl_mod.auto_start_esp32_sim()
        self.assertIsNone(ctrl_mod._esp32_sim)


class TestEsp32SimulatorBinding(unittest.TestCase):
    def test_listens_on_loopback_only(self):
        # Every client is this backend (launch and status use 127.0.0.1).
        simulator = Esp32Simulator(port=0)
        simulator.start()
        try:
            self.assertEqual(simulator.server.server_address[0], "127.0.0.1")
        finally:
            simulator.stop()
            simulator.server.server_close()


if __name__ == "__main__":
    unittest.main()
