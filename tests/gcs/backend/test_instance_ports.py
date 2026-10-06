"""Tests for per-chat port/sys_id isolation (instance_ports + wiring)."""
import importlib.util
import os
import pathlib
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend import instance_ports as ip
from gcs.backend import navpy_sim_runtime as runtime_mod
from gcs.backend.routes import settings as settings_routes
from gcs.backend.settings_model import GcsSettings

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load_launcher():
    spec = importlib.util.spec_from_file_location(
        "gcs_launch", _ROOT / "scripts" / "gcs_launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestInstancePorts(unittest.TestCase):
    def test_base_ports_for_chat_zero(self):
        self.assertEqual(ip.frontend_port(0), 3000)
        self.assertEqual(ip.backend_port(0), 8000)
        # Mission Planner on the MAVLink-standard 14550 -> auto-connects for chat 0.
        self.assertEqual(ip.mission_planner_port(0), 14550)
        self.assertEqual(ip.monitor_port(0), 15550)

    def test_ports_shift_with_chat_index(self):
        self.assertEqual(ip.frontend_port(2), 3002)
        self.assertEqual(ip.backend_port(2), 8002)
        self.assertEqual(ip.mission_planner_port(2), 14552)
        self.assertEqual(ip.monitor_port(2), 15552)

    def test_gcs_and_mission_planner_never_collide(self):
        # Disjoint UDP bands across the entire supported chat range.
        monitors = {ip.monitor_port(n) for n in range(ip.MAX_CHAT_INDEX + 1)}
        mps = {ip.mission_planner_port(n) for n in range(ip.MAX_CHAT_INDEX + 1)}
        self.assertEqual(monitors & mps, set())

    def test_verify_port_derivation(self):
        self.assertEqual(ip.verify_port(0), 16550)
        self.assertEqual(ip.verify_port(1), 16551)
        self.assertEqual(ip.verify_port(5), 16555)

    def test_verify_port_never_collides_with_other_udp_bands(self):
        # The launcher binds the verify port while the backend holds the monitor
        # and Mission Planner may hold its port — so it must be disjoint from both
        # (and from frontend/backend) across the whole chat range.
        rng = range(ip.MAX_CHAT_INDEX + 1)
        verify = {ip.verify_port(n) for n in rng}
        others = (
            {ip.monitor_port(n) for n in rng}
            | {ip.mission_planner_port(n) for n in rng}
            | {ip.frontend_port(n) for n in rng}
            | {ip.backend_port(n) for n in rng}
        )
        self.assertEqual(verify & others, set())

    def test_sysids_are_globally_unique_and_contiguous(self):
        self.assertEqual(ip.sysids_for_chat(0), [1, 2, 3])
        self.assertEqual(ip.sysids_for_chat(1), [4, 5, 6])
        self.assertEqual(ip.sysids_for_chat(2), [7, 8, 9])
        all_sysids = [s for n in range(5) for s in ip.sysids_for_chat(n)]
        self.assertEqual(len(all_sysids), len(set(all_sysids)))

    def test_top_sysid_fits_in_uint8(self):
        self.assertLessEqual(max(ip.sysids_for_chat(ip.MAX_CHAT_INDEX)), ip.MAX_SYSID)

    def test_companion_device_is_local_udp_server(self):
        # sys_id g -> local UDP bind at 5760 + 10*(g-1); SITL serial0
        # (udpclient) streams into it, so no host detection is involved.
        self.assertEqual(ip.companion_device(1), "udp:0.0.0.0:5760")
        self.assertEqual(ip.companion_device(2), "udp:0.0.0.0:5770")
        self.assertEqual(ip.companion_device(3), "udp:0.0.0.0:5780")
        self.assertEqual(ip.companion_device(4), "udp:0.0.0.0:5790")
        self.assertEqual(ip.companion_device(6), "udp:0.0.0.0:5810")

    def test_companion_device_needs_no_wsl_host_detection(self):
        # The TCP-era WSL-IP machinery must stay deleted: a UDP server bind
        # has no peer host to resolve, and the old 'wsl hostname -I'
        # subprocess was a startup latency + failure source.
        for name in ("companion_host", "_detect_wsl_host", "_parse_wsl_host"):
            self.assertFalse(hasattr(ip, name), name)

    def test_monitor_device_string(self):
        self.assertEqual(ip.monitor_device(0), "udp:0.0.0.0:15550")
        self.assertEqual(ip.monitor_device(3), "udp:0.0.0.0:15553")

    def test_router_win_ports_only_gcs_facing(self):
        # Companions use dedicated per-vehicle UDP links, so only the two GCS
        # ports are router-exported here.
        self.assertEqual(ip.router_win_ports(0), [14550, 15550])
        self.assertEqual(ip.router_win_ports(1), [14551, 15551])

    def test_bands_are_disjoint_and_cover_the_split(self):
        lo_i, hi_i = ip.interactive_band()
        lo_e, hi_e = ip.eval_band()
        self.assertEqual((lo_i, hi_i), (0, ip.EVAL_CHAT_MIN - 1))
        self.assertEqual((lo_e, hi_e), (ip.EVAL_CHAT_MIN, ip.MAX_CHAT_INDEX))
        self.assertLess(hi_i, lo_e)  # no overlap

    def test_band_for_label_maps_eval_labels_to_eval_band(self):
        self.assertEqual(ip.band_for_label("sitl-eval"), ip.eval_band())
        # Everything else — including None and unknown labels — is interactive.
        for label in ("gcs", "sitl", "", None, "whatever"):
            self.assertEqual(ip.band_for_label(label), ip.interactive_band())

    def test_in_band_is_inclusive(self):
        self.assertTrue(ip.in_band(0, ip.interactive_band()))
        self.assertTrue(ip.in_band(ip.EVAL_CHAT_MIN - 1, ip.interactive_band()))
        self.assertFalse(ip.in_band(ip.EVAL_CHAT_MIN, ip.interactive_band()))
        self.assertTrue(ip.in_band(ip.EVAL_CHAT_MIN, ip.eval_band()))
        self.assertTrue(ip.in_band(ip.MAX_CHAT_INDEX, ip.eval_band()))

    def test_chat_index_env(self):
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": "5"}):
            self.assertEqual(ip.chat_index(), 5)
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": ""}):
            self.assertIsNone(ip.chat_index())
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": "nope"}):
            self.assertIsNone(ip.chat_index())
        env = dict(os.environ)
        env.pop("GCS_CHAT_INDEX", None)
        with patch.dict(os.environ, env, clear=True):
            self.assertIsNone(ip.chat_index())


class TestSimConnectionUsesChatIndex(unittest.TestCase):
    def _settings(self):
        m = MagicMock()
        m.simulation.sitl_presets = ["udp:0.0.0.0:14560", "udp:0.0.0.0:14570"]
        return m

    def test_uses_companion_device_when_chat_index_set(self):
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": "1"}):
            # chat 1 -> sys_ids 4,5,6 -> companion UDP binds 5790/5800/5810
            self.assertEqual(
                runtime_mod.sim_connection_from_settings(4, self._settings()),
                "udp:0.0.0.0:5790",
            )

    def test_falls_back_to_presets_without_chat_index(self):
        env = dict(os.environ)
        env.pop("GCS_CHAT_INDEX", None)
        with patch.dict(os.environ, env, clear=True):
            self.assertEqual(
                runtime_mod.sim_connection_from_settings(1, self._settings()),
                "udp:0.0.0.0:14560",
            )


class TestSettingsDefaults(unittest.TestCase):
    def test_demo_and_dev_on_by_default(self):
        s = GcsSettings()
        self.assertTrue(s.simulation.sim_mode)
        self.assertTrue(s.simulation.dev_mode)

    def test_auto_connect_on_by_default(self):
        self.assertTrue(GcsSettings().connection.auto_connect)

    def test_default_connection_device_is_gcs_monitor_port(self):
        self.assertEqual(GcsSettings().connection.default_device, ip.monitor_device(0))


class TestSettingsDeviceInjection(unittest.TestCase):
    def test_injects_monitor_device_in_sim_mode(self):
        data = {"simulation": {"sim_mode": True}, "connection": {"default_device": "x"}}
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": "2"}):
            out = settings_routes._with_instance_device(data)
        # chat 2 monitor is the per-run default connection.
        self.assertEqual(out["connection"]["default_device"], "udp:0.0.0.0:15552")

    def test_injects_companion_presets_in_sim_mode(self):
        data = {"simulation": {"sim_mode": True}, "connection": {"default_device": "x"}}
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": "1"}):
            out = settings_routes._with_instance_device(data)
        # chat 1 -> sys_ids 4,5,6 -> per-vehicle companion UDP binds.
        self.assertEqual(
            out["simulation"]["sitl_presets"],
            ["udp:0.0.0.0:5790", "udp:0.0.0.0:5800", "udp:0.0.0.0:5810"],
        )

    def test_no_injection_when_sim_mode_off(self):
        data = {"simulation": {"sim_mode": False}, "connection": {"default_device": "x"}}
        with patch.dict(os.environ, {"GCS_CHAT_INDEX": "2"}):
            out = settings_routes._with_instance_device(data)
        self.assertEqual(out["connection"]["default_device"], "x")

    def test_no_injection_without_chat_index(self):
        data = {"simulation": {"sim_mode": True}, "connection": {"default_device": "x"}}
        env = dict(os.environ)
        env.pop("GCS_CHAT_INDEX", None)
        with patch.dict(os.environ, env, clear=True):
            out = settings_routes._with_instance_device(data)
        self.assertEqual(out["connection"]["default_device"], "x")


class TestLauncherImports(unittest.TestCase):
    def test_launcher_loads_and_claims_via_registry(self):
        launcher = _load_launcher()
        # Slot allocation now goes through the shared registry (atomic, cross-clone),
        # not a local port scan.
        self.assertTrue(hasattr(launcher, "reg"))
        self.assertTrue(callable(launcher.main))


if __name__ == "__main__":
    unittest.main()
