"""gcs_stop reaps its chat's ports and stops the SITL supervisor first."""
import importlib.util
import pathlib
import sys
import unittest
from unittest.mock import patch

from gcs.backend import instance_ports as ip

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load_stop():
    spec = importlib.util.spec_from_file_location("gcs_stop", _ROOT / "scripts" / "gcs_stop.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class TestGcsStopReap(unittest.TestCase):
    def test_stop_chat_reaps_frontend_and_backend_ports(self):
        m = _load_stop()
        with patch.object(m.reg, "get", return_value=None), \
             patch.object(m.reg, "release"), \
             patch.object(m.reg, "registered_pids", return_value=set()), \
             patch.object(m.reg, "reap_port", return_value=[]) as mreap, \
             patch.object(m.reg, "reap_companion_ports", return_value=[]), \
             patch.object(m.swarm_run, "cleanup"):
            m.stop_chat(3, kill_sitl=True)
        reaped_ports = {c.args[0] for c in mreap.call_args_list}
        self.assertIn(ip.frontend_port(3), reaped_ports)  # 3003
        self.assertIn(ip.backend_port(3), reaped_ports)   # 8003

    def test_stop_chat_reaps_companion_udp_ports_sparing_registered(self):
        # A leaked companion would not block a rebind (SO_REUSEADDR) — it
        # would silently split the next session's datagram stream. The reap
        # must run and must spare other live sessions' registered pids.
        m = _load_stop()
        protected = {111, 222}
        with patch.object(m.reg, "get", return_value=None), \
             patch.object(m.reg, "release"), \
             patch.object(m.reg, "registered_pids", return_value=protected), \
             patch.object(m.reg, "reap_port", return_value=[]), \
             patch.object(m.reg, "reap_companion_ports",
                          return_value=[]) as mcomp, \
             patch.object(m.swarm_run, "cleanup"):
            m.stop_chat(3, kill_sitl=True)
        mcomp.assert_called_once_with(3, exclude=protected)

    def test_stop_chat_kills_sitl_supervisor_before_swarm_cleanup(self):
        # Killing only the WSL SITL while swarm_run is still in its bring-up
        # verify loop makes it observe missing heartbeats and RELAUNCH the
        # swarm the operator just stopped — the supervisor dies first.
        m = _load_stop()
        entry = {"sitl_pid": 4242, "sitl_pid_start": None,
                 "backend_pid": None, "frontend_pid": None}
        order = []
        with patch.object(m.reg, "get", return_value=entry), \
             patch.object(m.reg, "release"), \
             patch.object(m.reg, "registered_pids", return_value=set()), \
             patch.object(m.reg, "reap_port", return_value=[]), \
             patch.object(m.reg, "reap_companion_ports", return_value=[]), \
             patch.object(m.reg, "pid_matches", return_value=True), \
             patch.object(m, "_kill_tree",
                          side_effect=lambda pid: order.append(("kill", pid))), \
             patch.object(m.swarm_run, "cleanup",
                          side_effect=lambda n: order.append(("cleanup", n))):
            m.stop_chat(3, kill_sitl=True)
        self.assertIn(("kill", 4242), order)
        self.assertLess(order.index(("kill", 4242)),
                        order.index(("cleanup", 3)))


if __name__ == "__main__":
    unittest.main()
