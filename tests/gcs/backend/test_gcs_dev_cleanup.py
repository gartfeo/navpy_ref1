"""gcs_dev_cleanup.py stops only THIS session's chat by default (not broad)."""
import importlib.util
import pathlib
import sys
import unittest
from unittest.mock import patch

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load():
    spec = importlib.util.spec_from_file_location(
        "gcs_dev_cleanup", _ROOT / "scripts" / "gcs_dev_cleanup.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclass resolution needs the module registered
    spec.loader.exec_module(mod)
    return mod


class TestSessionScopedDefault(unittest.TestCase):
    def test_stops_only_this_sessions_ports_and_sysids(self):
        c = _load()
        entry = {"chat_index": 2, "backend": 8002, "frontend": 3002, "sysids": [7, 8, 9]}
        with patch.object(c.reg, "owner_for", return_value="session:x"), \
                patch.object(c.reg, "find_for_owner", return_value=entry), \
                patch.object(c, "stop_port_listener") as spl, \
                patch.object(c, "stop_navpy_sim") as sns, \
                patch.object(c.reg, "release") as rel:
            c.stop_this_session(dry_run=False)
        ports = [call.args[0] for call in spl.call_args_list]
        self.assertIn(8002, ports)
        self.assertIn(3002, ports)
        self.assertNotIn(8000, ports)   # never touches chat 0's hardcoded ports
        self.assertNotIn(3000, ports)
        self.assertEqual(sns.call_args.kwargs["sysids"], {7, 8, 9})  # only this chat's sims
        rel.assert_called_once_with(2)

    def test_no_registered_chat_does_nothing(self):
        c = _load()
        with patch.object(c.reg, "owner_for", return_value="session:x"), \
                patch.object(c.reg, "find_for_owner", return_value=None), \
                patch.object(c, "stop_port_listener") as spl, \
                patch.object(c, "stop_navpy_sim") as sns:
            c.stop_this_session(dry_run=False)
        spl.assert_not_called()
        sns.assert_not_called()

    def test_broad_sweep_only_via_all(self):
        c = _load()
        # The legacy broad sweep passes sysids=None (all) and hardcoded ports.
        with patch.object(c, "stop_port_listener") as spl, \
                patch.object(c, "stop_navpy_sim") as sns:
            c.stop_stale_processes(dry_run=True)
        ports = {call.args[0] for call in spl.call_args_list}
        self.assertEqual(ports, {c.BACKEND_PORT, c.FRONTEND_PORT})
        self.assertIsNone(sns.call_args.kwargs["sysids"])


if __name__ == "__main__":
    unittest.main()
