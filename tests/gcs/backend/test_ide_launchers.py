"""IDE backend/frontend launchers resolve THIS project's chat from the registry
(no hardcoded chat 0), and backend + frontend land on the SAME slot."""
import importlib.util
import os
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import patch

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, _ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class TestIdeLaunchers(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._env = patch.dict(
            os.environ, {"GCS_INSTANCE_REGISTRY": str(pathlib.Path(self._tmp.name) / "r.json")})
        self._env.start()
        from gcs.backend import instance_registry as reg
        self.reg = reg
        self._free = patch.object(reg, "slot_ports_free", lambda n: True)
        self._free.start()

    def tearDown(self):
        self._free.stop()
        self._env.stop()
        self._tmp.cleanup()

    def test_backend_and_frontend_resolve_same_slot(self):
        be = _load("gcs_backend")
        fe = _load("gcs_frontend")
        e_be = be.resolve_slot()
        e_fe = fe.resolve_slot()
        # Same project/owner -> same chat slot (not two colliding chat-0s).
        self.assertEqual(e_be["chat_index"], e_fe["chat_index"])
        self.assertEqual(e_be["backend"], e_fe["backend"])

    def test_backend_never_hardcodes_chat_0(self):
        be = _load("gcs_backend")
        # occupy chat 0 so a correct launcher must pick a different slot
        self.reg.claim(owner="session:other")   # takes chat 0
        e = be.resolve_slot()
        self.assertNotEqual(e["chat_index"], 0)   # would be 0 if it hardcoded

    def test_backend_refuses_while_previous_launch_alive(self):
        be = _load("gcs_backend")
        # First resolve guards the slot with THIS live process's pid (the IDE
        # launcher runs uvicorn in-process), so a same-slot relaunch refuses.
        be.resolve_slot()
        with self.assertRaises(RuntimeError):
            be.resolve_slot()

    def test_backend_takes_over_after_previous_launch_died(self):
        be = _load("gcs_backend")
        first = be.resolve_slot()
        with patch.object(self.reg, "_pid_alive", return_value=False), \
             patch.object(self.reg, "_port_bound", return_value=False):
            second = be.resolve_slot()
        self.assertEqual(second["chat_index"], first["chat_index"])


if __name__ == "__main__":
    unittest.main()
