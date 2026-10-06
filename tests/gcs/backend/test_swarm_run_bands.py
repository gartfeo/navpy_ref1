"""swarm_run reserves eval-only SITL in the eval band, interactive in the low band."""
import importlib.util
import os
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import patch

from gcs.backend import instance_ports as ip

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load_swarm():
    spec = importlib.util.spec_from_file_location("swarm_run", _ROOT / "scripts" / "swarm_run.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class TestSwarmRunBands(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._env = patch.dict(
            os.environ, {"GCS_INSTANCE_REGISTRY": str(pathlib.Path(self._tmp.name) / "r.json")})
        self._env.start()
        from gcs.backend import instance_registry as reg
        self._free = patch.object(reg, "slot_ports_free", lambda n: True)
        self._free.start()

    def tearDown(self):
        self._free.stop()
        self._env.stop()
        self._tmp.cleanup()

    def test_eval_mode_reserves_in_eval_band(self):
        m = _load_swarm()
        n = m._resolve_chat(None, eval_mode=True)
        self.assertGreaterEqual(n, ip.EVAL_CHAT_MIN)

    def test_default_reserves_in_interactive_band(self):
        m = _load_swarm()
        n = m._resolve_chat(None, eval_mode=False)
        self.assertLess(n, ip.EVAL_CHAT_MIN)

    def test_explicit_chat_overrides_band(self):
        m = _load_swarm()
        self.assertEqual(m._resolve_chat(7, eval_mode=True), 7)

    def test_eval_mode_ignores_this_dirs_interactive_slot(self):
        # An interactive GCS is already up in this directory; an eval SITL run must
        # NOT reuse its interactive slot — it takes a fresh eval-band slot instead.
        m = _load_swarm()
        from gcs.backend import instance_registry as reg
        owner = reg.owner_for(str(m.ROOT))
        gui = reg.claim(label="gcs", owner=owner, hi=ip.interactive_chat_hi())
        n = m._resolve_chat(None, eval_mode=True)
        self.assertNotEqual(n, gui["chat_index"])
        self.assertGreaterEqual(n, ip.EVAL_CHAT_MIN)


if __name__ == "__main__":
    unittest.main()
