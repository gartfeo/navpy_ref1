"""gcs_stop resolves THIS directory's slot in the band it means: the default
stops the interactive GCS slot, ``--eval`` stops the eval (SITL-only) slot. This
is the regression guard for the eval-sweep teardown killing a concurrent
interactive session launched from the same worktree."""
import importlib.util
import os
import pathlib
import sys
import tempfile
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


class TestGcsStopBandResolution(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._env = patch.dict(
            os.environ, {"GCS_INSTANCE_REGISTRY": str(pathlib.Path(self._tmp.name) / "r.json")})
        self._env.start()
        from gcs.backend import instance_registry as reg
        self.reg = reg
        self._free = patch.object(reg, "slot_ports_free", lambda n: True)
        self._free.start()
        self.stop = _load_stop()
        # The SAME directory owns both an interactive GCS slot and an eval slot.
        self.owner = reg.owner_for(str(self.stop.ROOT))
        self.gui = reg.claim(label="gcs", owner=self.owner, hi=ip.interactive_chat_hi())
        self.ev = reg.claim(label="sitl-eval", owner=self.owner, lo=ip.EVAL_CHAT_MIN)

    def tearDown(self):
        self._free.stop()
        self._env.stop()
        self._tmp.cleanup()

    def _run(self, argv):
        with patch.object(self.stop, "stop_chat") as mstop, \
             patch.object(sys, "argv", ["gcs_stop.py", *argv]):
            rc = self.stop.main()
        return rc, mstop

    def test_default_stops_the_interactive_slot(self):
        rc, mstop = self._run([])
        self.assertEqual(rc, 0)
        mstop.assert_called_once()
        self.assertEqual(mstop.call_args.args[0], self.gui["chat_index"])

    def test_eval_stops_the_eval_slot(self):
        rc, mstop = self._run(["--eval"])
        self.assertEqual(rc, 0)
        mstop.assert_called_once()
        self.assertEqual(mstop.call_args.args[0], self.ev["chat_index"])

    def test_eval_teardown_never_targets_the_interactive_slot(self):
        # The exact regression: eval-band teardown must not resolve/stop the
        # interactive slot in the same directory.
        _, mstop = self._run(["--eval"])
        self.assertNotEqual(mstop.call_args.args[0], self.gui["chat_index"])

    def test_missing_eval_slot_errors_without_falling_back_to_interactive(self):
        self.reg.release(self.ev["chat_index"])  # only the interactive slot remains
        rc, mstop = self._run(["--eval"])
        self.assertEqual(rc, 2)
        mstop.assert_not_called()  # must NOT stop the interactive slot instead

    def test_interactive_not_found_message_points_to_eval(self):
        # With no interactive slot, the default must guide the operator to --eval
        # (the eval-band slot) rather than only --all (the forbidden broad teardown).
        import contextlib
        import io
        self.reg.release(self.gui["chat_index"])
        self.reg.release(self.ev["chat_index"])
        buf = io.StringIO()
        with patch.object(self.stop, "stop_chat") as mstop, \
             patch.object(sys, "argv", ["gcs_stop.py"]), \
             contextlib.redirect_stderr(buf):
            rc = self.stop.main()
        self.assertEqual(rc, 2)
        mstop.assert_not_called()
        self.assertIn("--eval", buf.getvalue())


class TestGcsStopVerifiedKill(unittest.TestCase):
    """stop_chat must never tree-kill a pid that is no longer the recorded
    process — a recycled pid belongs to an unrelated session."""

    ENTRY = {"backend_pid": 111, "backend_pid_start": 1,
             "frontend_pid": 222, "frontend_pid_start": 2}

    def _stop_with(self, matches):
        stop = _load_stop()
        killed = []
        with patch.object(stop.reg, "get", return_value=dict(self.ENTRY)), \
             patch.object(stop, "_kill_tree", side_effect=killed.append), \
             patch.object(stop.reg, "reap_port", return_value=[]), \
             patch.object(stop.reg, "release"), \
             patch.object(stop.swarm_run, "cleanup"), \
             patch.object(stop.reg, "pid_matches", side_effect=matches):
            stop.stop_chat(3)
        return [pid for pid in killed if pid]

    def test_matching_pids_are_killed(self):
        killed = self._stop_with(lambda pid, start=None: True)
        self.assertEqual(killed, [111, 222])

    def test_recycled_pid_is_spared(self):
        # backend pid 111 no longer matches its creation stamp -> spared;
        # the still-matching frontend is killed normally.
        killed = self._stop_with(lambda pid, start=None: pid == 222)
        self.assertEqual(killed, [222])


if __name__ == "__main__":
    unittest.main()
