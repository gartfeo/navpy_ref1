"""Start, record and stop of the sim video test source, with no real processes."""
import importlib.util
import logging
import os
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend import instance_ports as ip
from gcs.backend import sim_video_process as svp

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SCRIPTS = str(_ROOT / "scripts")


def _load(name):
    spec = importlib.util.spec_from_file_location(name, _ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class _ScriptsPath(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, _SCRIPTS)
        self.addCleanup(sys.path.remove, _SCRIPTS)


class TestStart(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def _start(self, mediamtx="mtx", ffmpeg="ff", spawn=None):
        spawn = spawn or MagicMock()
        with patch.object(svp.binaries, "find_mediamtx",
                          return_value=pathlib.Path(mediamtx) if mediamtx else None), \
             patch.object(svp.binaries, "find_ffmpeg",
                          return_value=pathlib.Path(ffmpeg) if ffmpeg else None), \
             patch.object(svp.binaries, "find_font", return_value=None):
            return svp.start_sim_video(4, 3, 3004, self.dir, spawn), spawn

    def test_writes_the_config_and_spawns_mediamtx_on_it(self):
        process, spawn = self._start()
        self.assertIs(process, spawn.return_value)
        command, _cwd, log_file = spawn.call_args.args
        config = svp.config_path(self.dir, 4)
        self.assertEqual(command, [str(pathlib.Path("mtx")), str(config)])
        self.assertIn(str(ip.sim_video_rtsp_port(4)), config.read_text(encoding="utf-8"))
        self.assertEqual(log_file.parent, self.dir)

    def test_missing_binary_leaves_video_off_and_logs(self):
        for missing in ({"mediamtx": None}, {"ffmpeg": None}):
            spawn = MagicMock()
            with self.subTest(missing=missing), self.assertLogs(svp.log, logging.WARNING):
                process, _ = self._start(spawn=spawn, **missing)
            self.assertIsNone(process)
            spawn.assert_not_called()

    def test_spawn_failure_leaves_video_off_and_logs(self):
        with self.assertLogs(svp.log, logging.ERROR):
            process, _ = self._start(spawn=MagicMock(side_effect=OSError("no exec")))
        self.assertIsNone(process)


class TestLauncherGlue(_ScriptsPath):
    def setUp(self):
        super().setUp()
        self.glue = _load("gcs_launch_video")
        self.entry = {"chat_index": 4, "frontend": 3004}

    def test_off_by_default_starts_nothing(self):
        with patch.object(self.glue, "video_test_source_enabled", return_value=False), \
             patch.object(self.glue, "start_sim_video") as start, \
             patch.object(self.glue.reg, "record_pids") as record:
            self.glue.launch_sim_video(self.entry, 3)
        start.assert_not_called()
        record.assert_not_called()

    def test_on_records_the_pid_in_the_registry(self):
        with patch.object(self.glue, "video_test_source_enabled", return_value=True), \
             patch.object(self.glue, "start_sim_video", return_value=MagicMock(pid=555)) as start, \
             patch.object(self.glue.reg, "record_pids") as record:
            self.glue.launch_sim_video(self.entry, 2)
        record.assert_called_once_with(4, video_pid=555)
        self.assertEqual(start.call_args.args[:3], (4, 2, 3004))

    def test_failed_start_records_nothing_and_does_not_raise(self):
        for start in (MagicMock(return_value=None), MagicMock(side_effect=RuntimeError("x"))):
            with self.subTest(start=start), \
                 patch.object(self.glue, "video_test_source_enabled", return_value=True), \
                 patch.object(self.glue, "start_sim_video", start), \
                 patch.object(self.glue.reg, "record_pids") as record:
                self.glue.launch_sim_video(self.entry, 3)
            record.assert_not_called()

    def test_unreadable_settings_do_not_stop_the_launch(self):
        with patch.object(self.glue, "video_test_source_enabled", side_effect=ValueError("bad")), \
             patch.object(self.glue.reg, "record_pids") as record:
            self.glue.launch_sim_video(self.entry, 3)
        record.assert_not_called()

    def test_reaps_exactly_the_slots_three_ports(self):
        with patch.object(self.glue.reg, "reap_port", return_value=[]) as reap:
            self.glue.reap_sim_video_ports(4, {9})
        self.assertEqual(
            [(c.args[0], c.kwargs.get("kind", "tcp")) for c in reap.call_args_list],
            [(ip.sim_video_rtsp_port(4), "tcp"), (ip.sim_video_whep_port(4), "tcp"),
             (ip.sim_video_webrtc_udp_port(4), "udp")])


class TestLaunchSurvivesVideoFailure(_ScriptsPath):
    def test_stack_still_spawns_when_video_cannot_start(self):
        launch = _load("gcs_launch")
        entry = {"chat_index": 5, "frontend": 3005, "backend": 8005, "monitor": 15555,
                 "mission_planner": 14555, "sysids": [16, 17, 18]}
        spawn = MagicMock()
        with patch.object(launch, "_spawn", spawn), \
             patch.object(launch.reg, "claim", return_value=entry), \
             patch.object(launch.reg, "registered_pids", return_value=set()), \
             patch.object(launch.reg, "reap_port", return_value=[]), \
             patch.object(launch.reg, "reap_companion_ports", return_value=[]), \
             patch.object(launch.reg, "record_pids") as record, \
             patch.object(launch.reg, "release"), \
             patch.object(launch.reg, "owner_for", return_value="session:test"), \
             patch.object(launch.webbrowser, "open"), \
             patch("gcs_launch_video.video_test_source_enabled", return_value=True), \
             patch("gcs_launch_video.start_sim_video", return_value=None), \
             patch.object(sys, "argv", ["gcs_launch.py", "--no-browser", "--no-sitl"]):
            rc = launch.main()
        self.assertEqual(rc, 0)
        self.assertEqual(spawn.call_count, 2)  # backend and frontend
        self.assertTrue(all("video_pid" not in c.kwargs for c in record.call_args_list))


class TestRegistryAndStop(_ScriptsPath):
    def setUp(self):
        super().setUp()
        self._tmp = tempfile.TemporaryDirectory()
        env = patch.dict(os.environ, {"GCS_INSTANCE_REGISTRY": str(pathlib.Path(self._tmp.name) / "r.json")})
        env.start()
        self.addCleanup(env.stop)
        self.addCleanup(self._tmp.cleanup)
        from gcs.backend import instance_registry as reg
        self.reg = reg
        free = patch.object(reg, "slot_ports_free", lambda n: True)
        free.start()
        self.addCleanup(free.stop)

    def test_registry_records_the_video_pid_and_protects_it(self):
        entry = self.reg.claim(label="gcs", owner="o", hi=ip.interactive_chat_hi())
        self.reg.record_pids(entry["chat_index"], video_pid=4321)
        got = self.reg.get(entry["chat_index"])
        self.assertEqual(got["video_pid"], 4321)
        self.assertIn("video_pid_start", got)
        with patch.object(self.reg, "live", return_value=[got]):
            self.assertIn(4321, self.reg.registered_pids())

    def test_a_new_claim_clears_the_previous_video_pid(self):
        entry = self.reg.claim(label="gcs", owner="o", hi=ip.interactive_chat_hi())
        self.reg.record_pids(entry["chat_index"], video_pid=4321)
        with patch.object(self.reg, "_pid_alive", return_value=False), \
             patch.object(self.reg, "_port_bound", return_value=False):
            again = self.reg.claim(label="gcs", owner="o", hi=ip.interactive_chat_hi(),
                                   launcher_pid=77)
        self.assertEqual(again["chat_index"], entry["chat_index"])
        self.assertIsNone(again["video_pid"])

    def _stop(self, matches):
        stop = _load("gcs_stop")
        entry = {"backend_pid": None, "frontend_pid": None, "video_pid": 4321, "video_pid_start": 5}
        killed = []
        with patch.object(stop.reg, "get", return_value=entry), \
             patch.object(stop.reg, "release"), \
             patch.object(stop.reg, "registered_pids", return_value={999}), \
             patch.object(stop.reg, "reap_port", return_value=[]) as reap, \
             patch.object(stop.reg, "reap_companion_ports", return_value=[]), \
             patch.object(stop.reg, "pid_matches", return_value=matches), \
             patch.object(stop, "_kill_tree", side_effect=killed.append), \
             patch.object(stop.swarm_run, "cleanup"):
            stop.stop_chat(3, kill_sitl=True)
        return killed, reap

    def test_gcs_stop_kills_only_this_slots_video_tree(self):
        killed, reap = self._stop(True)
        self.assertEqual([p for p in killed if p], [4321])
        reaped = {c.args[0] for c in reap.call_args_list}
        self.assertTrue({ip.sim_video_rtsp_port(3), ip.sim_video_whep_port(3),
                         ip.sim_video_webrtc_udp_port(3)} <= reaped)

    def test_gcs_stop_spares_a_recycled_video_pid(self):
        killed, _ = self._stop(False)
        self.assertNotIn(4321, killed)

    def test_video_port_reap_spares_registered_pids(self):
        _, reap = self._stop(False)
        video_ports = {ip.sim_video_rtsp_port(3), ip.sim_video_whep_port(3),
                       ip.sim_video_webrtc_udp_port(3)}
        calls = [c for c in reap.call_args_list if c.args[0] in video_ports]
        self.assertEqual(len(calls), 3)
        self.assertTrue(all(c.kwargs.get("exclude") == {999} for c in calls))
