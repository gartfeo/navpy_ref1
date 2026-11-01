"""The sim companion announces the slot's test stream only when the setting is on."""
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend import instance_ports as ip
from gcs.backend import navpy_process_launch as launch
from gcs.backend import navpy_process_spawn as spawn
from gcs.backend import navpy_sim_runtime as runtime
from gcs.backend.settings_model import GcsSettings
from gcs.backend.sim_video_companion import sim_video_env
from navpy.app.perception.sim_video_test_source import (
    SIM_VIDEO_NAME_ENV,
    SIM_VIDEO_URI_ENV,
    with_sim_video_stream,
)
from navpy.core.perception.profile.vision_video_streams import parse_video_streams


def _settings(on=True, sim_mode=True):
    s = GcsSettings()
    s.simulation.sim_mode = sim_mode
    s.simulation.video_test_source = on
    return s


class TestGcsSide(unittest.TestCase):
    def test_env_names_the_uavs_own_path(self):
        with patch.object(ip, "chat_index", return_value=2):
            env = sim_video_env(_settings(), 2 * 3 + 2)
        self.assertEqual(env, {
            SIM_VIDEO_URI_ENV: f"rtsp://127.0.0.1:{ip.sim_video_rtsp_port(2)}/uav2",
            SIM_VIDEO_NAME_ENV: "SIM TEST UAV 2"})

    def test_nothing_when_off_outside_sim_or_foreign_sysid(self):
        with patch.object(ip, "chat_index", return_value=2):
            self.assertEqual(sim_video_env(_settings(on=False), 7), {})
            self.assertEqual(sim_video_env(_settings(sim_mode=False), 7), {})
            self.assertEqual(sim_video_env(_settings(), 1), {})
        with patch.object(ip, "chat_index", return_value=None):
            self.assertEqual(sim_video_env(_settings(), 7), {})

    def test_start_instance_passes_the_env_only_when_on(self):
        manager = MagicMock()
        with patch.object(runtime, "get_or_create_mgr", return_value=manager), \
             patch.object(runtime, "_mgr_factory", None), \
             patch.object(ip, "chat_index", return_value=2):
            runtime.start_instance(7, "udp:0.0.0.0:5760", _settings(on=False))
            self.assertNotIn("extra_env", manager.start.call_args.kwargs)
            runtime.start_instance(7, "udp:0.0.0.0:5760", _settings())
            self.assertEqual(manager.start.call_args.kwargs["extra_env"][SIM_VIDEO_NAME_ENV],
                             "SIM TEST UAV 1")

    def test_launch_merges_the_env_into_the_child_environment(self):
        popen = MagicMock()
        kwargs = dict(
            detector_debug_show=False, vision_profile=None, log_level=None,
            navigation_speedup=0.0, popen_factory=popen, output_reader=lambda inst: None,
            logger=MagicMock())
        with patch.object(spawn, "_create_job", side_effect=OSError), \
             patch.object(launch.rt, "Thread"):
            launch.launch_navpy_instance(7, "udp:0.0.0.0:5760", extra_env={"A": "b"}, **kwargs)
            self.assertEqual(popen.call_args.kwargs["env"]["A"], "b")
            launch.launch_navpy_instance(7, "udp:0.0.0.0:5760", **kwargs)
            self.assertNotIn("A", popen.call_args.kwargs["env"])


class TestNavpySide(unittest.TestCase):
    PROFILE = {"devices": [{"name": "cam"}, {"name": "cam2"}]}
    ENV = {SIM_VIDEO_URI_ENV: "rtsp://127.0.0.1:20551/uav1", SIM_VIDEO_NAME_ENV: "SIM TEST UAV 1"}

    def test_injects_stream_1_on_the_first_device_when_set(self):
        result = with_sim_video_stream(self.PROFILE, self.ENV)
        (stream,) = parse_video_streams(result["devices"][0], "cam")  # real validation
        self.assertEqual((stream.stream_id, stream.name, stream.uri, stream.encoding),
                         (1, "SIM TEST UAV 1", "rtsp://127.0.0.1:20551/uav1", "H264"))
        self.assertNotIn("video_streams", result["devices"][1])
        self.assertNotIn("video_streams", self.PROFILE["devices"][0])  # input untouched

    def test_unchanged_when_unset(self):
        self.assertIs(with_sim_video_stream(self.PROFILE, {}), self.PROFILE)
        self.assertIs(with_sim_video_stream(self.PROFILE, {SIM_VIDEO_URI_ENV: " "}), self.PROFILE)

    def test_a_profiles_own_streams_win(self):
        profile = {"devices": [{"name": "cam", "video_streams": []}]}
        self.assertIs(with_sim_video_stream(profile, self.ENV), profile)

    def test_profile_without_devices_is_unchanged(self):
        self.assertEqual(with_sim_video_stream({}, self.ENV), {})

    def test_composition_injects_only_for_simulated_companions(self):
        from navpy.app.perception import vision_profile_composition as comp
        seen = []
        with patch.object(comp, "resolve_profile", return_value=("p", dict(self.PROFILE), None)), \
             patch.object(comp, "build_camera_mounts",
                          side_effect=lambda raw, *a, **k: seen.append(raw) or []), \
             patch.dict("os.environ", self.ENV):
            for sim in (True, False):
                with self.assertRaises(ValueError):  # no mounts: stops after the capture
                    comp.build_vision_profile("p", MagicMock(), MagicMock(), None,
                                              use_sim_gimbal=sim)
        self.assertIn("video_streams", seen[0]["devices"][0])
        self.assertNotIn("video_streams", seen[1]["devices"][0])
