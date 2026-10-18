"""The generated sim MediaMTX config opens only the slot's three ports."""
import re
import unittest
from pathlib import Path

import yaml

from gcs.backend import instance_ports as ip
from gcs.backend import sim_video_config as cfg

FFMPEG = Path("C:/tools/ffmpeg/bin/ffmpeg.exe")


def _config(chat, uavs=3, font=None):
    return yaml.safe_load(cfg.mediamtx_config(chat, uavs, FFMPEG, 3000 + chat, font))


class TestListeners(unittest.TestCase):
    def test_only_the_three_allocated_ports_open_on_loopback(self):
        for chat in (0, 7, 39):
            data = _config(chat)
            addresses = {k: v for k, v in data.items() if k.lower().endswith("address") and v}
            self.assertEqual(addresses, {
                "rtspAddress": f"127.0.0.1:{ip.sim_video_rtsp_port(chat)}",
                "webrtcAddress": f"127.0.0.1:{ip.sim_video_whep_port(chat)}",
                "webrtcLocalUDPAddress": f"127.0.0.1:{ip.sim_video_webrtc_udp_port(chat)}",
            })

    def test_no_other_port_number_appears_outside_the_publish_commands(self):
        data = _config(4)
        data["paths"] = {}
        ports = {int(p) for p in re.findall(r":(\d{4,5})\b", yaml.safe_dump(data))}
        self.assertEqual(ports, {ip.sim_video_rtsp_port(4), ip.sim_video_whep_port(4),
                                 ip.sim_video_webrtc_udp_port(4), 3004})  # 3004: origin

    def test_everything_else_is_disabled(self):
        data = _config(2)
        for key in ("rtmp", "hls", "srt", "moq", "api", "metrics", "pprof", "playback"):
            self.assertIs(data[key], False, key)
        self.assertEqual(data["rtspTransports"], ["tcp"])
        self.assertEqual(data["rtspEncryption"], "no")
        self.assertEqual(data["webrtcLocalTCPAddress"], "")
        self.assertIs(data["webrtcEncryption"], False)
        self.assertIs(data["webrtcIPsFromInterfaces"], False)

    def test_origins_are_the_slots_frontend(self):
        self.assertEqual(_config(5)["webrtcAllowOrigins"],
                         ["http://localhost:3005", "http://127.0.0.1:3005"])

    def test_two_slots_have_distinct_configs_and_ports(self):
        a, b = _config(1), _config(2)
        self.assertNotEqual(a, b)
        for key in ("rtspAddress", "webrtcAddress", "webrtcLocalUDPAddress"):
            self.assertNotEqual(a[key], b[key])


class TestPaths(unittest.TestCase):
    def test_one_on_demand_path_per_uav(self):
        paths = _config(3, uavs=2)["paths"]
        self.assertEqual(sorted(paths), ["uav1", "uav2"])
        for name, path in paths.items():
            self.assertIs(path["runOnDemandRestart"], True)
            self.assertIn(f"rtsp://127.0.0.1:{ip.sim_video_rtsp_port(3)}/{name}", path["runOnDemand"])

    def test_publish_command_is_low_latency_h264_over_tcp(self):
        command = cfg.ffmpeg_publish_command(FFMPEG, 3, 2)
        for part in ("libx264", "zerolatency", "-bf 0", "-rtsp_transport tcp", "-f rtsp",
                     "SIM TEST UAV 2", "640x360"):
            self.assertIn(part, command)
        self.assertLessEqual(cfg.KEYFRAME_INTERVAL_FRAMES / cfg.FRAMERATE, 2.0)
        self.assertIn(f"-g {cfg.KEYFRAME_INTERVAL_FRAMES}", command)
        self.assertNotIn("'", command)  # double quotes only, for sh and cmd

    def test_font_drive_colon_is_escaped_for_the_filtergraph(self):
        command = cfg.ffmpeg_publish_command(FFMPEG, 1, 1, Path("C:/Windows/Fonts/arial.ttf"))
        self.assertIn(r"fontfile='C\:/Windows/Fonts/arial.ttf'", command)

    def test_stream_uri_and_label(self):
        self.assertEqual(cfg.stream_uri(6, 2), f"rtsp://127.0.0.1:{ip.sim_video_rtsp_port(6)}/uav2")
        self.assertEqual(cfg.stream_label(2), "SIM TEST UAV 2")
