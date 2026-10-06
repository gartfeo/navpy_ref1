import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalLossPolicy,
    GimbalTrackingSetup,
)
from navpy.modules.vision.gimbal_rate_types import GimbalRateTrackerConfig
from navpy.modules.vision.target_zoom_tracker import TargetZoomTrackerConfig
from scripts.python import run_gimbal_tracker_assembly as runner_assembly
from scripts.python import run_gimbal_tracker_commands as runner_commands


def _load_script_module():
    script_path = Path(__file__).resolve().parents[3] / "scripts" / "python" / "run_gimbal_tracker.py"
    spec = importlib.util.spec_from_file_location("run_gimbal_tracker", script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TestRunGimbalTracker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_script_module()

    def test_parse_args_uses_demo_tracking_defaults(self):
        with patch.object(sys, "argv", ["run_gimbal_tracker.py"]):
            args = self.module.parse_args()

        self.assertIsNone(args.stream)
        self.assertEqual(args.preset, "vehicles")
        self.assertIsNone(args.conf)
        self.assertEqual(args.anchor, 5)
        self.assertAlmostEqual(args.pitch, 15.0)
        self.assertIsNone(args.max_rate)
        self.assertFalse(hasattr(args, "target_pixels"))
        self.assertFalse(hasattr(args, "perf_profile"))
        self.assertIsNone(args.detect_hz)
        self.assertIsNone(args.track_hz)
        self.assertIsNone(args.deep_search)
        self.assertIsNone(args.deep_search_hz)
        self.assertIsNone(args.tracker_backend)

    def test_runner_uses_detector_as_single_tracking_source(self):
        source = Path(runner_assembly.__file__).read_text(encoding="utf-8")

        self.assertIn("auto_target_lock=False", source)
        self.assertNotIn("TrackerCSRT", source)
        self.assertNotIn("TrackerKCF", source)
        self.assertNotIn("vis_tracker", source)
        self.assertNotIn("[VIS]", source)

    def test_anchor_pixel_center_returns_geometric_midpoint(self):
        # Anchor 5 is the geometric image centre, independent of the K
        # matrix's optical principal point (see history: we moved off the
        # optical centre so tracking holds the visual middle of the frame).
        ax, ay = self.module.anchor_pixel(5, 1920, 1080)
        self.assertAlmostEqual(ax, 1920 / 2)
        self.assertAlmostEqual(ay, 1080 / 2)

    def test_anchor_pixel_corners_no_bbox(self):
        pad = self.module.ANCHOR_PADDING_PX
        ax, ay = self.module.anchor_pixel(1, 1920, 1080)
        self.assertAlmostEqual(ax, pad)          # left
        self.assertAlmostEqual(ay, 1080 - pad)   # bottom

        ax, ay = self.module.anchor_pixel(9, 1920, 1080)
        self.assertAlmostEqual(ax, 1920 - pad)   # right
        self.assertAlmostEqual(ay, pad)           # top

    def test_anchor_pixel_insets_by_bbox_size(self):
        pad = self.module.ANCHOR_PADDING_PX
        # BL with 200x150 bbox (half=100, 75)
        # inset_x = 100 + pad, inset_y = 75 + pad
        ax, ay = self.module.anchor_pixel(1, 1920, 1080,
                                          bbox_half_w=100, bbox_half_h=75)
        self.assertAlmostEqual(ax, 100 + pad)
        self.assertAlmostEqual(ay, 1080 - (75 + pad))

        # TR with same bbox
        ax, ay = self.module.anchor_pixel(9, 1920, 1080,
                                          bbox_half_w=100, bbox_half_h=75)
        self.assertAlmostEqual(ax, 1920 - (100 + pad))
        self.assertAlmostEqual(ay, 75 + pad)

        # Centre is unchanged regardless of bbox — always image centre
        ax, ay = self.module.anchor_pixel(5, 1920, 1080,
                                          bbox_half_w=100, bbox_half_h=75)
        self.assertAlmostEqual(ax, 1920 / 2)
        self.assertAlmostEqual(ay, 1080 / 2)

    def test_build_anchor_k_center_preserves_principal_point(self):
        k = np.array([[800, 0, 960], [0, 800, 540], [0, 0, 1]], dtype=np.float64)
        dist = np.zeros(5)
        anchor_k = self.module.build_anchor_k(k, dist, 5, 1920, 1080)
        np.testing.assert_array_almost_equal(anchor_k, k)

    def test_build_anchor_k_with_bbox_insets_from_edge(self):
        k = np.array([[800, 0, 960], [0, 800, 540], [0, 0, 1]], dtype=np.float64)
        dist = np.zeros(5)
        pad = self.module.ANCHOR_PADDING_PX
        # Anchor 8 (TC) with bbox 200px tall (half_h=100)
        # inset_y = 100 + pad
        anchor_k = self.module.build_anchor_k(k, dist, 8, 1920, 1080,
                                              bbox_half_w=0, bbox_half_h=100)
        self.assertAlmostEqual(float(anchor_k[0, 2]), 960.0)
        self.assertAlmostEqual(float(anchor_k[1, 2]), 100 + pad)

    def test_preset_vehicles_selects_yolov8s_and_class_filter(self):
        with patch.object(sys, "argv", ["run_gimbal_tracker.py", "--preset", "vehicles"]):
            args = self.module.parse_args()
        model_file, classes = self.module.DETECTOR_PRESETS[args.preset]
        self.assertEqual(model_file, "yolov8s.pt")
        self.assertEqual(classes, [2])

    def test_preset_default(self):
        with patch.object(sys, "argv", ["run_gimbal_tracker.py"]):
            args = self.module.parse_args()
        self.assertIn(args.preset, self.module.DETECTOR_PRESETS)

    def test_runner_has_no_zoom_threshold_override(self):
        with patch.object(
            sys,
            "argv",
            ["run_gimbal_tracker.py", "--target-pixels", "360"],
        ):
            with self.assertRaises(SystemExit):
                self.module.parse_args()

    def test_build_deep_search_config_uses_profile_settings(self):
        args = Mock()
        args.deep_search = None
        args.deep_search_hz = None
        args.deep_search_imgsz = None
        args.deep_search_conf = None
        detector_settings = {
            "deep_search": {
                "enabled": True,
                "hz": 3.0,
                "imgsz": 960,
                "conf": 0.12,
            }
        }

        config = self.module.build_deep_search_config(args, detector_settings, "model.pt", [2])

        self.assertTrue(config["enabled"])
        self.assertEqual(config["hz"], 3.0)
        self.assertEqual(config["imgsz"], 960)
        self.assertEqual(config["conf"], 0.12)
        self.assertEqual(config["model_path"], "model.pt")
        self.assertEqual(config["classes"], [2])

    def test_build_deep_search_config_allows_cli_overrides(self):
        args = Mock()
        args.deep_search = True
        args.deep_search_hz = 5.0
        args.deep_search_imgsz = 1280
        args.deep_search_conf = 0.08
        detector_settings = {}

        config = self.module.build_deep_search_config(args, detector_settings, "model.pt", [2])

        self.assertTrue(config["enabled"])
        self.assertEqual(config["hz"], 5.0)
        self.assertEqual(config["imgsz"], 1280)
        self.assertEqual(config["conf"], 0.08)

    def test_build_deep_search_config_can_disable_profile(self):
        args = Mock()
        args.deep_search = False
        args.deep_search_hz = None
        args.deep_search_imgsz = None
        args.deep_search_conf = None
        detector_settings = {"deep_search": {"enabled": True}}

        self.assertIsNone(self.module.build_deep_search_config(args, detector_settings, "model.pt", [2]))

    def test_build_tracker_config_uses_profile_settings(self):
        args = Mock()
        args.tracker_backend = None
        detector_settings = {"tracker": {"backend": "botsort", "max_age": 90}}

        config = self.module.build_tracker_config(args, detector_settings)

        self.assertEqual(config["backend"], "botsort")
        self.assertEqual(config["max_age"], 90)

    def test_build_tracker_config_allows_cli_override(self):
        args = Mock()
        args.tracker_backend = "strongsort"
        detector_settings = {"tracker": {"backend": "botsort", "max_age": 90}}

        config = self.module.build_tracker_config(args, detector_settings)

        self.assertEqual(config["backend"], "strongsort")
        self.assertEqual(config["max_age"], 90)

    def test_build_tracker_config_returns_none_when_unconfigured(self):
        args = Mock()
        args.tracker_backend = None

        self.assertIsNone(self.module.build_tracker_config(args, {}))

    def test_profile_rates_remain_canonical_without_explicit_overrides(self):
        args = SimpleNamespace(
            detect_hz=None,
            track_hz=None,
            deep_search=None,
            deep_search_hz=None,
            deep_search_imgsz=None,
            deep_search_conf=None,
            tracker_backend=None,
        )
        settings = {
            "detect_hz": 30.0,
            "track_hz": 120.0,
            "tracker": {"backend": "botsort", "frame_rate": 30},
            "deep_search": {"enabled": True, "hz": 2.0},
        }

        resolved = self.module.apply_detector_overrides(args, settings)

        self.assertEqual(resolved, settings)

    def test_explicit_deep_search_and_tracker_overrides_win(self):
        args = SimpleNamespace(
            detect_hz=None,
            track_hz=None,
            deep_search=False,
            deep_search_hz=3.0,
            deep_search_imgsz=1280,
            deep_search_conf=0.1,
            tracker_backend="strongsort",
        )
        settings = {
            "detect_hz": 30.0,
            "track_hz": 120.0,
            "tracker": {"backend": "botsort", "frame_rate": 30},
            "deep_search": {"enabled": True, "hz": 2.0},
        }

        resolved = self.module.apply_detector_overrides(args, settings)

        self.assertEqual(resolved["tracker"]["backend"], "strongsort")
        self.assertEqual(
            resolved["deep_search"],
            {
                "enabled": False,
                "hz": 3.0,
                "imgsz": 1280,
                "conf": 0.1,
            },
        )

    def test_explicit_rate_overrides_win(self):
        args = SimpleNamespace(
            detect_hz=12.0,
            track_hz=30.0,
            deep_search=None,
            deep_search_hz=None,
            deep_search_imgsz=None,
            deep_search_conf=None,
            tracker_backend=None,
        )
        settings = {"detect_hz": 30.0, "track_hz": 60.0, "tracker": {"frame_rate": 30}}

        resolved = self.module.apply_detector_overrides(args, settings)

        self.assertEqual(resolved["detect_hz"], 12.0)
        self.assertEqual(resolved["track_hz"], 30.0)
        self.assertEqual(resolved["tracker"]["frame_rate"], 12)

    def test_navigation_uses_canonical_profile_tracking_gains(self):
        args = SimpleNamespace(max_rate=None, pitch=15.0)
        tracking_setup = GimbalTrackingSetup(
            GimbalRateTrackerConfig(correction_bw=1.75, max_rate=50.0),
            GimbalLossPolicy(repoint_sec=3.0),
        )
        zoom_config = TargetZoomTrackerConfig(
            target_pixels={"0": 36.0, "default": 20.0}
        )
        with patch.object(runner_assembly, "GimbalNavigation") as navigation_cls:
            runner_assembly.build_navigation(
                args,
                Mock(),
                Mock(),
                tracking_setup,
                zoom_config,
            )

        tracking = navigation_cls.call_args.kwargs["tracking"]
        self.assertAlmostEqual(tracking.rate.correction_bw, 1.75)
        self.assertEqual(tracking.rate.max_rate, 50.0)
        self.assertEqual(tracking.loss.repoint_sec, 3.0)
        configured_zoom = navigation_cls.call_args.kwargs["zoom_config"]
        self.assertIs(configured_zoom, zoom_config)
        self.assertEqual(
            tuple(configured_zoom.__dataclass_fields__),
            ("target_pixels",),
        )

    def test_detector_preserves_profile_appearance_configuration(self):
        args = SimpleNamespace(
            conf=None,
            device=None,
            deep_search=None,
            deep_search_hz=None,
            deep_search_imgsz=None,
            deep_search_conf=None,
            tracker_backend=None,
        )
        settings = {
            "appearance": {"enabled": True, "reid_device": "auto"},
        }
        with patch.object(runner_assembly, "Detector") as detector_cls:
            runner_assembly.build_detector(
                args,
                Mock(),
                Mock(),
                settings,
                "model.pt",
                [2],
                "rtsp://source",
            )
        config = detector_cls.call_args.args[1]
        self.assertEqual(
            config.model.appearance,
            {"enabled": True, "reid_device": "auto"},
        )
        self.assertFalse(config.pipeline.auto_target_lock)

    def test_operator_zoom_only_sets_initial_hardware_zoom(self):
        from navpy.modules.common.models.attitude import Attitude
        from navpy.modules.vision.peripheral.gimbal_abc import GimbalData

        gimbal = Mock()
        gimbal.is_connected.return_value = True
        gimbal.get_data.return_value = GimbalData(
            att=Attitude(0.0, 0.0, 0.0)
        )
        mount = Mock()
        mount.gimbal = gimbal
        args = SimpleNamespace(pitch=15.0, yaw=0.0, zoom=5.0)

        with patch.object(runner_assembly.time, "sleep"):
            runner_assembly.initialize_gimbal(args, mount, Mock())

        mount.set_zoom.assert_called_once_with("5.0")


class TestParseCmd(unittest.TestCase):
    """_parse_cmd: stdin command → (name, arg) tuple."""

    def test_n_no_arg(self):
        self.assertEqual(runner_commands.parse_command("n"), ("n", None))

    def test_f_no_arg(self):
        self.assertEqual(runner_commands.parse_command("f"), ("f", None))

    def test_n_with_extra_arg_rejected(self):
        # Bare commands take no arg; extra tokens flag a typo, not silent ignore.
        self.assertEqual(runner_commands.parse_command("n 1"), (None, None))

    def test_z_with_zoom_level(self):
        self.assertEqual(runner_commands.parse_command("z 3"), ("z", "3"))

    def test_z_keeps_arg_as_string(self):
        # Mount.set_zoom accepts str|float; let dispatcher convert.
        self.assertEqual(runner_commands.parse_command("z 1.5"), ("z", "1.5"))

    def test_z_without_arg_rejected(self):
        self.assertEqual(runner_commands.parse_command("z"), (None, None))

    def test_p_with_anchor(self):
        self.assertEqual(runner_commands.parse_command("p 7"), ("p", "7"))

    def test_p_without_arg_rejected(self):
        self.assertEqual(runner_commands.parse_command("p"), (None, None))

    def test_unknown_command_returns_none(self):
        self.assertEqual(runner_commands.parse_command("x"), (None, None))
        self.assertEqual(runner_commands.parse_command("quit"), (None, None))

    def test_empty_string_returns_none(self):
        self.assertEqual(runner_commands.parse_command(""), (None, None))
        self.assertEqual(runner_commands.parse_command("   "), (None, None))

    def test_leading_trailing_whitespace_ignored(self):
        self.assertEqual(runner_commands.parse_command("  z 5  "), ("z", "5"))

    def test_case_insensitive_command_name(self):
        # Operator on PuTTY may have caps lock on by accident; tolerate.
        self.assertEqual(runner_commands.parse_command("N"), ("n", None))
        self.assertEqual(runner_commands.parse_command("Z 4"), ("z", "4"))


class TestPickNextId(unittest.TestCase):
    """_pick_next_id: cycle through track ids by current locked id."""

    def test_empty_list_returns_none(self):
        self.assertIsNone(runner_commands.pick_next_id([], None))
        self.assertIsNone(runner_commands.pick_next_id([], 5))

    def test_no_current_picks_first(self):
        self.assertEqual(runner_commands.pick_next_id([3, 7, 9], None), 3)

    def test_cycle_to_next(self):
        self.assertEqual(runner_commands.pick_next_id([3, 7, 9], 3), 7)
        self.assertEqual(runner_commands.pick_next_id([3, 7, 9], 7), 9)

    def test_wraps_at_end(self):
        self.assertEqual(runner_commands.pick_next_id([3, 7, 9], 9), 3)

    def test_current_not_in_list_picks_first(self):
        # Target lost, list rebuilt with new ids → start fresh.
        self.assertEqual(runner_commands.pick_next_id([3, 7, 9], 42), 3)

    def test_single_item_returns_itself(self):
        # Cycle wraps to the only id.
        self.assertEqual(runner_commands.pick_next_id([5], 5), 5)


if __name__ == "__main__":
    unittest.main()
