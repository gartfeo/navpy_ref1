"""Tests for vision_profiles module."""
import math
import unittest
from collections.abc import Mapping
from pathlib import Path
from unittest.mock import Mock

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision import vision_profiles
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class TestVisionProfilesFacade(unittest.TestCase):
    """The compatibility module must expose the canonical leaf objects."""

    def test_reexports_exact_profile_symbols(self):
        from navpy.modules.vision.vision_camera_factory import build_camera_mounts
        from navpy.modules.vision.vision_profile_loader import (
            _resolve_profile_definitions,
            load_profiles,
        )
        from navpy.modules.vision.vision_profile_types import CameraMountSpec
        from navpy.modules.vision.vision_class_profile import (
            compute_detect_slant_range,
            get_min_pixels_for_class,
        )
        from navpy.modules.vision.vision_tracking_composition import (
            build_tracking_config,
        )

        self.assertIs(vision_profiles.CameraMountSpec, CameraMountSpec)
        self.assertIs(vision_profiles.build_camera_mounts, build_camera_mounts)
        self.assertIs(vision_profiles.load_profiles, load_profiles)
        self.assertIs(
            vision_profiles._resolve_profile_definitions,
            _resolve_profile_definitions,
        )
        self.assertIs(
            vision_profiles.compute_detect_slant_range,
            compute_detect_slant_range,
        )
        self.assertIs(
            vision_profiles.get_min_pixels_for_class,
            get_min_pixels_for_class,
        )
        self.assertIs(vision_profiles.build_tracking_config, build_tracking_config)


class TestLoadProfiles(unittest.TestCase):
    """Tests for load_profiles function."""

    def test_load_profiles_returns_dict(self):
        """load_profiles returns profiles dict."""
        profiles, default, path = vision_profiles.load_profiles()
        self.assertIsInstance(profiles, dict)
        self.assertTrue(len(profiles) > 0)

    def test_load_profiles_returns_default_profile_name(self):
        """load_profiles returns default profile name."""
        profiles, default, path = vision_profiles.load_profiles()
        self.assertIsInstance(default, str)

    def test_load_profiles_returns_path(self):
        """load_profiles returns JSON file path."""
        profiles, default, path = vision_profiles.load_profiles()
        self.assertIsInstance(path, Path)
        self.assertTrue(path.exists())


class TestResolveProfile(unittest.TestCase):
    """Tests for resolve_profile function."""

    def setUp(self):
        self.logger = Mock()

    def test_resolve_profile_by_name(self):
        """resolve_profile returns named profile."""
        profiles, default, _ = vision_profiles.load_profiles()
        name = next(iter(profiles.keys()))

        key, profile, path = vision_profiles.resolve_profile(name, self.logger)

        self.assertEqual(key, name)
        self.assertIsInstance(profile, dict)

    def test_resolve_profile_uses_default(self):
        """resolve_profile uses default when name is empty."""
        key, profile, path = vision_profiles.resolve_profile("", self.logger)

        self.assertIsInstance(key, str)
        self.assertIsInstance(profile, dict)
        self.logger.info.assert_called()

    def test_resolve_novoxy18_profile(self):
        """resolve_profile returns novoxy18 single-device profile."""
        key, profile, path = vision_profiles.resolve_profile("novoxy18", self.logger)

        self.assertEqual(key, "novoxy18")
        devices = vision_profiles.get_devices(profile)
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0]["name"], "novoxy_18_down")
        self.assertLess(devices[0]["gimbal"]["camera_pitch"], 0)  # downward-looking

    def test_resolve_novoxy10_profile(self):
        """resolve_profile returns novoxy10 single-device profile."""
        key, profile, path = vision_profiles.resolve_profile("novoxy10", self.logger)

        self.assertEqual(key, "novoxy10")
        devices = vision_profiles.get_devices(profile)
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0]["name"], "novoxy_10_straight")
        self.assertLess(devices[0]["gimbal"]["camera_pitch"], 0)  # downward-looking

    def test_siyi_horizon_profile_only_overrides_static_camera_pitch(self):
        """The co-alt setup inherits SIYI calibration without changing ground."""
        profiles, _, _ = vision_profiles.load_profiles()
        ground = profiles["siyi_zr10"]
        horizon = profiles["siyi_zr10_horizon"]
        key, resolved_horizon, _ = vision_profiles.resolve_profile(
            "siyi_zr10_horizon", self.logger
        )

        self.assertEqual(key, "siyi_zr10_horizon")
        self.assertEqual(resolved_horizon, horizon)
        self.assertEqual(horizon["detector"], ground["detector"])
        self.assertEqual(
            horizon["devices"][0]["camera"], ground["devices"][0]["camera"]
        )

        ground_gimbal = ground["devices"][0]["gimbal"]
        horizon_gimbal = horizon["devices"][0]["gimbal"]
        self.assertEqual(ground_gimbal["camera_pitch"], -14.0)
        self.assertEqual(horizon_gimbal["camera_pitch"], 0.0)
        self.assertEqual(
            {k: v for k, v in horizon_gimbal.items() if k != "camera_pitch"},
            {k: v for k, v in ground_gimbal.items() if k != "camera_pitch"},
        )

    def test_default_siyi_profile_uses_ground_air_camera_pitch(self):
        """The existing operator profile uses the unified ground/air angle."""
        profiles, default, _ = vision_profiles.load_profiles()

        self.assertEqual(default, "siyi_zr10")
        self.assertEqual(
            profiles["siyi_zr10"]["devices"][0]["gimbal"]["camera_pitch"],
            -14.0,
        )

    def test_profile_inheritance_rejects_cycles(self):
        with self.assertRaisesRegex(ValueError, "inheritance cycle"):
            vision_profiles._resolve_profile_definitions(
                {
                    "profile_a": {"extends": "profile_b"},
                    "profile_b": {"extends": "profile_a"},
                }
            )

    def test_profile_device_override_requires_named_base_device(self):
        with self.assertRaisesRegex(ValueError, "overrides unknown device"):
            vision_profiles._resolve_profile_definitions(
                {
                    "base": {"devices": [{"name": "camera_a"}]},
                    "variant": {
                        "extends": "base",
                        "device_overrides": {"camera_b": {"gimbal": {}}},
                    },
                }
            )

    def test_profile_devices_reject_non_mapping_entries(self):
        with self.assertRaisesRegex(ValueError, "entries must be JSON objects"):
            vision_profiles._resolve_profile_definitions(
                {"invalid": {"devices": ["camera_a"]}}
            )

    def test_resolve_profile_invalid_name_raises(self):
        """resolve_profile raises ValueError for unknown profile."""
        with self.assertRaises(ValueError) as ctx:
            vision_profiles.resolve_profile("nonexistent_profile_xyz", self.logger)

        self.assertIn("not found", str(ctx.exception))
        self.logger.error.assert_called()


class TestGetDevices(unittest.TestCase):
    """Tests for get_devices function."""

    def test_get_devices_returns_list(self):
        """get_devices returns devices list from profile."""
        profile = {"devices": [{"name": "cam1"}, {"name": "cam2"}]}
        devices = vision_profiles.get_devices(profile)

        self.assertEqual(len(devices), 2)
        self.assertEqual(devices[0]["name"], "cam1")

    def test_get_devices_empty_profile(self):
        """get_devices returns empty list for profile without devices."""
        profile = {}
        devices = vision_profiles.get_devices(profile)

        self.assertEqual(devices, [])

    def test_get_devices_invalid_type_raises(self):
        """get_devices raises ValueError when devices is not a list."""
        profile = {"devices": "not_a_list"}

        with self.assertRaises(ValueError) as ctx:
            vision_profiles.get_devices(profile)

        self.assertIn("must be a list", str(ctx.exception))

    def test_get_devices_rejects_non_mapping_entries(self):
        with self.assertRaisesRegex(ValueError, "entries must be JSON objects"):
            vision_profiles.get_devices({"devices": [None]})


class TestGetDetectorSettings(unittest.TestCase):
    """Tests for get_detector_settings function."""

    def test_get_detector_settings_defaults(self):
        """get_detector_settings returns defaults when not specified."""
        profile = {}
        settings = vision_profiles.get_detector_settings(profile)

        self.assertEqual(settings["detect_hz"], 15.0)
        self.assertEqual(settings["conf"], 0.35)
        self.assertEqual(settings["imgsz"], 640)
        self.assertFalse(settings["ideal_360"])

    def test_get_detector_settings_override(self):
        """get_detector_settings overrides defaults with profile values."""
        profile = {
            "detector": {
                "detect_hz": 30.0,
                "conf": 0.5,
            }
        }
        settings = vision_profiles.get_detector_settings(profile)

        self.assertEqual(settings["detect_hz"], 30.0)
        self.assertEqual(settings["conf"], 0.5)
        # Defaults still present
        self.assertEqual(settings["imgsz"], 640)
        self.assertFalse(settings["ideal_360"])

    def test_ideal_360_profile_enables_sim_ideal_sensor(self):
        """ideal_360 profile opts into the sim-only static infinite-FOV sensor."""
        _, profile, _ = vision_profiles.resolve_profile("ideal_360", Mock())

        settings = vision_profiles.get_detector_settings(profile)
        devices = vision_profiles.get_devices(profile)

        self.assertTrue(settings["ideal_360"])
        self.assertNotIn("loop_rate_hz", settings)
        for unused_key in ("reference_height_m", "imgsz", "conf"):
            self.assertNotIn(unused_key, profile["detector"])
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0]["name"], "ideal_360")
        self.assertNotIn("tracking", devices[0]["gimbal"])
        self.assertNotIn("zoom", devices[0]["gimbal"])

    def test_get_detector_settings_rejects_invalid_detector(self):
        profile = {"detector": "invalid"}
        with self.assertRaisesRegex(ValueError, "detector must be a mapping"):
            vision_profiles.get_detector_settings(profile)

    def test_siyi_profile_enables_gpu_deep_search(self):
        _, profile, _ = vision_profiles.resolve_profile("siyi_zr10", Mock())

        settings = vision_profiles.get_detector_settings(profile)

        self.assertEqual(settings["device"], "auto")
        self.assertEqual(settings["detect_hz"], 30.0)
        self.assertEqual(settings["track_hz"], 60.0)
        self.assertAlmostEqual(settings["conf"], 0.2)
        self.assertEqual(settings["tracker"]["backend"], "botsort")
        self.assertEqual(settings["tracker"]["max_age"], 90)
        self.assertEqual(settings["tracker"]["reid_device"], "auto")
        # Live tracking must keep the high-rate MOT path light. Appearance ReID
        # runs on demand around the selected lock/reacquire path instead.
        # Tracker-internal ReID stays OFF (it would block the 60Hz track loop);
        # scene-wide appearance id stability is provided by the ASYNC gallery
        # below, using VEHICLE CLIP-ReID weights.
        self.assertFalse(settings["tracker"]["with_reid"])
        self.assertEqual(settings["tracker"]["reid_weights"], "clip_veri.pt")
        # Camera-motion compensation for the gimbal (sparse optical flow).
        self.assertEqual(settings["tracker"]["cmc_method"], "sof")
        self.assertTrue(settings["appearance"]["enabled"])
        self.assertEqual(settings["appearance"]["reid_device"], "auto")
        self.assertEqual(settings["appearance"]["weights"], "clip_veri.pt")
        self.assertTrue(settings["deep_search"]["enabled"])
        self.assertEqual(settings["deep_search"]["imgsz"], 960)
        self.assertEqual(settings["deep_search"]["hz"], 2.0)


class TestBuildGimbalData(unittest.TestCase):
    """Tests for build_gimbal_data function."""

    def test_build_gimbal_data_basic(self):
        """build_gimbal_data creates GimbalData from device config."""
        device = {
            "gimbal": {
                "camera_pitch": -45,
                "stabilize_roll": False,
                "stabilize_pitch": True,
                "seq": "XYZ",
                "setup_att": [90, 0, 90],
                "setup_seq": "ZYX",
                "setup_dist_mm": [100, 200, 300],
            }
        }

        data = vision_profiles.build_gimbal_data(device, 0)

        self.assertIsInstance(data, GimbalData)
        self.assertEqual(data.att.pitch, -45)
        self.assertFalse(data.roll_stabilize)
        self.assertTrue(data.pitch_stabilize)
        self.assertEqual(data.g_seq, "XYZ")
        self.assertEqual(data.setup_att.pitch, 90)
        self.assertEqual(data.setup_seq, "ZYX")
        self.assertEqual(data.setup_dist, [0.1, 0.2, 0.3])  # Converted to meters
        self.assertEqual(data.max_detect_distance, 3000.0)  # fallback, no camera

    def test_build_gimbal_data_no_gimbal_returns_none(self):
        """build_gimbal_data returns None when no gimbal config."""
        device = {"camera": {}}
        data = vision_profiles.build_gimbal_data(device, 0)

        self.assertIsNone(data)

    def test_build_gimbal_data_invalid_gimbal_returns_none(self):
        """build_gimbal_data returns None when gimbal is not dict."""
        device = {"gimbal": "invalid"}
        data = vision_profiles.build_gimbal_data(device, 0)

        self.assertIsNone(data)

    def test_build_gimbal_data_sets_name_with_index(self):
        """build_gimbal_data sets name with index."""
        device = {
            "gimbal": {
                "camera_pitch": 0,
                "setup_att": [0, 0, 0],
            }
        }

        data = vision_profiles.build_gimbal_data(device, 5)

        self.assertEqual(data.name, "gimbal_5")

    def test_build_gimbal_data_defaults_missing_setup_att(self):
        """build_gimbal_data uses the catalog setup_att default."""
        device = {"gimbal": {"camera_pitch": -15}}

        data = vision_profiles.build_gimbal_data(device, 0)

        self.assertEqual(data.setup_att.pitch, 90)
        self.assertEqual(data.setup_att.yaw, 0)
        self.assertEqual(data.setup_att.roll, 90)
        self.assertEqual(data.setup_seq, "XYZ")
        self.assertEqual(data.g_seq, "XYZ")

    def test_resolve_gimbal_device_id_rejects_float(self):
        """gimbal_device_id values must be explicit integers."""
        with self.assertRaisesRegex(ValueError, "must be an integer"):
            vision_profiles.resolve_gimbal_device_id({"gimbal_device_id": 1.9}, 0)

    def test_resolve_gimbal_device_id_rejects_bool(self):
        """bool is not accepted as a MAVLink device ID."""
        with self.assertRaisesRegex(ValueError, "must be an integer"):
            vision_profiles.resolve_gimbal_device_id({"gimbal_device_id": True}, 0)

    def test_setup_vectors_require_exactly_three_finite_values(self):
        for setup_att in ([1, 2], [1, 2, 3, 4], [1, math.nan, 3]):
            with self.subTest(setup_att=setup_att):
                with self.assertRaisesRegex(ValueError, "exactly 3|finite"):
                    vision_profiles.resolve_gimbal_setup_att(
                        {"setup_att": setup_att}
                    )

    def test_setup_distance_requires_exactly_three_finite_values(self):
        for setup_dist in ([1, 2], [1, math.inf, 3]):
            with self.subTest(setup_dist=setup_dist):
                with self.assertRaisesRegex(ValueError, "exactly 3|finite"):
                    vision_profiles.build_gimbal_data(
                        {"gimbal": {"setup_dist_mm": setup_dist}},
                        0,
                    )

    def test_rotation_sequences_must_be_known(self):
        with self.assertRaisesRegex(ValueError, "must be one of"):
            vision_profiles.resolve_gimbal_seq({"seq": "ABC"})

class TestComputeMaxDetectDist(unittest.TestCase):
    """Tests for compute_max_detect_dist."""

    def test_known_values(self):
        """compute_max_detect_dist returns detection range with 10% margin."""
        # Formula: fy * _MAX_CLASS_SIZE / MIN_DETECT_PIXELS * 1.1
        # _MAX_CLASS_SIZE = sqrt(4.5^2 + 1.5^2) = sqrt(22.5) (Car diagonal)
        _max_size = math.sqrt(22.5)
        expected = 2000.0 * _max_size / 8 * 1.1
        result = vision_profiles.compute_max_detect_dist(fy=2000)
        self.assertAlmostEqual(result, expected, places=1)

    def test_higher_fy_increases_range(self):
        """Higher fy (more zoom) produces larger confirmation range."""
        d1 = vision_profiles.compute_max_detect_dist(2000)
        d2 = vision_profiles.compute_max_detect_dist(4000)
        self.assertAlmostEqual(d2, d1 * 2, places=2)

    def test_build_gimbal_data_computes_from_optics(self):
        """build_gimbal_data computes max_detect_distance from camera optics."""
        device = {
            "camera": {
                "image_width": 1920,
                "image_height": 1080,
                "intrinsics": {
                    "zooms": {
                        "1": {"fx": 2000.0, "fy": 2000.0, "cx": 960, "cy": 540}
                    }
                }
            },
            "gimbal": {
                "camera_pitch": -15,
                "setup_att": [90, 0, 90],
            }
        }
        det_settings = {"reference_height_m": 2.0, "imgsz": 640}

        data = vision_profiles.build_gimbal_data(device, 0, det_settings)

        # Formula: fy * _MAX_CLASS_SIZE / MIN_DETECT_PIXELS * 1.1
        # _MAX_CLASS_SIZE = sqrt(22.5) (Car: 4.5m x 1.5m diagonal)
        expected = 2000.0 * math.sqrt(22.5) / 8 * 1.1
        self.assertAlmostEqual(data.max_detect_distance, expected, places=1)

    def test_build_gimbal_data_fallback_without_detector_settings(self):
        """build_gimbal_data uses 3000.0 fallback without detector_settings."""
        device = {
            "camera": {
                "image_width": 1920,
                "image_height": 1080,
                "intrinsics": {
                    "zooms": {
                        "1": {"fx": 2000.0, "fy": 2000.0, "cx": 960, "cy": 540}
                    }
                }
            },
            "gimbal": {
                "camera_pitch": -15,
                "setup_att": [90, 0, 90],
            }
        }

        data = vision_profiles.build_gimbal_data(device, 0)

        self.assertEqual(data.max_detect_distance, 3000.0)

    def test_c720hd_profile_computed_distance(self):
        """Real c720hd profile computes expected max_detect_distance."""
        _, profile, _ = vision_profiles.resolve_profile("c720hd", Mock())
        det_settings = vision_profiles.get_detector_settings(profile)
        devices = vision_profiles.get_devices(profile)
        device = devices[0]

        data = vision_profiles.build_gimbal_data(device, 0, det_settings)

        # Formula: fy * _MAX_CLASS_SIZE / MIN_DETECT_PIXELS * 1.1
        # _MAX_CLASS_SIZE = sqrt(22.5) (Car diagonal)
        expected = 1439.8475316765432 * math.sqrt(22.5) / 8 * 1.1
        self.assertAlmostEqual(data.max_detect_distance, expected, places=1)


class TestComputeExplicitSlantRanges(unittest.TestCase):
    """Tests for explicit detect/confirm slant-range helpers."""

    def test_compute_detect_slant_range_uses_raw_detector_gate(self):
        """Raw detect range uses class size and MIN_DETECT_PIXELS."""
        result = vision_profiles.compute_detect_slant_range(fy=2000, class_size=2.96)
        self.assertAlmostEqual(result, 2000.0 * 2.96 / vision_profiles.MIN_DETECT_PIXELS, places=1)

    def test_compute_confirm_slant_range_uses_confirmation_gate(self):
        """Confirmation range uses _MIN_CLASS_SIZE (Person diagonal) and MIN_CONFIRM_PIXELS."""
        # _MIN_CLASS_SIZE = sqrt(0.5^2 + 1.8^2) = sqrt(3.49) ≈ 1.868 (Person diagonal)
        _min_size = math.sqrt(0.5**2 + 1.8**2)
        expected = 2000.0 * _min_size / vision_profiles.MIN_CONFIRM_PIXELS
        result = vision_profiles.compute_confirm_slant_range(fy=2000)
        self.assertAlmostEqual(result, expected, places=4)

    def test_detect_range_exceeds_confirm_range_for_same_poi(self):
        """Raw detection range is larger than confirmation range for the same POI class."""
        detect = vision_profiles.compute_detect_slant_range(fy=2000, class_size=2.96)
        confirm = vision_profiles.compute_confirm_slant_range(fy=2000, class_size=2.96)
        self.assertGreater(detect, confirm)

    def test_range_math_rejects_nonfinite_or_nonpositive_inputs(self):
        invalid_cases = (
            (math.nan, 2.0, 8.0),
            (math.inf, 2.0, 8.0),
            (1000.0, -2.0, 8.0),
            (1000.0, 2.0, 0.0),
        )
        for fy, class_size, pixels in invalid_cases:
            with self.subTest(fy=fy, class_size=class_size, pixels=pixels):
                with self.assertRaisesRegex(ValueError, "positive finite"):
                    vision_profiles.compute_detect_slant_range(
                        fy,
                        class_size,
                        pixels,
                    )


class TestComputeApproachInterval(unittest.TestCase):
    """Tests for compute_approach_interval function."""

    def test_returns_interval_for_valid_config(self):
        """Steep camera with large fy returns a valid interval."""
        # fy=3000, cy=540, ih=1080, pitch=30, alt=200, Medium=2.96
        result = vision_profiles.compute_approach_interval(3000, 540, 1080, 30, 200, 2.96)
        self.assertIsNotNone(result)
        g_min, g_max = result
        self.assertGreater(g_max, g_min)
        self.assertGreater(g_min, 0)

    def test_returns_none_when_slant_below_alt(self):
        """When confirm slant < altitude, returns None."""
        result = vision_profiles.compute_approach_interval(500, 540, 1080, 15, 500, 1.80)
        self.assertIsNone(result)

    def test_returns_none_when_no_overlap(self):
        """Shallow pitch at high alt — confirm min exceeds confirm max."""
        result = vision_profiles.compute_approach_interval(1440, 337, 720, 5, 200, 1.80)
        self.assertIsNone(result)

    def test_steeper_pitch_widens_interval(self):
        """Steeper camera gives wider interval (lower g_confirm_min)."""
        r_shallow = vision_profiles.compute_approach_interval(3000, 540, 1080, 15, 200, 2.96)
        r_steep = vision_profiles.compute_approach_interval(3000, 540, 1080, 30, 200, 2.96)
        self.assertIsNotNone(r_steep)
        if r_shallow is not None:
            self.assertGreater(r_steep[1] - r_steep[0], r_shallow[1] - r_shallow[0])

    def test_cy_below_budget_line_gives_zero_g_min(self):
        """When cy > max_y_pct * img_h, g_confirm_min is 0 (always within budget)."""
        # cy=900 in 1080px image: 0.80*1080=864 < 900 → px_from_cy < 0 → g_min=0
        result = vision_profiles.compute_approach_interval(3000, 900, 1080, 30, 100, 2.96)
        if result is not None:
            g_min, g_max = result
            self.assertEqual(g_min, 0.0)


class TestBuildTrackingConfig(unittest.TestCase):
    """Tests for build_tracking_config function."""

    def test_siyi_zr10_returns_config(self):
        """siyi_zr10 profile has tracking config with correct values."""
        _, profile, _ = vision_profiles.resolve_profile("siyi_zr10", Mock())
        devices = vision_profiles.get_devices(profile)
        device = devices[0]
        config = vision_profiles.build_tracking_config(device, sim=True)

        self.assertIsNotNone(config)
        # siyi_zr10 uses correction_bw=2.5 (the library default). #186 raised it
        # to 5.0 for orbit-lag, but 5.0 overshoots so the POI leaves frame
        # during the confirm orbit; reverted to 2.5 as part of the confirm-loss
        # ladder fix.
        self.assertAlmostEqual(config.rate.correction_bw, 2.5)
        self.assertEqual(config.rate.max_rate, 100)
        self.assertAlmostEqual(config.loss.repoint_sec, 2.0)
        self.assertFalse(hasattr(config, "neutral_pitch"))
        self.assertAlmostEqual(config.rate.command_lead_time, 0.0)

    def test_loss_ladder_keys_flow_through_allowlist(self):
        """loss_hold_sec / loss_repoint_sec / preserve_zoom_during_loss are
        profile-overridable via the tracking allowlist."""
        device = {"gimbal": {"camera_pitch": -20.0, "tracking": {
            "enabled": True,
            "loss_hold_sec": 0.4,
            "loss_repoint_sec": 1.8,
            "preserve_zoom_during_loss": False,
        }}}
        config = vision_profiles.build_tracking_config(device, sim=True)
        self.assertIsNotNone(config)
        self.assertAlmostEqual(config.loss.hold_sec, 0.4)
        self.assertAlmostEqual(config.loss.repoint_sec, 1.8)
        self.assertFalse(config.loss.preserve_zoom_during_loss)

    def test_loss_ladder_defaults_when_not_overridden(self):
        """Without overrides the loss-ladder boundaries keep library defaults."""
        device = {"gimbal": {"camera_pitch": -20.0, "tracking": {"enabled": True}}}
        config = vision_profiles.build_tracking_config(device, sim=True)
        self.assertAlmostEqual(config.loss.hold_sec, 0.5)
        self.assertAlmostEqual(config.loss.repoint_sec, 2.0)
        self.assertTrue(config.loss.preserve_zoom_during_loss)

    def test_c720hd_returns_none(self):
        """c720hd profile has no tracking config."""
        _, profile, _ = vision_profiles.resolve_profile("c720hd", Mock())
        devices = vision_profiles.get_devices(profile)
        config = vision_profiles.build_tracking_config(devices[0], sim=True)

        self.assertIsNone(config)

    def test_device_without_gimbal_returns_none(self):
        """Device with no gimbal section returns None."""
        config = vision_profiles.build_tracking_config({"camera": {}}, sim=True)
        self.assertIsNone(config)

    def test_tracking_disabled_returns_none(self):
        """When tracking.enabled is false, returns None."""
        device = {"gimbal": {"tracking": {"enabled": False}}}
        config = vision_profiles.build_tracking_config(device, sim=True)
        self.assertIsNone(config)

    def test_tracking_enabled_must_be_boolean(self):
        for enabled in (1, 0, "true", "false", None):
            with self.subTest(enabled=enabled):
                device = {"gimbal": {"tracking": {"enabled": enabled}}}
                with self.assertRaisesRegex(ValueError, "enabled must be boolean"):
                    vision_profiles.build_tracking_config(device, sim=True)

    def test_preserve_zoom_during_loss_must_be_boolean(self):
        for preserve in (1, 0, "true", "false", None):
            with self.subTest(preserve=preserve):
                device = {"gimbal": {"tracking": {
                    "enabled": True,
                    "preserve_zoom_during_loss": preserve,
                }}}
                with self.assertRaisesRegex(
                    ValueError,
                    "preserve_zoom_during_loss must be boolean",
                ):
                    vision_profiles.build_tracking_config(device, sim=True)

    def test_unknown_tracking_keys_are_rejected(self):
        for key in (
            "min_pixels",
            "rate_clamp_slew_multiple",
            "target_band_ratio",
            "tracking_pixels",
        ):
            with self.subTest(key=key):
                device = {
                    "gimbal": {
                        "tracking": {"enabled": True, key: 20},
                    },
                }
                with self.assertRaisesRegex(
                    ValueError,
                    "Unknown gimbal.tracking key",
                ):
                    vision_profiles.build_tracking_config(device, sim=True)

    def test_estimator_override_flows_into_config(self):
        """gimbal.tracking.estimator overrides reach the nested estimator
        config, so the robust outlier-rejection knee is profile-tunable."""
        device = {"gimbal": {"camera_pitch": -20.0, "tracking": {
            "enabled": True, "estimator": {"robust_nis_knee": 16.0}}}}
        config = vision_profiles.build_tracking_config(device, sim=True)
        self.assertIsNotNone(config)
        self.assertAlmostEqual(config.rate.estimator.robust_nis_knee, 16.0)

    def test_estimator_defaults_when_not_overridden(self):
        """Without an estimator block the nested config keeps library defaults
        (the robust knee stays on by default — not silently disabled)."""
        from navpy.modules.vision.poi_angle_estimator import PoiAngleEstimatorConfig
        device = {"gimbal": {"camera_pitch": -20.0, "tracking": {"enabled": True}}}
        config = vision_profiles.build_tracking_config(device, sim=True)
        self.assertAlmostEqual(
            config.rate.estimator.robust_nis_knee,
            PoiAngleEstimatorConfig().robust_nis_knee)

    def test_invalid_params_return_none(self):
        self.assertIsNone(vision_profiles.compute_approach_interval(0, 540, 1080, 15, 200, 2.96))
        self.assertIsNone(vision_profiles.compute_approach_interval(2000, 540, 1080, 15, 0, 2.96))
        self.assertIsNone(vision_profiles.compute_approach_interval(2000, 540, 1080, 15, 200, 0))


class TestBuildZoomConfig(unittest.TestCase):
    """Tests for build_zoom_config function."""

    def test_siyi_zr10_derives_target_pixels_from_profile_presets(self):
        """siyi_zr10 zoom config no longer carries target_pixels — values
        derive from the profile's dock_presets via get_min_pixels_for_class."""
        _, profile, _ = vision_profiles.resolve_profile("siyi_zr10", Mock())
        presets = profile["detector"]["dock_presets"]
        # class-0 recognition gate relaxed 48 -> 36 for the 3-UAV demo; both
        # medium and large map to class 0, so both are 36 (the effective
        # class-0 gate is max(medium, large)).
        self.assertEqual(presets["medium"]["min_pixel_size"], 36)
        self.assertEqual(presets["large"]["min_pixel_size"], 36)

        devices = vision_profiles.get_devices(profile)
        config = vision_profiles.build_zoom_config(devices[0], profile)

        self.assertIsNotNone(config)
        self.assertEqual(tuple(config.__dataclass_fields__), ("target_pixels",))

        # Detection class 0 (class 0) should reflect the strictest recognition threshold
        # shared by medium/large in the ZR10 profile (max of 36/36 after
        # the demo relax from 48 -> 36).
        self.assertAlmostEqual(config.target_pixels["0"], 36.0)
        # Person (class 4) → small preset → 42.
        self.assertAlmostEqual(config.target_pixels["4"], 42.0)
        # Default must always be present.
        self.assertIn("default", config.target_pixels)

    def test_c720hd_returns_none(self):
        """c720hd profile has no zoom block → returns None."""
        _, profile, _ = vision_profiles.resolve_profile("c720hd", Mock())
        devices = vision_profiles.get_devices(profile)
        config = vision_profiles.build_zoom_config(devices[0], profile)

        self.assertIsNone(config)

    def test_profiles_without_zoom_block_all_return_none(self):
        """Every non-siyi profile must return None — ensures the new
        two-arg signature is safe across every shipped profile."""
        for profile_name in ("c720hd", "novoxy10", "novoxy18", "novoxy_dual", "zenfone7pro"):
            with self.subTest(profile=profile_name):
                _, profile, _ = vision_profiles.resolve_profile(profile_name, Mock())
                for device in vision_profiles.get_devices(profile):
                    self.assertIsNone(
                        vision_profiles.build_zoom_config(device, profile),
                        msg=f"expected None for {profile_name}/{device.get('name')}",
                    )

    def test_device_without_gimbal_returns_none(self):
        """Device without a gimbal section returns None."""
        self.assertIsNone(vision_profiles.build_zoom_config({"camera": {}}))

    def test_zoom_disabled_returns_none(self):
        """When zoom.enabled is false, returns None."""
        device = {"gimbal": {"zoom": {"enabled": False}}}
        self.assertIsNone(vision_profiles.build_zoom_config(device))

    def test_defaults_when_fields_omitted(self):
        """Only 'enabled=true' provided and no profile → MIN_CONFIRM_PIXELS default."""
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        device = {"gimbal": {"zoom": {"enabled": True}}}
        config = vision_profiles.build_zoom_config(device)

        self.assertIsNotNone(config)
        self.assertEqual(config.target_pixels, {"default": float(MIN_CONFIRM_PIXELS)})

    def test_rejects_all_removed_zoom_knobs(self):
        removed = (
            "target_pixels",
            "tracking_pixels",
            "target_band_ratio",
            "centering_divergence_deadband_frac",
            "min_zoom_interval",
            "absolute_command_step",
            "frame_margin_ratio",
            "min_zoom",
            "max_zoom",
        )
        for key in removed:
            with self.subTest(key=key):
                with self.assertRaisesRegex(ValueError, "Unknown gimbal.zoom"):
                    vision_profiles.build_zoom_config(
                        {"gimbal": {"zoom": {"enabled": True, key: 1}}}
                    )

    def test_enabled_must_be_boolean(self):
        for enabled in (1, 0, "true", None):
            with self.subTest(enabled=enabled):
                with self.assertRaises(ValueError):
                    vision_profiles.build_zoom_config(
                        {"gimbal": {"zoom": {"enabled": enabled}}}
                    )

    def test_derived_thresholds_are_detached_and_immutable(self):
        profile = {
            "detector": {
                "dock_presets": {
                    "medium": {"min_pixel_size": 36},
                    "small": {"min_pixel_size": 42},
                }
            }
        }
        config = vision_profiles.build_zoom_config(
            {"gimbal": {"zoom": {"enabled": True}}},
            profile,
        )
        profile["detector"]["dock_presets"]["medium"]["min_pixel_size"] = 999

        self.assertEqual(config.target_pixels["0"], 36.0)
        with self.assertRaises(TypeError):
            config.target_pixels["0"] = 50.0


class TestGetMinPixelsForClass(unittest.TestCase):
    """Tests for get_min_pixels_for_class resolver."""

    def test_returns_max_for_collisions(self):
        """Multiple presets mapping to the same class_id → strictest (max) wins."""
        profile = {
            "detector": {
                "dock_presets": {
                    "medium": {"min_pixel_size": 11},
                    "large": {"min_pixel_size": 15},
                    "small": {"min_pixel_size": 18},
                },
            },
        }
        # medium + large both map to class 0 → max(11, 15) = 15
        self.assertAlmostEqual(
            vision_profiles.get_min_pixels_for_class(profile, 0), 15.0,
        )
        # small → class 4 → 18
        self.assertAlmostEqual(
            vision_profiles.get_min_pixels_for_class(profile, 4), 18.0,
        )

    def test_falls_back_to_min_confirm_pixels(self):
        """Classes with no matching preset fall back to MIN_CONFIRM_PIXELS."""
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        profile = {"detector": {"dock_presets": {"medium": {"min_pixel_size": 11}}}}
        # Class 4 (Person) has no matching preset → fallback
        self.assertAlmostEqual(
            vision_profiles.get_min_pixels_for_class(profile, 4),
            float(MIN_CONFIRM_PIXELS),
        )

    def test_empty_profile_returns_fallback(self):
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        self.assertAlmostEqual(
            vision_profiles.get_min_pixels_for_class({}, 0),
            float(MIN_CONFIRM_PIXELS),
        )

    def test_ignores_non_numeric_values(self):
        profile = {
            "detector": {
                "dock_presets": {"medium": {"min_pixel_size": "not-a-number"}},
            },
        }
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        self.assertAlmostEqual(
            vision_profiles.get_min_pixels_for_class(profile, 0),
            float(MIN_CONFIRM_PIXELS),
        )

    def test_rejects_nonfinite_or_nonpositive_numeric_values(self):
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        for value in (-1, 0, math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                profile = {
                    "detector": {
                        "dock_presets": {
                            "medium": {"min_pixel_size": value},
                        },
                    },
                }
                self.assertEqual(
                    vision_profiles.get_min_pixels_for_class(profile, 0),
                    float(MIN_CONFIRM_PIXELS),
                )

    def test_rejects_malformed_profile_mapping_shapes(self):
        for profile in (
            {"detector": []},
            {"detector": {"dock_presets": []}},
        ):
            with self.subTest(profile=profile):
                with self.assertRaisesRegex(ValueError, "must be a mapping"):
                    vision_profiles.get_min_pixels_for_class(profile, 0)


class TestGetClassDetectSize(unittest.TestCase):
    """Tests for get_class_detect_size function."""

    def test_known_classes(self):
        # Values are bbox diagonals: sqrt(w^2 + h^2) from DETECTOR_CLASS_DIMENSIONS.
        # Detection class 0(0):   sqrt(3.5^2 + 2.5^2) = sqrt(18.5)  ≈ 4.301
        # Car(3):    sqrt(4.5^2 + 1.5^2) = sqrt(22.5)  ≈ 4.743
        # Person(4): sqrt(0.5^2 + 1.8^2) = sqrt(3.49)  ≈ 1.868
        self.assertAlmostEqual(vision_profiles.get_class_detect_size(0), math.sqrt(18.5), places=4)
        self.assertAlmostEqual(vision_profiles.get_class_detect_size(3), math.sqrt(22.5), places=4)
        self.assertAlmostEqual(vision_profiles.get_class_detect_size(4), math.sqrt(3.49), places=4)

    def test_unknown_class_returns_default(self):
        # Default dims (2.0, 2.0) → diagonal = sqrt(8) ≈ 2.828
        self.assertAlmostEqual(vision_profiles.get_class_detect_size(99), math.sqrt(8.0), places=4)


class TestBuildCameraModel(unittest.TestCase):
    """Tests for build_camera_model function."""

    def setUp(self):
        self.logger = Mock()

    def test_build_camera_model_creates_camera_intrinsics(self):
        """build_camera_model creates CameraIntrinsics from device config."""
        device = {
            "camera": {
                "image_width": 1920,
                "image_height": 1080,
                "intrinsics": {
                    "zooms": {
                        "1": {
                            "fx": 1000.0,
                            "fy": 1000.0,
                            "cx": 960.0,
                            "cy": 540.0,
                        }
                    }
                }
            }
        }

        camera = vision_profiles.build_camera_model(device, self.logger)

        self.assertEqual(camera.image_width, 1920)
        self.assertEqual(camera.image_height, 1080)
        self.logger.info.assert_called()

    def test_build_camera_model_no_log(self):
        """build_camera_model respects log=False."""
        device = {
            "camera": {
                "image_width": 1920,
                "image_height": 1080,
                "intrinsics": {
                    "zooms": {
                        "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
                    }
                }
            }
        }

        camera = vision_profiles.build_camera_model(
            device, self.logger, log=False
        )

        self.logger.info.assert_not_called()

    def test_build_camera_model_rejects_invalid_optics_and_zoom_values(self):
        base_zoom = {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
        invalid_cases = (
            ("0", base_zoom),
            ("1", {**base_zoom, "fx": math.nan}),
            ("1", {**base_zoom, "fy": math.inf}),
            ("1", {**base_zoom, "fy": -1.0}),
        )
        for zoom_key, zoom in invalid_cases:
            with self.subTest(zoom_key=zoom_key, zoom=zoom):
                device = {
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {"zooms": {zoom_key: zoom}},
                    },
                }
                with self.assertRaisesRegex(ValueError, "positive finite"):
                    vision_profiles.build_camera_model(device, self.logger)


class TestBuildCameraMounts(unittest.TestCase):
    """Tests for build_camera_mounts function."""

    def setUp(self):
        self.logger = Mock()
        self.vehicle = Mock()
        self.vehicle.attitude = Attitude(0, 0, 0)

    def test_build_camera_mounts_single_device(self):
        """build_camera_mounts creates mount for single device."""
        profile = {
            "devices": [
                {
                    "name": "test_camera",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {
                            "zooms": {
                                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
                            }
                        }
                    },
                    "gimbal": {
                        "camera_pitch": -45,
                        "stabilize_roll": False,
                        "stabilize_pitch": False,
                        "setup_att": [90, 0, 90],
                    }
                }
            ]
        }

        specs = vision_profiles.build_camera_mounts(
            profile, self.vehicle, self.logger, use_sim_gimbal=False
        )

        self.assertEqual(len(specs), 1)
        spec = specs[0]
        mount = spec.mount
        device = spec.device
        self.assertEqual(mount.name, "test_camera")
        self.assertIsInstance(device, Mapping)
        self.assertEqual(spec.gimbal_device_id, 1)

    def test_camera_mount_spec_snapshots_device_recursively(self):
        from navpy.modules.vision.vision_profile_types import CameraMountSpec

        source = {
            "name": "camera_a",
            "camera": {"image_width": 1920},
            "setup": [1, 2, 3],
        }
        spec = CameraMountSpec(Mock(), source, 1, 0)

        source["name"] = "mutated"
        source["camera"]["image_width"] = 1
        source["setup"].append(4)

        self.assertEqual(spec.device["name"], "camera_a")
        self.assertEqual(spec.device["camera"]["image_width"], 1920)
        self.assertEqual(spec.device["setup"], (1, 2, 3))
        with self.assertRaises(TypeError):
            spec.device["name"] = "caller-mutation"
        with self.assertRaises(TypeError):
            spec.device["camera"]["image_width"] = 1

    def test_build_camera_mounts_siyi_sim_logs_not_fixed(self):
        """A SIYI sim gimbal is articulated; the mount log must report
        fixed=False even when the stabilize_* flags are unset. Regression:
        the label was derived from those flags, so it wrongly read True."""
        profile = {
            "devices": [
                {
                    "name": "siyi_sim_cam",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {
                            "zooms": {
                                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
                            }
                        }
                    },
                    "gimbal": {
                        "type": "siyi",
                        "camera_pitch": -45,
                        "stabilize_roll": False,
                        "stabilize_pitch": False,
                        "setup_att": [90, 0, 90],
                    }
                }
            ]
        }

        specs = vision_profiles.build_camera_mounts(
            profile, self.vehicle, self.logger, use_sim_gimbal=True
        )
        self.assertEqual(len(specs), 1)

        logged = " ".join(
            str(c.args[0]) for c in self.logger.info.call_args_list if c.args
        )
        self.assertIn("fixed=False", logged)
        self.assertNotIn("fixed=True", logged)

    def test_build_camera_mounts_wires_zoom_calibration(self):
        """Profile zoom calibration is available to CameraMount."""
        profile = {
            "devices": [
                {
                    "name": "calibrated_camera",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "zoom_calibration": {
                            "1.0": 1.1,
                            "2.0": 1.8,
                        },
                        "intrinsics": {
                            "zooms": {
                                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0},
                                "2": {"fx": 2000.0, "fy": 2000.0, "cx": 960.0, "cy": 540.0},
                            }
                        }
                    },
                    "gimbal": {
                        "camera_pitch": -45,
                        "stabilize_roll": False,
                        "stabilize_pitch": False,
                        "setup_att": [90, 0, 90],
                    }
                }
            ]
        }

        specs = vision_profiles.build_camera_mounts(
            profile, self.vehicle, self.logger, use_sim_gimbal=False
        )

        mount = specs[0].mount
        self.assertIsNotNone(mount.zoom_calibration)
        self.assertAlmostEqual(
            mount.zoom_calibration.readback_to_command(1.8),
            2.0,
        )

    def test_build_camera_mounts_multiple_devices(self):
        """build_camera_mounts creates mounts for multiple devices."""
        profile = {
            "devices": [
                {
                    "name": "cam1",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {
                            "zooms": {"1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}}
                        }
                    },
                    "gimbal": {"camera_pitch": -30, "setup_att": [0, 0, 0]}
                },
                {
                    "name": "cam2",
                    "camera": {
                        "image_width": 1280,
                        "image_height": 720,
                        "intrinsics": {
                            "zooms": {"1": {"fx": 800.0, "fy": 800.0, "cx": 640.0, "cy": 360.0}}
                        }
                    },
                    "gimbal": {"camera_pitch": -60, "setup_att": [0, 0, 0]}
                }
            ]
        }

        specs = vision_profiles.build_camera_mounts(
            profile, self.vehicle, self.logger, use_sim_gimbal=False
        )

        self.assertEqual(len(specs), 2)
        self.assertEqual(specs[0].mount.name, "cam1")
        self.assertEqual(specs[1].mount.name, "cam2")
        self.assertEqual([spec.gimbal_device_id for spec in specs], [1, 2])

    def test_build_camera_mounts_skips_no_gimbal(self):
        """build_camera_mounts skips devices without gimbal config."""
        profile = {
            "devices": [
                {
                    "name": "no_gimbal",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {
                            "zooms": {"1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}}
                        }
                    }
                    # No gimbal config
                }
            ]
        }

        specs = vision_profiles.build_camera_mounts(
            profile, self.vehicle, self.logger, use_sim_gimbal=False
        )

        self.assertEqual(len(specs), 0)
        self.logger.warning.assert_called()

    def test_build_camera_mounts_uses_fixed_gimbal_when_not_stabilized(self):
        """build_camera_mounts uses FixedGimbal when not stabilized."""
        from navpy.modules.vision.peripheral.fixed_gimbal import FixedGimbal

        profile = {
            "devices": [
                {
                    "name": "fixed_cam",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {
                            "zooms": {"1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}}
                        }
                    },
                    "gimbal": {
                        "camera_pitch": -45,
                        "stabilize_roll": False,
                        "stabilize_pitch": False,
                        "setup_att": [0, 0, 0]
                    }
                }
            ]
        }

        specs = vision_profiles.build_camera_mounts(
            profile, self.vehicle, self.logger, use_sim_gimbal=True
        )

        self.assertEqual(len(specs), 1)
        self.assertIsInstance(specs[0].mount.gimbal, FixedGimbal)


if __name__ == '__main__':
    unittest.main()
