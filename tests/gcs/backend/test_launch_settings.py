"""Tests for LaunchSettings model defaults and serialization."""
import unittest

from gcs.backend.settings_model import LaunchSettings, GcsSettings


class TestLaunchSettingsDefaults(unittest.TestCase):
    """Verify default values for LaunchSettings."""

    def test_defaults(self):
        ls = LaunchSettings()
        self.assertEqual(ls.launch_type, "bungee")
        self.assertEqual(ls.esp32_host, "192.168.4.1")
        self.assertEqual(ls.esp32_port, 80)
        self.assertEqual(ls.default_channel_map, {})
        self.assertEqual(ls.altitude_threshold_m, 10.0)
        self.assertEqual(ls.arm_timeout_s, 10.0)
        self.assertEqual(ls.altitude_timeout_s, 60.0)

    def test_tuning_defaults_preserve_current_behavior(self):
        """New DEV-tunable knobs must default to today's exact behavior."""
        ls = LaunchSettings()
        # Per-path timing
        self.assertEqual(ls.container_settle_s, 0.3)
        self.assertEqual(ls.bungee_settle_s, 0.0)
        self.assertEqual(ls.bungee_arm_timeout_s, 5.0)
        self.assertEqual(ls.container_stagger_s, 0.0)
        self.assertEqual(ls.bungee_stagger_s, 0.0)
        # Container airborne / climb (off by default)
        self.assertFalse(ls.airborne_require_armed)
        self.assertFalse(ls.airborne_require_throttle)
        self.assertEqual(ls.airborne_min_throttle_pct, 20.0)
        self.assertEqual(ls.min_climb_rate_ms, 0.0)
        self.assertEqual(ls.climb_confirm_s, 0.0)
        # Readiness gates (all enabled, current thresholds)
        self.assertTrue(ls.check_gps_enabled)
        self.assertFalse(ls.check_gps_acc_enabled)
        self.assertEqual(ls.max_gps_hacc_m, 1.0)
        self.assertTrue(ls.check_throttle_enabled)
        self.assertEqual(ls.max_throttle_rc3, 1050)
        self.assertTrue(ls.check_battery_enabled)
        self.assertEqual(ls.min_battery_pct, 15.0)
        self.assertTrue(ls.check_prearm_enabled)
        self.assertFalse(ls.block_on_unknown_battery)
        # Launch sequence (empty = discovery order)
        self.assertEqual(ls.launch_order, [])

    def test_backward_compat_old_json_without_new_keys(self):
        """An old gcs_settings.json launch block (no new keys) must still load."""
        legacy = {
            "launch_type": "container",
            "esp32_host": "10.0.0.1",
            "esp32_port": 8032,
            "default_channel_map": {"1": 2},
            "altitude_threshold_m": 12.0,
            "arm_timeout_s": 8.0,
            "altitude_timeout_s": 45.0,
        }
        ls = LaunchSettings.model_validate(legacy)
        # Preserved explicit legacy values
        self.assertEqual(ls.launch_type, "container")
        self.assertEqual(ls.altitude_timeout_s, 45.0)
        # New keys fall back to behavior-preserving defaults
        self.assertEqual(ls.container_settle_s, 0.3)
        self.assertEqual(ls.bungee_arm_timeout_s, 5.0)
        self.assertTrue(ls.check_battery_enabled)
        self.assertEqual(ls.launch_order, [])

    def test_custom_values(self):
        ls = LaunchSettings(
            launch_type="container",
            esp32_host="10.0.0.1",
            esp32_port=8032,
            default_channel_map={"1": 3, "2": 4},
            altitude_threshold_m=15.0,
            arm_timeout_s=20.0,
            altitude_timeout_s=90.0,
        )
        self.assertEqual(ls.launch_type, "container")
        self.assertEqual(ls.esp32_host, "10.0.0.1")
        self.assertEqual(ls.esp32_port, 8032)
        self.assertEqual(ls.default_channel_map, {"1": 3, "2": 4})
        self.assertEqual(ls.altitude_threshold_m, 15.0)

    def test_round_trip_json(self):
        ls = LaunchSettings(launch_type="container", default_channel_map={"1": 2})
        data = ls.model_dump()
        restored = LaunchSettings.model_validate(data)
        self.assertEqual(ls, restored)

    def test_round_trip_json_with_tuning(self):
        ls = LaunchSettings(
            launch_type="container",
            container_settle_s=0.5,
            bungee_arm_timeout_s=7.0,
            container_stagger_s=2.0,
            airborne_require_armed=True,
            min_climb_rate_ms=1.5,
            climb_confirm_s=2.0,
            check_throttle_enabled=False,
            max_throttle_rc3=1100,
            min_battery_pct=20.0,
            block_on_unknown_battery=True,
            launch_order=[3, 1, 2],
        )
        restored = LaunchSettings.model_validate(ls.model_dump())
        self.assertEqual(ls, restored)
        self.assertEqual(restored.launch_order, [3, 1, 2])
        self.assertFalse(restored.check_throttle_enabled)


class TestGcsSettingsIncludesLaunch(unittest.TestCase):
    """Verify LaunchSettings is part of GcsSettings."""

    def test_default_gcs_has_launch(self):
        gcs = GcsSettings()
        self.assertIsInstance(gcs.launch, LaunchSettings)
        self.assertEqual(gcs.launch.launch_type, "bungee")

    def test_gcs_round_trip(self):
        gcs = GcsSettings(launch=LaunchSettings(launch_type="container", esp32_port=8032))
        data = gcs.model_dump()
        restored = GcsSettings.model_validate(data)
        self.assertEqual(restored.launch.launch_type, "container")
        self.assertEqual(restored.launch.esp32_port, 8032)


if __name__ == "__main__":
    unittest.main()
