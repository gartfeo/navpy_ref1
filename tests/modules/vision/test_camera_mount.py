"""Tests for CameraMount and FixedGimbal."""
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.peripheral.camera_abc import CameraAbc
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.fixed_gimbal import FixedGimbal
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalAbc,
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.target_zoom_mount_adapters import MountZoomAdapter
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState
from tests.conftest import create_test_camera


class TestFixedGimbal(unittest.TestCase):
    """Tests for FixedGimbal class."""

    def test_init_stores_data(self):
        """FixedGimbal stores initial data."""
        data = GimbalData(att=Attitude(-45, 0, 0))
        gimbal = FixedGimbal(data)

        result = gimbal.get_data()
        self.assertEqual(result.att.pitch, -45)
        self.assertEqual(result.att.roll, 0)
        self.assertEqual(result.att.yaw, 0)

    def test_set_att_updates_attitude(self):
        """set_att updates the gimbal attitude."""
        data = GimbalData(att=Attitude(-45, 0, 0))
        gimbal = FixedGimbal(data)

        # Attitude order is (pitch, yaw, roll)
        gimbal.set_att(Attitude(-30, 10, 5))

        result = gimbal.get_data()
        self.assertEqual(result.att.pitch, -30)
        self.assertEqual(result.att.yaw, 10)
        self.assertEqual(result.att.roll, 5)

    def test_set_att_preserves_other_fields(self):
        """set_att preserves non-attitude fields."""
        data = GimbalData(
            att=Attitude(-45, 0, 0),
            name="test_gimbal",
            max_detect_distance=5000
        )
        gimbal = FixedGimbal(data)

        gimbal.set_att(Attitude(-30, 0, 0))

        result = gimbal.get_data()
        self.assertEqual(result.name, "test_gimbal")
        self.assertEqual(result.max_detect_distance, 5000)

    def test_args_property_returns_self(self):
        """GimbalData.args returns self for backward compatibility."""
        data = GimbalData(att=Attitude(-45, 0, 0), setup=GimbalMountSetup(seq="ZYX"))
        gimbal = FixedGimbal(data)

        result = gimbal.get_data()
        self.assertIs(result.args, result)
        self.assertEqual(result.args.setup_seq, "ZYX")


class TestCameraMount(unittest.TestCase):
    """Tests for CameraMount class."""

    def setUp(self):
        self.camera = create_test_camera(
            image_width=1920,
            image_height=1080,
            fov=60,
            sensor_width=4.8,
            sensor_height=3.6
        )
        self.gimbal_data = GimbalData(att=Attitude(-45, 0, 0))
        self.gimbal = FixedGimbal(self.gimbal_data)

    def test_static_mount_health_is_an_explicit_noop(self):
        camera = CameraIntrinsics(100.0, 100.0, 50.0, 50.0)
        mount = CameraMount("fixed", camera, self.gimbal)

        mount.raise_if_failed()

    def test_legacy_abc_implementations_inherit_static_health_default(self):
        class LegacyCamera(CameraAbc):
            def get_k(self):
                return np.eye(3)

            def set_zoom(self, zoom):
                return zoom

        class LegacyGimbal(GimbalAbc):
            def get_data(self):
                return self.gimbal_data

            def set_att(self, att):
                self.gimbal_data = GimbalData(att=att)

        legacy_gimbal = LegacyGimbal()
        legacy_gimbal.gimbal_data = self.gimbal_data
        mount = CameraMount("legacy", LegacyCamera(), legacy_gimbal)

        mount.raise_if_failed()

    def test_mount_propagates_exact_gimbal_health_failure(self):
        failure = RuntimeError("gimbal worker failed")

        class FailingGimbal(FixedGimbal):
            def raise_if_failed(self) -> None:
                raise failure

        camera = CameraIntrinsics(100.0, 100.0, 50.0, 50.0)
        mount = CameraMount(
            "worker-backed",
            camera,
            FailingGimbal(self.gimbal_data),
        )

        with self.assertRaises(RuntimeError) as raised:
            mount.raise_if_failed()

        self.assertIs(raised.exception, failure)

    def test_zoom_capabilities_follow_live_gimbal_replacement(self):
        class _ContinuousGimbal(FixedGimbal):
            def supports_continuous_zoom_control(self):
                return True

            def zoom_in(self):
                return False

            def zoom_out(self):
                return False

            def zoom_hold(self):
                return False

        mount = CameraMount(
            name="test",
            camera=self.camera,
            gimbal=self.gimbal,
        )
        adapter = MountZoomAdapter(mount, Mock())
        self.assertFalse(adapter.capabilities.supported)

        mount.gimbal = _ContinuousGimbal(self.gimbal_data)
        self.assertTrue(adapter.capabilities.continuous)
        self.assertFalse(
            adapter.start_continuous(ZoomTrackingState.ZOOMING_IN)
        )
        self.assertFalse(adapter.hold())

        mount.gimbal = self.gimbal
        self.assertFalse(adapter.capabilities.supported)

    def test_get_live_optics_fixed_mount_reports_camera_intrinsics(self):
        """A fixed mount (no zoom readback) reports optics from the camera's own
        current intrinsics, tagged camera-sourced, with no set_zoom side effect."""
        mount = CameraMount(name="test", camera=self.camera, gimbal=self.gimbal)
        self.camera.set_zoom = Mock(wraps=self.camera.set_zoom)

        optics = mount.get_live_optics()

        self.assertIsNotNone(optics)
        self.assertFalse(optics.sample_from_hardware)
        k = mount.get_k()
        expected_fov_v = 2.0 * math.atan(mount.image_height / (2.0 * float(k[1, 1])))
        expected_fov_h = 2.0 * math.atan(mount.image_width / (2.0 * float(k[0, 0])))
        self.assertAlmostEqual(optics.fov_v_rad, expected_fov_v, places=9)
        self.assertAlmostEqual(optics.fov_h_rad, expected_fov_h, places=9)
        self.camera.set_zoom.assert_not_called()

    def test_get_live_optics_stale_hardware_readback_fails_closed(self):
        """A mount WITH zoom-readback capability but no fresh sample returns None
        — it must not fall back to possibly-wrong base intrinsics."""
        class _ReadbackGimbal(FixedGimbal):
            def supports_zoom_readback(self):
                return True

            def get_zoom_level_sample(self):
                return None

        mount = CameraMount(
            name="test", camera=self.camera,
            gimbal=_ReadbackGimbal(self.gimbal_data),
        )
        self.assertTrue(mount._gimbal_has_zoom_readback())
        self.assertIsNone(mount.get_live_optics())

    def test_get_live_optics_broken_readback_probe_surfaces_driver_failure(self):
        """Programming/driver defects are not disguised as missing telemetry."""
        class _BrokenGimbal(FixedGimbal):
            def supports_zoom_readback(self):
                raise RuntimeError("probe boom")

        mount = CameraMount(
            name="t", camera=self.camera, gimbal=_BrokenGimbal(self.gimbal_data),
        )
        with self.assertRaisesRegex(RuntimeError, "probe boom"):
            mount.get_live_optics()

    def test_gimbal_has_zoom_readback_probe(self):
        """The capability probe distinguishes a fixed mount from a readback mount."""
        class _ReadbackGimbal(FixedGimbal):
            def supports_zoom_readback(self):
                return True

            def get_zoom_level_sample(self):
                return (1.0, 0.0, "s")

        fixed = CameraMount(name="f", camera=self.camera, gimbal=self.gimbal)
        readback = CameraMount(
            name="r", camera=self.camera,
            gimbal=_ReadbackGimbal(self.gimbal_data),
        )
        self.assertFalse(fixed._gimbal_has_zoom_readback())
        self.assertTrue(readback._gimbal_has_zoom_readback())

    def test_init_stores_components(self):
        """CameraMount stores camera and gimbal."""
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal
        )

        self.assertEqual(mount.name, "test_mount")
        self.assertIs(mount.camera, self.camera)
        self.assertIs(mount.gimbal, self.gimbal)

    def test_get_k_returns_camera_intrinsics(self):
        """get_k returns camera intrinsic matrix."""
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal
        )

        k = mount.get_k()
        self.assertEqual(k.shape, (3, 3))
        self.assertIsInstance(k, np.ndarray)

    def test_get_gimbal_data_returns_gimbal_state(self):
        """get_gimbal_data returns current gimbal data."""
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal
        )

        data = mount.get_gimbal_data()
        self.assertEqual(data.att.pitch, -45)

    def test_capture_frame_state_freezes_fixed_gimbal_and_scaled_intrinsics(self):
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal,
        )
        expected_k = mount.get_k_for_frame(960, 540).copy()

        state = mount.capture_frame_state(960, 540)
        self.gimbal.set_att(Attitude(-10, 20, 30))

        self.assertIsNotNone(state)
        self.assertTrue(state.gimbal_is_static)
        self.assertEqual(state.gimbal_data.att, Attitude(-45, 0, 0))
        np.testing.assert_allclose(state.k, expected_k)
        self.assertFalse(state.k.flags.writeable)
        self.assertFalse(state.dist.flags.writeable)
        self.assertEqual(state.zoom_sample_id, "static:fixed")
        self.assertEqual(state.zoom_sample_age_s, 0.0)

    def test_capture_frame_state_binds_dynamic_gimbal_zoom_and_k(self):
        camera = CameraIntrinsics(
            zoom_map={
                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
                "2": {"fx": 2000.0, "fy": 2500.0, "cx": 500.0, "cy": 250.0},
            },
            image_width=1000,
            image_height=500,
        )
        sample = GimbalData(
            att=Attitude(-20.0, 10.0, 5.0),
            timestamp_s=99.98,
        )
        gimbal = SimpleNamespace(
            state_is_static=lambda: False,
            supports_zoom_readback=lambda: True,
            get_frame_state_sample=lambda: (sample, 2.0, 0.02, 7),
        )
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        with patch(
            "navpy.modules.vision.camera_mount_frame_state.time.monotonic",
            side_effect=(10.0, 10.005),
        ):
            state = mount.capture_frame_state(1000, 500)

        self.assertIsNotNone(state)
        self.assertFalse(state.gimbal_is_static)
        self.assertEqual(state.gimbal_data.att, sample.att)
        self.assertIsNot(state.gimbal_data, sample)
        self.assertEqual(state.gimbal_timestamp_s, 99.98)
        self.assertEqual(state.zoom_command, "2.0")
        self.assertEqual(state.zoom_sample_id, "7")
        self.assertAlmostEqual(state.zoom_sample_age_s, 0.025)
        self.assertEqual(float(state.k[0, 0]), 2000.0)
        self.assertEqual(float(state.k[1, 1]), 2500.0)

    def test_capture_frame_state_keeps_unproven_zoom_for_operator_path(self):
        sample = GimbalData(
            att=Attitude(-20.0, 10.0, 5.0),
            timestamp_s=100.0,
        )
        gimbal = SimpleNamespace(
            state_is_static=lambda: False,
            supports_zoom_readback=lambda: True,
            get_frame_state_sample=lambda: (sample, None, None, None),
        )
        mount = CameraMount(name="test", camera=self.camera, gimbal=gimbal)

        state = mount.capture_frame_state(1920, 1080)

        self.assertIsNotNone(state)
        self.assertIsNone(state.zoom_sample_id)
        self.assertIsNone(state.zoom_sample_age_s)
        self.assertEqual(state.gimbal_timestamp_s, 100.0)
        self.assertEqual(state.k.shape, (3, 3))

    def test_capture_frame_state_serializes_zoom_tag_and_k_against_writers(self):
        camera = CameraIntrinsics(
            zoom_map={
                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
                "2": {"fx": 2000.0, "fy": 2000.0, "cx": 500.0, "cy": 250.0},
            },
            image_width=1000,
            image_height=500,
        )
        gimbal_sample = GimbalData(
            att=Attitude(-20.0, 10.0, 5.0),
            timestamp_s=time.time(),
        )
        hardware_set = threading.Event()
        gimbal = SimpleNamespace(
            state_is_static=lambda: False,
            supports_zoom_readback=lambda: True,
            get_frame_state_sample=lambda: (gimbal_sample, 1.0, 0.01, 7),
            set_zoom=lambda _zoom: hardware_set.set() or True,
        )
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        get_k_entered = threading.Event()
        release_get_k = threading.Event()
        original_get_k = camera.get_k

        def blocking_get_k():
            get_k_entered.set()
            if not release_get_k.wait(timeout=1.0):
                raise TimeoutError("test did not release get_k")
            return original_get_k()

        camera.get_k = blocking_get_k
        captured = []
        capture_thread = threading.Thread(
            target=lambda: captured.append(mount.capture_frame_state(1000, 500))
        )
        capture_thread.start()
        self.assertTrue(get_k_entered.wait(timeout=1.0))

        writer_started = threading.Event()

        def write_zoom():
            writer_started.set()
            mount.set_zoom("2")

        writer_thread = threading.Thread(target=write_zoom)
        writer_thread.start()
        self.assertTrue(writer_started.wait(timeout=1.0))
        self.assertFalse(hardware_set.wait(timeout=0.05))

        release_get_k.set()
        capture_thread.join(timeout=1.0)
        writer_thread.join(timeout=1.0)

        self.assertFalse(capture_thread.is_alive())
        self.assertFalse(writer_thread.is_alive())
        self.assertEqual(captured[0].zoom_command, "1.0")
        self.assertEqual(float(captured[0].k[0, 0]), 1000.0)
        self.assertEqual(float(camera.get_k()[0, 0]), 2000.0)

    def test_image_dimensions(self):
        """image_width and image_height return camera dimensions."""
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal
        )

        self.assertEqual(mount.image_width, 1920)
        self.assertEqual(mount.image_height, 1080)

    def test_is_valid_in_bounds(self):
        """is_valid returns True for coordinates within image."""
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal
        )

        self.assertTrue(mount.is_valid(960, 540))  # Center
        self.assertTrue(mount.is_valid(0, 0))       # Top-left
        self.assertTrue(mount.is_valid(1919, 1079)) # Bottom-right

    def test_is_valid_out_of_bounds(self):
        """is_valid returns False for coordinates outside image."""
        mount = CameraMount(
            name="test_mount",
            camera=self.camera,
            gimbal=self.gimbal
        )

        # create_test_camera uses inclusive bounds (0 <= u <= width)
        self.assertFalse(mount.is_valid(-1, 540))
        self.assertFalse(mount.is_valid(960, -1))
        self.assertFalse(mount.is_valid(1921, 540))  # > image_width
        self.assertFalse(mount.is_valid(960, 1081))  # > image_height

    def test_start_calls_gimbal_start(self):
        """start calls gimbal.start if it exists."""
        gimbal = Mock()
        mount = CameraMount(name="test", camera=self.camera, gimbal=gimbal)

        mount.start()
        gimbal.start.assert_called_once()

    def test_stop_calls_gimbal_stop(self):
        """stop calls gimbal.stop if it exists."""
        gimbal = Mock()
        gimbal.stop.return_value = True
        mount = CameraMount(name="test", camera=self.camera, gimbal=gimbal)

        self.assertIs(mount.stop(), True)
        gimbal.stop.assert_called_once()

    def test_stop_rejects_non_boolean_gimbal_quiescence(self):
        gimbal = Mock()
        gimbal.stop.return_value = None
        mount = CameraMount(name="test", camera=self.camera, gimbal=gimbal)

        with self.assertRaisesRegex(TypeError, "stop.*return bool"):
            mount.stop()

    def test_stop_propagates_incomplete_gimbal_quiescence(self):
        gimbal = Mock()
        gimbal.stop.return_value = False
        mount = CameraMount(name="test", camera=self.camera, gimbal=gimbal)

        self.assertIs(mount.stop(), False)

    def test_refresh_calls_both(self):
        """refresh calls refresh on both camera and gimbal."""
        camera = Mock()
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        mount.refresh()
        camera.refresh.assert_called_once()
        gimbal.refresh.assert_called_once()

    def test_set_zoom_updates_camera_without_hardware_zoom(self):
        """set_zoom falls back to camera intrinsics when no hardware path exists."""
        camera = Mock()
        camera._zoom_map = {}
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.set_zoom.return_value = None
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        result = mount.set_zoom("2")

        self.assertTrue(result)
        gimbal.set_zoom.assert_called_once_with("2")
        camera.set_zoom.assert_called_once_with("2")

    def test_set_zoom_prepares_intrinsics_before_hardware_commit(self):
        """set_zoom validates intrinsics before committing hardware zoom."""
        calls = []
        camera = Mock()
        camera._zoom_map = {"1": {}, "2": {}}
        camera.set_zoom.side_effect = lambda zoom: calls.append(("camera", zoom)) or True
        gimbal = Mock()
        gimbal.set_zoom.side_effect = lambda zoom: calls.append(("gimbal", zoom)) or True
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        result = mount.set_zoom("2")

        self.assertTrue(result)
        self.assertEqual(calls, [("camera", "2"), ("gimbal", "2")])

    def test_set_zoom_rolls_back_intrinsics_when_hardware_commit_fails(self):
        """set_zoom restores the previous zoom if hardware commit fails."""
        calls = []

        camera = Mock()
        camera._zoom_map = {"1": {}, "2": {}}
        camera._zoom = "1"

        def _camera_set_zoom(zoom):
            calls.append(("camera", zoom))
            camera._zoom = zoom
            return True

        camera.set_zoom.side_effect = _camera_set_zoom

        gimbal = Mock()
        gimbal.set_zoom.side_effect = lambda zoom: calls.append(("gimbal", zoom)) or False

        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        result = mount.set_zoom("2")

        self.assertFalse(result)
        self.assertEqual(camera._zoom, "1")
        self.assertEqual(calls, [("camera", "2"), ("gimbal", "2"), ("camera", "1")])

    def test_set_zoom_uncalibrated_level_passed_to_camera(self):
        """set_zoom passes uncalibrated levels to camera for interpolation."""
        camera = Mock()
        camera._zoom_map = {"1": {}, "2": {}}
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.set_zoom.return_value = True
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        result = mount.set_zoom("3")

        self.assertTrue(result)
        camera.set_zoom.assert_called_once_with("3")

    def test_command_zoom_does_not_update_camera_intrinsics(self):
        """Live command path sends hardware zoom only."""
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.set_zoom.return_value = True
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        result = mount.command_zoom("2")

        self.assertTrue(result)
        gimbal.set_zoom.assert_called_once_with("2")
        camera.set_zoom.assert_not_called()

    def test_command_zoom_without_hardware_support_returns_false(self):
        """No gimbal set_zoom path means no live absolute zoom support."""
        camera = Mock()
        gimbal = Mock(spec=[])
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertFalse(mount.command_zoom("2"))

    def test_get_current_zoom_command_falls_back_to_camera_key(self):
        """Without hardware readback, command-space zoom comes from camera."""
        camera = Mock()
        camera._zoom = "2"
        gimbal = Mock(spec=[])
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertEqual(mount.get_current_zoom_command(), "2")

    def test_sync_zoom_from_hardware_updates_intrinsics(self):
        """sync_zoom_from_hardware pushes gimbal zoom into camera intrinsics."""
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 2.5
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertTrue(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_called_once_with("2.5")

    def test_sync_zoom_from_hardware_skips_when_unchanged(self):
        """sync_zoom_from_hardware skips the camera call when zoom already matches."""
        camera = Mock()
        camera._zoom = "2.5"
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 2.5
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertFalse(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_not_called()

    def test_sync_zoom_from_hardware_without_gimbal_support(self):
        """No get_zoom_level on gimbal → returns False and does nothing."""
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock(spec=[])  # no get_zoom_level
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertFalse(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_not_called()

    def test_sync_zoom_from_hardware_ignores_invalid_level(self):
        """Non-numeric or non-positive hardware zoom is ignored."""
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 0.0
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertFalse(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_not_called()

    def test_sync_zoom_from_hardware_propagates_camera_failure(self):
        """If camera rejects the new level, sync returns False."""
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = False
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 4.0
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertFalse(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_called_once_with("4.0")

    def test_get_live_optics_uses_synced_camera_intrinsics(self):
        """Live optics report FOV from the post-readback camera K matrix."""
        camera = CameraIntrinsics(
            zoom_map={
                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
                "2": {"fx": 2000.0, "fy": 2500.0, "cx": 500.0, "cy": 250.0},
            },
            image_width=1000,
            image_height=500,
        )
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 1
        gimbal.get_zoom_level_age_s.return_value = 99.0
        gimbal.get_zoom_level_sample_id.return_value = 99
        gimbal.get_zoom_level_sample.return_value = (2, 0.1, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        optics = mount.get_live_optics()

        self.assertIsNotNone(optics)
        self.assertEqual(optics.zoom_command, "2")
        self.assertEqual(optics.zoom_level, "2")
        self.assertEqual(optics.sample_id, "7")
        self.assertAlmostEqual(optics.fov_h_rad, 2 * np.arctan(1000 / (2 * 2000)))
        self.assertAlmostEqual(optics.fov_v_rad, 2 * np.arctan(500 / (2 * 2500)))
        gimbal.get_zoom_level.assert_not_called()
        gimbal.get_zoom_level_age_s.assert_not_called()
        gimbal.get_zoom_level_sample_id.assert_not_called()

    def test_get_live_optics_rejects_unsynced_intrinsics(self):
        """A hardware readback is not enough if camera intrinsics cannot sync."""
        camera = Mock()
        camera._zoom = "1"
        camera.image_width = 1000
        camera.image_height = 500
        camera.get_k.return_value = np.array([[1000.0, 0.0, 500.0],
                                              [0.0, 1000.0, 250.0],
                                              [0.0, 0.0, 1.0]])
        camera.set_zoom.return_value = False
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (2, 0.1, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_live_optics())
        camera.set_zoom.assert_called_once_with("2")

    def test_get_live_optics_returns_none_for_invalid_intrinsics(self):
        """Invalid K/image dimensions fail closed."""
        camera = Mock()
        camera.get_k.return_value = np.array([[0.0, 0.0, 0.0],
                                              [0.0, 1000.0, 0.0],
                                              [0.0, 0.0, 1.0]])
        camera.image_width = 1000
        camera.image_height = 500
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (2, 0.1, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_live_optics())

    def test_get_live_optics_requires_fresh_hardware_readback(self):
        """Cached hardware zoom older than the freshness window is rejected."""
        camera = CameraIntrinsics(
            zoom_map={
                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
            },
            image_width=1000,
            image_height=500,
        )
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (1, 99.0, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_live_optics())

    def test_get_live_optics_requires_hardware_sample_id(self):
        """Fresh age without a sample identity is not enough to republish optics."""
        camera = CameraIntrinsics(
            zoom_map={
                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
            },
            image_width=1000,
            image_height=500,
        )
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (1, 0.1, None)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_live_optics())

    def test_get_live_optics_static_mount_reports_camera_optics(self):
        """A mount with NO zoom-readback capability reports the camera's own
        current intrinsics as (camera-sourced) optics rather than failing closed,
        so a fixed camera can publish its FOV."""
        camera = CameraIntrinsics(
            zoom_map={
                "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
            },
            image_width=1000,
            image_height=500,
        )
        gimbal = Mock(spec=[])
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        optics = mount.get_live_optics()
        self.assertIsNotNone(optics)
        self.assertFalse(optics.sample_from_hardware)
        self.assertAlmostEqual(
            optics.fov_h_rad, 2.0 * math.atan(1000 / (2.0 * 1000.0)), places=9)
        self.assertAlmostEqual(
            optics.fov_v_rad, 2.0 * math.atan(500 / (2.0 * 1000.0)), places=9)

    def test_get_fresh_zoom_command_uses_sample_without_camera_sync(self):
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (2, 0.1, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertEqual(mount.get_fresh_zoom_command(), "2")
        camera.set_zoom.assert_not_called()
        gimbal.get_zoom_level.assert_not_called()

    def test_get_fresh_zoom_command_requires_live_hardware_readback(self):
        camera = Mock()
        camera._zoom = "30"
        gimbal = Mock(spec=[])
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_fresh_zoom_command())

    def test_get_fresh_zoom_command_rejects_stale_hardware_readback(self):
        camera = Mock()
        camera._zoom = "30"
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 30.0
        gimbal.get_zoom_level_age_s.return_value = 99.0
        gimbal.get_zoom_level_sample.return_value = (30.0, 99.0, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_fresh_zoom_command())

    def test_get_fresh_zoom_sample_id_returns_sample_token(self):
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (2, 0.1, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertEqual(mount.get_fresh_zoom_sample_id(), "7")

    def test_get_fresh_zoom_sample_id_rejects_stale_readback(self):
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock()
        gimbal.get_zoom_level_sample.return_value = (2, 99.0, 7)
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_fresh_zoom_sample_id())

    def test_get_fresh_zoom_sample_id_requires_sample_telemetry(self):
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock(spec=[])
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        self.assertIsNone(mount.get_fresh_zoom_sample_id())


class TestSyncZoomReverseLookup(unittest.TestCase):
    """sync_zoom_from_hardware with a ZoomCalibrationTable → reverse-lookup."""

    def _siyi_cal(self):
        from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable
        return ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "1.4": 1.2, "1.5": 1.3, "1.6": 1.4, "1.7": 1.5,
            "1.8": 1.6, "1.9": 1.7, "2.0": 1.8, "3.0": 2.8,
        })

    def test_sync_uses_reverse_lookup_when_calibration_present(self):
        """Raw readback 1.5 → reverse-lookup cmd 1.70 → set_zoom('1.70')."""
        cal = self._siyi_cal()
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 1.5
        mount = CameraMount(
            name="test", camera=camera, gimbal=gimbal, zoom_calibration=cal,
        )

        self.assertTrue(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_called_once_with("1.70")

    def test_get_current_zoom_command_uses_reverse_lookup(self):
        """Live current zoom is reported in command space when calibrated."""
        cal = self._siyi_cal()
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 1.5
        mount = CameraMount(
            name="test", camera=camera, gimbal=gimbal, zoom_calibration=cal,
        )

        self.assertEqual(mount.get_current_zoom_command(), "1.70")

    def test_sync_compares_after_reverse_lookup(self):
        """If camera._zoom already stores the reverse-looked-up cmd key,
        sync must return False without calling set_zoom — the compare
        must happen AFTER reverse-lookup, not against the raw readback."""
        cal = self._siyi_cal()
        camera = Mock()
        camera._zoom = "1.70"   # cmd-space key, matches reverse-lookup(1.5)
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 1.5  # raw readback differs from key
        mount = CameraMount(
            name="test", camera=camera, gimbal=gimbal, zoom_calibration=cal,
        )

        self.assertFalse(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_not_called()

    def test_sync_falls_back_to_readback_string_when_no_calibration(self):
        """Without a calibration table the legacy readback-string path is used."""
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 1.5
        mount = CameraMount(
            name="test", camera=camera, gimbal=gimbal, zoom_calibration=None,
        )

        self.assertTrue(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_called_once_with("1.5")

    def test_sync_returns_false_when_readback_to_command_returns_none(self):
        """Empty calibration table → readback_to_command returns None →
        sync returns False and does not call set_zoom."""
        from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable
        empty_cal = ZoomCalibrationTable(entries=())
        camera = Mock()
        camera._zoom = "1"
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 1.5
        mount = CameraMount(
            name="test", camera=camera, gimbal=gimbal,
            zoom_calibration=empty_cal,
        )

        self.assertFalse(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_not_called()

    def test_sync_clamps_readback_outside_measured_range(self):
        """Readback above the calibrated max clamps to the highest canonical
        cmd. Example: cal covers readback 1.1..2.8; readback 5.0 clamps
        to the cmd that produces 2.8, i.e. '3.0' → '3.00'."""
        cal = self._siyi_cal()
        camera = Mock()
        camera._zoom = "1"
        camera.set_zoom.return_value = True
        gimbal = Mock()
        gimbal.get_zoom_level.return_value = 5.0
        mount = CameraMount(
            name="test", camera=camera, gimbal=gimbal, zoom_calibration=cal,
        )

        self.assertTrue(mount.sync_zoom_from_hardware())
        camera.set_zoom.assert_called_once_with("3.00")

    def test_has_zoom_true_when_fy_ratio_covers_detect_to_confirm_gap(self):
        """siyi-like fy span (2083→20828, ratio 10) exceeds the required 2.5×."""
        camera = Mock()
        camera._zoom_map = {
            "1": {"fy": 2083.0},
            "5": {"fy": 10500.0},
            "10": {"fy": 20828.0},
        }
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertTrue(mount.has_zoom)

    def test_has_zoom_false_when_fy_ratio_below_required(self):
        """novoxy_dual straight: keys 1/2/3 but fy ratio 1978→4363 ≈ 2.20 < 2.5.
        Must be classed as fixed — the camera cannot physically close the
        detect→confirm pixel gap."""
        camera = Mock()
        camera._zoom_map = {
            "1": {"fy": 1978.6},
            "2": {"fy": 3139.5},
            "3": {"fy": 4363.0},
        }
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertFalse(mount.has_zoom)

    def test_has_zoom_false_with_single_level(self):
        camera = Mock()
        camera._zoom_map = {"1": {"fy": 1440.0}}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertFalse(mount.has_zoom)

    def test_has_zoom_false_with_no_zoom_map(self):
        camera = Mock()
        camera._zoom_map = {}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertFalse(mount.has_zoom)

    def test_zoom_ratio_returns_fy_max_over_min(self):
        """Ratio is computed from fy, not calibration keys."""
        camera = Mock()
        camera._zoom_map = {
            "1": {"fy": 2000.0},
            "4": {"fy": 8000.0},
            "10": {"fy": 20000.0},
        }
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertAlmostEqual(mount.zoom_ratio, 10.0)

    def test_zoom_ratio_ignores_labels_uses_focal_length(self):
        """Calibration keys 1/10 with equal fy → ratio 1.0 (not 10)."""
        camera = Mock()
        camera._zoom_map = {
            "1": {"fy": 2000.0},
            "10": {"fy": 2000.0},  # Mis-calibrated duplicate
        }
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertAlmostEqual(mount.zoom_ratio, 1.0)

    def test_zoom_ratio_is_one_for_fixed_camera(self):
        camera = Mock()
        camera._zoom_map = {"1": {"fy": 1440.0}}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertAlmostEqual(mount.zoom_ratio, 1.0)

    def test_zoom_ratio_one_when_entries_lack_fy(self):
        """Profile entries missing fy are ignored; less than 2 valid → 1.0."""
        camera = Mock()
        camera._zoom_map = {"1": {}, "2": {}}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertAlmostEqual(mount.zoom_ratio, 1.0)

    def test_base_fy_returns_smallest_calibrated_fy(self):
        """base_fy is the 1x (widest-FOV) focal length = min fy across levels."""
        camera = Mock()
        camera._zoom_map = {
            "1": {"fy": 2083.0},
            "5": {"fy": 10500.0},
            "10": {"fy": 20828.0},
        }
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertAlmostEqual(mount.base_fy, 2083.0)

    def test_base_fy_ignores_entries_without_fy(self):
        """Entries missing fy are skipped; the smallest valid fy wins."""
        camera = Mock()
        camera._zoom_map = {"1": {"fy": 1440.0}, "2": {}, "3": {"fy": 4000.0}}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertAlmostEqual(mount.base_fy, 1440.0)

    def test_base_fy_none_without_usable_zoom_map(self):
        """No usable _zoom_map → None, so callers fall back to get_k."""
        camera = Mock()
        camera._zoom_map = {}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertIsNone(mount.base_fy)

    def test_base_fy_none_when_entries_lack_fy(self):
        """All entries missing fy → None (no usable focal length)."""
        camera = Mock()
        camera._zoom_map = {"1": {}, "2": {}}
        gimbal = Mock()
        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)
        self.assertIsNone(mount.base_fy)


class TestCameraMountWithGimbalSim(unittest.TestCase):
    """Integration tests for CameraMount with GimbalSim."""

    def test_mount_with_gimbal_sim(self):
        """CameraMount works with GimbalSim."""
        from unittest.mock import Mock
        from navpy.modules.vision.gimbal_attitude_reader import (
            VehicleAttitudeReader,
        )
        from navpy.modules.vision.sim.gimbal_sim import GimbalSim
        from navpy.modules.vehicle.vehicle_interface import IVehicle

        vehicle = Mock(spec=IVehicle)
        vehicle.attitude = Attitude(-45, 0, 0)
        camera = create_test_camera(image_width=1920, image_height=1080)
        gimbal_data = GimbalData(att=Attitude(-30, 0, 0))
        gimbal = GimbalSim(gimbal_data, VehicleAttitudeReader(vehicle))

        mount = CameraMount(name="test", camera=camera, gimbal=gimbal)

        # Should work without errors
        k = mount.get_k()
        data = mount.get_gimbal_data()

        self.assertEqual(k.shape, (3, 3))
        self.assertEqual(data.att.pitch, -30)


class _FakeCam:
    """Camera with a fixed calibration K and explicit canvas dims, for
    exercising CameraMount.get_k_for_frame scaling in isolation."""

    def __init__(self, k, image_width, image_height):
        self._k = np.array(k, dtype=np.float64)
        self.image_width = image_width
        self.image_height = image_height

    def get_k(self):
        return self._k

    def get_dist(self):
        return np.zeros(5, dtype=np.float32)


class TestGetKForFrame(unittest.TestCase):
    """The canvas-rescale fix: pixel measurements on a stream of a different
    size than the calibration canvas must be paired with a rescaled K."""

    CALIB_K = [[2066.0, 0.0, 1199.6], [0.0, 2066.0, 810.2], [0.0, 0.0, 1.0]]

    def _mount(self, w, h):
        cam = _FakeCam(self.CALIB_K, image_width=w, image_height=h)
        return CameraMount(name="t", camera=cam,
                           gimbal=FixedGimbal(GimbalData(att=Attitude(0, 0, 0))))

    def test_scales_fx_cx_fy_cy_by_axis_ratio(self):
        # calib 2560x1440, stream 1920x1080 -> uniform 0.75 on both axes
        mount = self._mount(2560, 1440)
        k = mount.get_k_for_frame(1920, 1080)
        sx, sy = 1920 / 2560, 1080 / 1440
        self.assertAlmostEqual(k[0, 0], 2066.0 * sx)   # fx
        self.assertAlmostEqual(k[0, 2], 1199.6 * sx)   # cx
        self.assertAlmostEqual(k[1, 1], 2066.0 * sy)   # fy
        self.assertAlmostEqual(k[1, 2], 810.2 * sy)    # cy

    def test_non_square_scale_uses_independent_axes(self):
        # calib 2000x1000, stream 1000x800 -> sx=0.5, sy=0.8 (must differ)
        mount = self._mount(2000, 1000)
        k = mount.get_k_for_frame(1000, 800)
        self.assertAlmostEqual(k[0, 0], 2066.0 * 0.5)
        self.assertAlmostEqual(k[1, 1], 2066.0 * 0.8)

    def test_equal_dims_returns_k_unchanged(self):
        mount = self._mount(1920, 1080)
        np.testing.assert_array_almost_equal(
            mount.get_k_for_frame(1920, 1080), np.array(self.CALIB_K))

    def test_missing_calib_dims_returns_raw_k(self):
        cam = _FakeCam(self.CALIB_K, image_width=None, image_height=None)
        mount = CameraMount(name="t", camera=cam,
                            gimbal=FixedGimbal(GimbalData(att=Attitude(0, 0, 0))))
        np.testing.assert_array_almost_equal(
            mount.get_k_for_frame(1920, 1080), np.array(self.CALIB_K))

    def test_nonpositive_frame_dims_returns_raw_k(self):
        mount = self._mount(2560, 1440)
        np.testing.assert_array_almost_equal(
            mount.get_k_for_frame(0, 1080), np.array(self.CALIB_K))
        np.testing.assert_array_almost_equal(
            mount.get_k_for_frame(1920, -1), np.array(self.CALIB_K))

    def test_does_not_mutate_source_k(self):
        mount = self._mount(2560, 1440)
        before = mount.get_k().copy()
        mount.get_k_for_frame(1920, 1080)
        np.testing.assert_array_equal(mount.get_k(), before)


if __name__ == '__main__':
    unittest.main()
