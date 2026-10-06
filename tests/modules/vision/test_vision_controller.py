"""Tests for VisionController."""
import threading
import unittest
from unittest.mock import Mock, patch

from navpy.exception_groups import ExceptionGroup
from navpy.modules.common.models.attitude import Attitude
from tests.conftest import create_mock_args


def _create_mock_vehicle():
    """Create mock vehicle with required attributes."""
    vehicle = Mock()
    vehicle.attitude = Attitude(0, 0, 0)
    vehicle.home_location = None
    vehicle.mission_items_count = 0
    # NavigationTargetArgs uses get_param_or_default
    vehicle.get_param_or_default = Mock(side_effect=lambda name, default: default)
    vehicle.source_system = 1
    return vehicle


class TestVisionControllerIntegration(unittest.TestCase):
    """Integration tests for VisionController using real profiles."""

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        self.logger = Mock()
        self.args = create_mock_args()

    def _create_vision_args(self, profile_name="c720hd", detector_type="sim"):
        """Create mock VisionArgs."""
        vision_args = Mock()
        vision_args.profile_name = profile_name
        vision_args.detector_type = detector_type
        vision_args.model_path = ""
        vision_args.debug_show = False
        return vision_args

    def test_init_with_default_profile(self):
        """VisionController initializes with default profile."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args(profile_name="")

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertIsNotNone(controller.profile_name)
        self.assertTrue(len(controller.mounts) > 0)

    def test_init_with_named_profile(self):
        """VisionController initializes with named profile."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args(profile_name="c720hd")

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertEqual(controller.profile_name, "c720hd")

    def test_init_creates_mounts(self):
        """VisionController creates camera mounts."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args()

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertTrue(len(controller.mounts) > 0)
        self.assertIsNotNone(controller.mounts[0].name)

    def test_init_creates_detectors(self):
        """VisionController creates one detector per mount."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args()

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertEqual(len(controller.detectors), len(controller.mounts))

    def test_init_creates_coordinator(self):
        """VisionController creates DetectionCoordinator."""
        from navpy.modules.vision.vision_controller import VisionController
        from navpy.modules.vision.detection_coordinator import DetectionCoordinator

        vision_args = self._create_vision_args()

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertIsNotNone(controller.coordinator)
        self.assertIsInstance(controller.coordinator, DetectionCoordinator)

    def test_detector_property_returns_coordinator(self):
        """detector property returns coordinator for NavController compatibility."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args()

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertEqual(controller.detector, controller.coordinator)

    def test_geo_ref_created(self):
        """VisionController creates GeoRefCalc."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args()

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertIsNotNone(controller.geo_ref)

    def test_siyi_zr10_detector_exposes_zoom_status(self):
        """The SIYI profile exposes read-only zoom status."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args(profile_name="siyi_zr10")
        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        detector = controller.detectors[0]
        self.assertIsNotNone(detector.get_zoom_result())

    def test_c720hd_detector_has_no_zoom_status(self):
        """A fixed camera without zoom exposes no zoom result."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args(profile_name="c720hd")
        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        detector = controller.detectors[0]
        self.assertIsNone(detector.get_zoom_result())

    def test_invalid_profile_raises(self):
        """VisionController raises for invalid profile name."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = self._create_vision_args(profile_name="nonexistent_xyz")

        with self.assertRaises(ValueError) as ctx:
            VisionController(
                self.vehicle, self.args, vision_args, self.logger
            )

        self.assertIn("not found", str(ctx.exception))


class TestVisionControllerMultiCamera(unittest.TestCase):
    """Tests for VisionController with multi-camera profiles."""

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        self.logger = Mock()
        self.args = create_mock_args()

    def test_dual_camera_profile(self):
        """VisionController creates two mounts for dual camera profile."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = Mock()
        vision_args.profile_name = "novoxy_dual"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertEqual(len(controller.mounts), 2)
        self.assertEqual(len(controller.detectors), 2)

    def test_dual_camera_mount_names(self):
        """Multi-camera mounts have distinct names."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = Mock()
        vision_args.profile_name = "novoxy_dual"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        names = [m.name for m in controller.mounts]
        self.assertEqual(len(names), len(set(names)))  # All unique


def _vision_mount_double(name="mount"):
    mount = Mock()
    mount.name = name
    mount.stop.return_value = True
    return mount


def _vision_reader_double():
    reader = Mock()
    reader.stop.return_value = True
    reader.is_quiescent = True
    return reader


class TestVisionControllerLifecycle(unittest.TestCase):
    """Tests for VisionController start/stop/refresh."""

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        self.logger = Mock()
        self.args = create_mock_args()

    def _create_controller(self):
        """Helper to create controller."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = Mock()
        vision_args.profile_name = "c720hd"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        return VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

    def test_start_and_stop(self):
        """Lifecycle starts and stops every exact owner."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double()
        publisher = _vision_reader_double()
        coordinator = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        lifecycle.start()
        self.assertTrue(lifecycle.stop())

        mount.start.assert_called_once()
        mount.stop.assert_called_once()
        publisher.start.assert_called_once()
        publisher.stop.assert_called_once()
        coordinator.start.assert_called_once()
        coordinator.stop.assert_called_once()

    def test_running_lifecycle_is_not_reported_quiescent(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double()
        publisher = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, _vision_reader_double(), None, Mock()
        )

        lifecycle.start()
        self.assertFalse(lifecycle.is_quiescent)
        self.assertTrue(lifecycle.stop())
        self.assertTrue(lifecycle.is_quiescent)

    def test_start_stop_order_keeps_publisher_between_mounts_and_vehicle(self):
        """Publisher starts after mounts and stops before mounts."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []

        class Ordered:
            def __init__(self, name):
                self.name = name

            def start(self):
                order.append(f"{self.name}.start")

            def stop(self):
                order.append(f"{self.name}.stop")
                return True

            @property
            def is_quiescent(self):
                return True

            def refresh(self):
                order.append(f"{self.name}.refresh")

        lifecycle = VisionLifecycle(
            [Ordered("mount")],
            Ordered("publisher"),
            Ordered("coordinator"),
            None,
            Mock(),
        )

        lifecycle.start()
        lifecycle.stop()

        self.assertEqual(
            order,
            [
                "mount.start",
                "publisher.start",
                "coordinator.start",
                "coordinator.stop",
                "publisher.stop",
                "mount.stop",
            ],
        )

    def test_start_failure_rolls_back_attempted_components_in_reverse(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []
        start_error = RuntimeError("second start failed")

        def component(name, failure=None):
            value = Mock()
            value.name = name

            def start():
                order.append(f"{name}.start")
                if failure is not None:
                    raise failure

            value.start.side_effect = start
            value.stop.side_effect = lambda: order.append(f"{name}.stop") or True
            value.is_quiescent = True
            return value

        lifecycle = VisionLifecycle(
            [component("first"), component("second", start_error)],
            component("publisher"),
            component("coordinator"),
            None,
            Mock(),
        )

        with self.assertRaises(RuntimeError) as raised:
            lifecycle.start()

        self.assertIs(raised.exception, start_error)
        self.assertEqual(
            order,
            ["first.start", "second.start", "second.stop", "first.stop"],
        )

    def test_start_preserves_original_and_cleanup_failures(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []
        start_error = RuntimeError("coordinator start")
        cleanup_error = RuntimeError("publisher rollback")
        mount = _vision_mount_double()
        mount.start.side_effect = lambda: order.append("mount.start")
        mount.stop.side_effect = lambda: order.append("mount.stop") or True
        publisher = _vision_reader_double()
        publisher.is_quiescent = False
        publisher.start.side_effect = lambda: order.append("publisher.start")
        publisher_stop_attempts = 0

        def fail_publisher_stop():
            nonlocal publisher_stop_attempts
            publisher_stop_attempts += 1
            order.append("publisher.stop")
            if publisher_stop_attempts == 1:
                raise cleanup_error
            publisher.is_quiescent = True
            return True

        publisher.stop.side_effect = fail_publisher_stop
        coordinator = _vision_reader_double()

        def fail_coordinator_start():
            order.append("coordinator.start")
            raise start_error

        coordinator.start.side_effect = fail_coordinator_start
        coordinator.stop.side_effect = (
            lambda: order.append("coordinator.stop") or True
        )
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        with self.assertRaises(ExceptionGroup) as raised:
            lifecycle.start()

        self.assertEqual(raised.exception.exceptions, (start_error, cleanup_error))
        self.assertEqual(
            order,
            [
                "mount.start",
                "publisher.start",
                "coordinator.start",
                "coordinator.stop",
                "publisher.stop",
            ],
        )
        self.assertTrue(lifecycle.stop())
        self.assertEqual(order[-2:], ["publisher.stop", "mount.stop"])

    def test_failed_start_requires_controller_reconstruction(self):
        """A rolled-back Thread-backed mount is deliberately not restarted."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double("thread-backed")
        publisher = _vision_reader_double()
        publisher.start.side_effect = RuntimeError("publisher failed")
        lifecycle = VisionLifecycle(
            [mount], publisher, Mock(), None, Mock()
        )

        with self.assertRaisesRegex(RuntimeError, "publisher failed"):
            lifecycle.start()
        with self.assertRaisesRegex(RuntimeError, "reconstruct the controller"):
            lifecycle.start()

        mount.start.assert_called_once_with()
        mount.stop.assert_called_once_with()

    def test_failed_start_retains_only_incomplete_rollback_for_stop_retry(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        start_error = RuntimeError("publisher failed")
        mount = _vision_mount_double("thread-backed")
        mount.stop.side_effect = [False, True]
        publisher = _vision_reader_double()
        publisher.start.side_effect = start_error
        publisher.stop.return_value = True
        debug = Mock()
        lifecycle = VisionLifecycle(
            [mount], publisher, Mock(), debug, Mock()
        )

        with self.assertRaises(ExceptionGroup) as raised:
            lifecycle.start()
        self.assertIs(raised.exception.exceptions[0], start_error)

        self.assertIs(lifecycle.stop(), True)
        self.assertEqual(mount.stop.call_count, 2)
        publisher.stop.assert_called_once_with()
        debug.close.assert_called_once_with()

    def test_keyboard_interrupt_during_start_does_not_deadlock_cleanup(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double("interrupting")
        mount.start.side_effect = KeyboardInterrupt()
        lifecycle = VisionLifecycle(
            [mount], _vision_reader_double(), _vision_reader_double(), None, Mock()
        )

        with self.assertRaises(KeyboardInterrupt):
            lifecycle.start()

        self.assertIs(lifecycle.stop(), True)
        mount.stop.assert_called_once_with()

    def test_stop_reports_unsafe_when_coordinator_stop_raises(self):
        """Sibling readers stop, but mounts wait for coordinator quiescence."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []

        class FailingCoordinator:
            is_quiescent = False

            def start(self):
                pass

            def stop(self):
                order.append("coordinator.stop")
                raise RuntimeError("coordinator failed")

            def refresh(self):
                pass

        class Ordered:
            def __init__(self, name):
                self.name = name

            def stop(self):
                order.append(f"{self.name}.stop")
                return True

            @property
            def is_quiescent(self):
                return True

            def start(self):
                pass

            def refresh(self):
                pass

        error = Mock()
        lifecycle = VisionLifecycle(
            [Ordered("mount")],
            Ordered("publisher"),
            FailingCoordinator(),
            None,
            error,
        )

        with self.assertRaisesRegex(RuntimeError, "coordinator failed"):
            lifecycle.stop()

        self.assertEqual(
            order,
            ["coordinator.stop", "publisher.stop"],
        )
        error.assert_called_once()

    def test_stop_reports_unsafe_close_when_mount_stop_raises(self):
        """Unknown mount quiescence keeps the vehicle connection open."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        class FailingMount:
            name = "mount"

            def start(self):
                pass

            def stop(self):
                raise RuntimeError("mount failed")

            def refresh(self):
                pass

        publisher = _vision_reader_double()
        error = Mock()
        lifecycle = VisionLifecycle(
            [FailingMount()], publisher, _vision_reader_double(), None, error
        )

        with self.assertRaisesRegex(RuntimeError, "mount failed"):
            lifecycle.stop()
        error.assert_called_once()

    def test_stop_reports_mount_timeout_as_unsafe(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double("thread-backed")
        mount.stop.return_value = False
        publisher = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, _vision_reader_double(), None, Mock()
        )

        self.assertIs(lifecycle.stop(), False)

    def test_stop_retries_only_components_that_are_not_yet_quiescent(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double("thread-backed")
        mount.stop.side_effect = [False, True]
        publisher = _vision_reader_double()
        coordinator = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        self.assertIs(lifecycle.stop(), False)
        self.assertIs(lifecycle.stop(), True)

        self.assertEqual(mount.stop.call_count, 2)
        publisher.stop.assert_called_once_with()
        coordinator.stop.assert_called_once_with()

    def test_none_reader_stop_cannot_release_mount_dependencies(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double("owned-mount")
        publisher = _vision_reader_double()
        coordinator = _vision_reader_double()
        coordinator.stop.return_value = None
        coordinator.is_quiescent = False
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        with self.assertRaisesRegex(TypeError, "stop.*return bool"):
            lifecycle.stop()

        mount.stop.assert_not_called()
        coordinator.stop.return_value = True
        coordinator.is_quiescent = True
        self.assertTrue(lifecycle.stop())
        mount.stop.assert_called_once_with()

    def test_quiescent_coordinator_cleanup_error_does_not_gate_mounts(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        cleanup_error = RuntimeError("detector resource close failed")

        class Coordinator:
            is_quiescent = True

            def __init__(self):
                self.stop_calls = 0

            def start(self):
                pass

            def stop(self):
                self.stop_calls += 1
                if self.stop_calls == 1:
                    raise cleanup_error
                return True

            def refresh(self):
                pass

        coordinator = Coordinator()
        mount = _vision_mount_double()
        publisher = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        with self.assertRaises(RuntimeError) as first:
            lifecycle.stop()
        self.assertIs(first.exception, cleanup_error)
        self.assertTrue(lifecycle.is_quiescent)
        mount.stop.assert_called_once_with()

        self.assertTrue(lifecycle.stop())
        self.assertEqual(coordinator.stop_calls, 2)
        mount.stop.assert_called_once_with()

    def test_keyboard_interrupt_during_stop_commits_retry_state_before_reraise(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double("interrupting")
        interrupt = KeyboardInterrupt()
        mount.stop.side_effect = [interrupt, True]
        publisher = _vision_reader_double()
        coordinator = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        with self.assertRaises(KeyboardInterrupt) as raised:
            lifecycle.stop()
        self.assertIs(raised.exception, interrupt)
        self.assertIs(lifecycle.stop(), True)

        self.assertEqual(mount.stop.call_count, 2)
        publisher.stop.assert_called_once_with()
        coordinator.stop.assert_called_once_with()

    def test_stop_retains_mounts_until_publisher_quiesces(self):
        """A live publisher keeps its mount dependencies owned for retry."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []

        class Ordered:
            def __init__(self, name, result=True):
                self.name = name
                self.result = result

            def stop(self):
                order.append(f"{self.name}.stop")
                return self.result

            @property
            def is_quiescent(self):
                return self.result is True

            def start(self):
                pass

            def refresh(self):
                pass

        error = Mock()
        mount = Ordered("mount")
        publisher = Ordered("publisher", result=False)
        lifecycle = VisionLifecycle(
            [mount],
            publisher,
            Ordered("coordinator"),
            None,
            error,
        )

        stopped = lifecycle.stop()

        self.assertFalse(stopped)
        self.assertEqual(order, ["coordinator.stop", "publisher.stop"])
        error.assert_called_once()

        publisher.result = True
        self.assertTrue(lifecycle.stop())
        self.assertEqual(
            order,
            [
                "coordinator.stop",
                "publisher.stop",
                "publisher.stop",
                "mount.stop",
            ],
        )

    def test_stop_retains_publisher_and_mounts_until_coordinator_quiesces(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []

        class Ordered:
            def __init__(self, name, result=True):
                self.name = name
                self.result = result

            def stop(self):
                order.append(f"{self.name}.stop")
                return self.result

            @property
            def is_quiescent(self):
                return self.result is True

            def start(self):
                pass

            def refresh(self):
                pass

        mount = Ordered("mount")
        publisher = Ordered("publisher")
        coordinator = Ordered("coordinator", result=False)
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        self.assertFalse(lifecycle.stop())
        self.assertEqual(order, ["coordinator.stop", "publisher.stop"])

        coordinator.result = True
        self.assertTrue(lifecycle.stop())
        self.assertEqual(
            order,
            [
                "coordinator.stop",
                "publisher.stop",
                "coordinator.stop",
                "mount.stop",
            ],
        )

    def test_start_and_stop_are_exactly_once_across_repeated_calls(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double()
        publisher = _vision_reader_double()
        coordinator = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, None, Mock()
        )

        lifecycle.start()
        lifecycle.start()
        self.assertTrue(lifecycle.stop())
        self.assertTrue(lifecycle.stop())

        mount.start.assert_called_once_with()
        mount.stop.assert_called_once_with()
        publisher.start.assert_called_once_with()
        publisher.stop.assert_called_once_with()
        coordinator.start.assert_called_once_with()
        coordinator.stop.assert_called_once_with()

    def test_concurrent_start_and_stop_each_execute_once(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        start_entered = threading.Event()
        release_start = threading.Event()
        stop_entered = threading.Event()
        release_stop = threading.Event()
        mount = _vision_mount_double()

        def start_mount():
            start_entered.set()
            self.assertTrue(release_start.wait(2.0))

        def stop_mount():
            stop_entered.set()
            self.assertTrue(release_stop.wait(2.0))
            return True

        mount.start.side_effect = start_mount
        mount.stop.side_effect = stop_mount
        publisher = _vision_reader_double()
        lifecycle = VisionLifecycle(
            [mount], publisher, _vision_reader_double(), None, Mock()
        )
        errors = []

        def call(action):
            try:
                action()
            except Exception as error:
                errors.append(error)

        start_callers = [
            threading.Thread(target=lambda: call(lifecycle.start))
            for _ in range(2)
        ]
        start_callers[0].start()
        self.assertTrue(start_entered.wait(1.0))
        start_callers[1].start()
        release_start.set()
        for caller in start_callers:
            caller.join(2.0)

        stop_callers = [
            threading.Thread(target=lambda: call(lifecycle.stop))
            for _ in range(2)
        ]
        stop_callers[0].start()
        self.assertTrue(stop_entered.wait(1.0))
        stop_callers[1].start()
        release_stop.set()
        for caller in stop_callers:
            caller.join(2.0)

        self.assertEqual(errors, [])
        mount.start.assert_called_once_with()
        mount.stop.assert_called_once_with()
        publisher.start.assert_called_once_with()
        publisher.stop.assert_called_once_with()

    def test_stop_retries_failed_cleanup_without_repeating_completed_cleanup(self):
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        mount = _vision_mount_double()
        mount_error = RuntimeError("mount failed")
        mount.stop.side_effect = [mount_error, True]
        publisher = _vision_reader_double()
        publisher.stop.side_effect = [False, True]
        coordinator = _vision_reader_double()
        coordinator_error = RuntimeError("coordinator failed")
        coordinator.stop.side_effect = [coordinator_error, True]
        debug = Mock()
        debug_error = RuntimeError("debug failed")
        debug.close.side_effect = [debug_error, None]
        error = Mock(side_effect=RuntimeError("error sink failed"))
        lifecycle = VisionLifecycle(
            [mount], publisher, coordinator, debug, error
        )

        with self.assertRaises(ExceptionGroup) as first:
            lifecycle.stop()
        self.assertEqual(
            first.exception.exceptions,
            (coordinator_error, debug_error),
        )
        with self.assertRaises(RuntimeError) as second:
            lifecycle.stop()
        self.assertIs(second.exception, mount_error)
        self.assertTrue(lifecycle.stop())

        self.assertEqual(coordinator.stop.call_count, 2)
        self.assertEqual(publisher.stop.call_count, 2)
        self.assertEqual(mount.stop.call_count, 2)
        self.assertEqual(debug.close.call_count, 2)

    def test_refresh(self):
        """Lifecycle refreshes mounts before the coordinator."""
        from navpy.modules.vision.vision_lifecycle import VisionLifecycle

        order = []
        mount = Mock()
        mount.name = "mount"
        mount.refresh.side_effect = lambda: order.append("mount.refresh")
        coordinator = Mock()
        coordinator.refresh.side_effect = lambda: order.append(
            "coordinator.refresh"
        )
        lifecycle = VisionLifecycle(
            [mount], Mock(), coordinator, None, Mock()
        )

        lifecycle.refresh()

        self.assertEqual(order, ["mount.refresh", "coordinator.refresh"])


class TestVisionControllerUiStep(unittest.TestCase):
    """Tests for VisionController ui_step method."""

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        self.logger = Mock()
        self.args = create_mock_args()

    def _create_controller(self):
        """Helper to create controller."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = Mock()
        vision_args.profile_name = "c720hd"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        return VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

    def test_ui_step_returns_true_for_sim(self):
        """ui_step returns True for sim detectors (no UI)."""
        controller = self._create_controller()

        result = controller.ui_step()

        self.assertTrue(result)

    def test_ui_step_with_mock_detector_exit(self):
        """ui_step returns False when detector signals exit."""
        from navpy.modules.vision.vision_sim_debug_view import VisionUiDispatcher

        mock_detector = Mock()
        mock_detector.ui_step.return_value = False
        dispatcher = VisionUiDispatcher([mock_detector], None)

        result = dispatcher.ui_step()

        self.assertFalse(result)
        mock_detector.ui_step.assert_called_once()

    def test_ui_dispatch_uses_only_the_explicit_ui_port(self):
        """Dispatch does not probe detector capabilities or implementation type."""
        from navpy.modules.vision.vision_sim_debug_view import VisionUiDispatcher

        class StrictUi:
            def ui_step(self):
                return True

            def __getattr__(self, name):
                raise AssertionError(f"unexpected capability probe: {name}")

        self.assertTrue(VisionUiDispatcher([StrictUi()], None).ui_step())


class TestVisionControllerSimDetector(unittest.TestCase):
    """Tests for VisionController sim detector creation."""

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        self.logger = Mock()
        self.args = create_mock_args()

    def test_creates_sim_detectors(self):
        """VisionController creates DetectorSim instances."""
        from navpy.modules.vision.vision_controller import VisionController
        from navpy.modules.vision.sim.detector_sim import DetectorSim

        vision_args = Mock()
        vision_args.profile_name = "c720hd"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        for detector in controller.detectors:
            self.assertIsInstance(detector, DetectorSim)

    def test_sim_detector_has_mount(self):
        """Sim detector has associated mount."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = Mock()
        vision_args.profile_name = "c720hd"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        for detector in controller.detectors:
            self.assertIsNotNone(detector.mount)
            self.assertTrue(len(detector.mounts) == 1)

    def test_ideal_360_profile_creates_lightweight_ideal_detector(self):
        """ideal_360 sim profile disables frame generation, tracking and zoom."""
        from navpy.modules.vision.vision_controller import VisionController

        vision_args = Mock()
        vision_args.profile_name = "ideal_360"
        vision_args.detector_type = "sim"
        vision_args.model_path = ""
        vision_args.debug_show = False

        controller = VisionController(
            self.vehicle, self.args, vision_args, self.logger
        )

        self.assertEqual(controller.profile_name, "ideal_360")
        self.assertEqual(len(controller.detectors), 1)
        detector = controller.detectors[0]
        self.assertTrue(detector.has_source_driven_detection_events)
        self.assertIsNone(detector.navigation)
        self.logger.info.assert_any_call(
            "DetectorSim(ideal_360): static ideal 360 enabled"
        )

    def test_sim_debug_snapshot_preserves_missing_intrinsics(self):
        """Missing K remains None so panel fallback geometry stays active."""
        from navpy.modules.vision.sim_detector_factory import SimDetectorFactory
        from navpy.modules.vision.vision_profiles import CameraMountSpec

        mount = Mock()
        mount.name = "main"
        mount.image_width = None
        mount.image_height = None
        mount.get_k.return_value = None
        detector = Mock()
        detector.source_name = "main"
        detector.get_latest_detections.return_value = []
        detector.rate_result = None
        detector.get_zoom_result.return_value = None
        detector.get_zoom_target_pixels.return_value = None
        factory = SimDetectorFactory(
            self.vehicle,
            self.args,
            self.logger,
            Mock(),
            {},
            None,
            None,
        )
        spec = CameraMountSpec(mount, {}, 1, 0)

        with patch(
            "navpy.modules.vision.sim_detector_factory.DetectorSim",
            return_value=detector,
        ):
            node = factory.create(spec, {}, 0)

        snapshot = node.sim_debug.read_snapshot()
        self.assertIsNone(snapshot.intrinsics)
        self.assertEqual(snapshot.image_width, 1920)
        self.assertEqual(snapshot.image_height, 1080)
        self.assertFalse(any(
            "static ideal 360 enabled" in str(call)
            for call in self.logger.info.call_args_list
        ))


class TestVisionControllerRealDetector(unittest.TestCase):
    """Tests for real detector creation settings."""

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        self.logger = Mock()
        self.args = create_mock_args()

    def test_create_real_detector_enables_target_lock(self):
        """Real detector creation explicitly enables target lock."""
        from navpy.modules.vision.real_detector_factory import RealDetectorFactory
        from navpy.modules.vision.vision_profiles import CameraMountSpec

        mount = Mock()
        mount.name = "main"
        mount_spec = CameraMountSpec(
            mount=mount,
            device={},
            gimbal_device_id=1,
            profile_device_index=0,
        )
        factory = RealDetectorFactory(
            self.vehicle,
            self.logger,
            {},
            "pyproject.toml",
            False,
        )

        detector_ctor = Mock(return_value=Mock())
        with patch("navpy.modules.vision.detector.Detector", detector_ctor):
            factory.create(mount_spec, {}, 0)

        dependencies, config = detector_ctor.call_args.args
        self.assertIs(dependencies.vehicle, self.vehicle)
        self.assertEqual(config.pipeline.output_mode, "all")
        self.assertTrue(config.pipeline.use_target_lock)


class TestVisionControllerAdapter(unittest.TestCase):
    def test_exact_public_adapter_delegates_through_one_port_bundle(self):
        from navpy.modules.vision.vision_controller import VisionController

        ports = Mock()
        ports.coordinator = Mock()
        ports.coordinator.coordination = Mock()
        ports.detectors = [Mock()]
        ports.mounts = [Mock()]
        ports.geo_ref = Mock()
        ports.profile_name = "profile"
        ports.profile = {"name": "profile"}
        ports.approach_kind = Mock()
        ports.lifecycle.stop.return_value = True
        with patch(
            "navpy.modules.vision.vision_controller.build_vision_controller",
            return_value=ports,
        ) as build:
            controller = VisionController(
                _create_mock_vehicle(),
                create_mock_args(),
                Mock(),
                Mock(),
            )

        self.assertEqual(vars(controller), {"_ports": ports})
        self.assertIs(controller.coordinator, ports.coordinator)
        self.assertIs(controller.coordination, ports.coordinator.coordination)
        self.assertIs(controller.detectors, ports.detectors)
        self.assertIs(controller.detector, ports.coordinator)
        self.assertIs(controller.mounts, ports.mounts)
        self.assertIs(controller.geo_ref, ports.geo_ref)
        self.assertEqual(controller.profile_name, "profile")
        self.assertIs(controller.profile, ports.profile)
        self.assertIs(controller.approach_kind, ports.approach_kind)
        controller.start()
        self.assertTrue(controller.stop())
        controller.refresh()
        self.assertTrue(controller.ui_step())
        ports.lifecycle.start.assert_called_once_with()
        ports.lifecycle.stop.assert_called_once_with()
        ports.lifecycle.refresh.assert_called_once_with()
        ports.ui.ui_step.assert_called_once_with()
        build.assert_called_once()


class TestVisionDetectorConstruction(unittest.TestCase):
    def test_ideal_sensor_rejects_real_detector_before_model_loading(self):
        from navpy.modules.vision.vision_controller_composition import (
            build_vision_controller,
        )
        from navpy.modules.vision.vision_profiles import CameraMountSpec

        mount = Mock(name="ideal_mount")
        profile = Mock(
            mount_specs=(CameraMountSpec(mount=mount, device={}),),
            detector_settings={"ideal_360": True},
        )
        vision_args = Mock(
            detector_type="real",
            model_path="irrelevant.pt",
            debug_show=False,
        )
        with patch(
            "navpy.modules.vision.vision_controller_composition.build_vision_profile",
            return_value=profile,
        ):
            with self.assertRaisesRegex(ValueError, "simulator-only"):
                build_vision_controller(
                    _create_mock_vehicle(),
                    Mock(),
                    vision_args,
                    Mock(),
                    None,
                    None,
                )

        mount.stop.assert_called_once_with()

    def test_factory_failure_rolls_back_prior_detectors_in_reverse(self):
        from navpy.modules.vision.vision_detector_factory_ports import (
            VisionDetectorNode,
            build_detector_nodes,
        )

        order = []
        failure = RuntimeError("third detector failed")

        class Factory:
            def create(self, mount_spec, detector_settings, mount_index):
                del mount_spec, detector_settings
                if mount_index == 2:
                    raise failure
                cleanup = Mock()
                cleanup.stop.side_effect = lambda: order.append(
                    f"detector-{mount_index}.stop"
                )
                return VisionDetectorNode(
                    detector=Mock(),
                    cleanup=cleanup,
                    real_ui=None,
                    sim_debug=None,
                )

        with self.assertRaises(RuntimeError) as raised:
            build_detector_nodes(Factory(), (Mock(), Mock(), Mock()), {})

        self.assertIs(raised.exception, failure)
        self.assertEqual(
            order,
            ["detector-1.stop", "detector-0.stop"],
        )

    def test_missing_real_model_rolls_back_profile_mounts(self):
        from navpy.modules.vision.vision_controller_composition import (
            build_vision_controller,
        )
        from navpy.modules.vision.vision_profiles import CameraMountSpec

        mount = Mock()
        mount.name = "main"
        mount_spec = CameraMountSpec(
            mount=mount,
            device={},
            gimbal_device_id=1,
            profile_device_index=0,
        )
        profile = Mock()
        profile.mount_specs = (mount_spec,)
        profile.profile = {}
        profile.detector_settings = {}
        vision_args = Mock()
        vision_args.detector_type = "real"
        vision_args.model_path = "missing-model-that-must-not-exist.pt"
        vision_args.debug_show = False

        with patch(
            "navpy.modules.vision.vision_controller_composition.build_vision_profile",
            return_value=profile,
        ):
            with self.assertRaisesRegex(ValueError, "does not exist"):
                build_vision_controller(
                    _create_mock_vehicle(),
                    Mock(),
                    vision_args,
                    Mock(),
                    None,
                    None,
                )

        mount.stop.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
