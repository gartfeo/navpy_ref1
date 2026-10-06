import math
import threading
import time
from unittest.mock import Mock, patch

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount import CameraMountOptics
from navpy.modules.vision import vision_profiles
from navpy.modules.vision.gimbal_telemetry_publisher import GimbalTelemetryPublisher
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class FakeMount:
    def __init__(self, gimbal_data=None, *, optics=None, exc=None):
        self._gimbal_data = gimbal_data
        self._optics = optics
        self._exc = exc

    def get_gimbal_data(self):
        if self._exc:
            raise self._exc
        return self._gimbal_data

    def get_live_optics(self):
        return self._optics


class FakeVehicle:
    def __init__(self, *, fail_first_send=False):
        self.messages = []
        self.sent = []
        self._fail_first_send = fail_first_send
        self._send_count = 0

    def send_mavlink_message(self, msg, *, source_component=None):
        self._send_count += 1
        if self._fail_first_send and self._send_count == 1:
            raise RuntimeError("send failed")
        self.messages.append(msg)
        self.sent.append((msg, source_component))


def _spec(mount, device_id):
    return vision_profiles.CameraMountSpec(
        mount=mount,
        device={},
        gimbal_device_id=device_id,
        profile_device_index=device_id - 1,
    )


def _assert_quat_close(actual, expected):
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert a == pytest.approx(e, abs=1e-9)


def test_publish_once_sends_gimbal_device_attitude_status():
    vehicle = FakeVehicle()
    mount = FakeMount(GimbalData(att=Attitude(0, 0, 0)))
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(mount, 3)], Mock())

    publisher.publish_once()

    assert len(vehicle.messages) == 1
    msg = vehicle.messages[0]
    assert msg.get_type() == "GIMBAL_DEVICE_ATTITUDE_STATUS"
    assert msg.gimbal_device_id == 3
    assert msg.flags == mav.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME
    _assert_quat_close(msg.q, [1.0, 0.0, 0.0, 0.0])
    assert math.isnan(msg.angular_velocity_x)
    assert math.isnan(msg.angular_velocity_y)
    assert math.isnan(msg.angular_velocity_z)
    assert msg.failure_flags == 0
    assert math.isnan(msg.delta_yaw)
    assert math.isnan(msg.delta_yaw_velocity)


def test_publish_once_sends_live_camera_optics_with_camera_component():
    vehicle = FakeVehicle()
    mount = FakeMount(
        GimbalData(att=Attitude(0, 0, 0)),
        optics=CameraMountOptics(
            zoom_command="2.50",
            zoom_level="2.60",
            sample_id="s1",
            fov_h_rad=0.4,
            fov_v_rad=0.2,
        ),
    )
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(mount, 2)], Mock())

    publisher.publish_once()

    assert [msg.get_type() for msg in vehicle.messages] == [
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        "CAMERA_FOV_STATUS",
        "CAMERA_SETTINGS",
    ]
    assert vehicle.sent[0][1] is None
    assert vehicle.sent[1][1] == mav.MAV_COMP_ID_CAMERA2
    assert vehicle.sent[2][1] == mav.MAV_COMP_ID_CAMERA2
    assert vehicle.messages[1].hfov == pytest.approx(math.degrees(0.4))
    assert vehicle.messages[1].vfov == pytest.approx(math.degrees(0.2))
    assert vehicle.messages[1].q == vehicle.messages[0].q
    assert vehicle.messages[2].zoomLevel == pytest.approx(2.6)


def test_publish_once_skips_camera_settings_without_numeric_zoom():
    vehicle = FakeVehicle()
    mount = FakeMount(
        GimbalData(att=Attitude(0, 0, 0)),
        optics=CameraMountOptics(
            zoom_command=None,
            zoom_level=None,
            sample_id="s1",
            fov_h_rad=0.4,
            fov_v_rad=0.2,
        ),
    )
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(mount, 1)], Mock())

    publisher.publish_once()

    assert [msg.get_type() for msg in vehicle.messages] == [
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        "CAMERA_FOV_STATUS",
    ]


def test_publish_once_does_not_republish_same_optics_sample():
    vehicle = FakeVehicle()
    mount = FakeMount(
        GimbalData(att=Attitude(0, 0, 0)),
        optics=CameraMountOptics(
            zoom_command="2.0",
            zoom_level="2.0",
            sample_id="same",
            fov_h_rad=0.4,
            fov_v_rad=0.2,
        ),
    )
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(mount, 1)], Mock())

    publisher.publish_once()
    publisher.publish_once()

    assert [msg.get_type() for msg in vehicle.messages] == [
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
        "CAMERA_FOV_STATUS",
        "CAMERA_SETTINGS",
        "GIMBAL_DEVICE_ATTITUDE_STATUS",
    ]


def test_publish_once_republishes_camera_sourced_optics_each_tick():
    """Camera-sourced (static, fixed-mount) optics carry a constant sample_id but
    must be re-published every tick so the GCS optics stays fresh."""
    vehicle = FakeVehicle()
    mount = FakeMount(
        GimbalData(att=Attitude(0, 0, 0)),
        optics=CameraMountOptics(
            zoom_command="1",
            zoom_level="1",
            sample_id="cam:1",
            fov_h_rad=0.4,
            fov_v_rad=0.2,
            sample_from_hardware=False,
        ),
    )
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(mount, 1)], Mock())

    publisher.publish_once()
    publisher.publish_once()

    types = [msg.get_type() for msg in vehicle.messages]
    assert types.count("CAMERA_FOV_STATUS") == 2   # re-sent both ticks (no dedup)
    assert types.count("CAMERA_SETTINGS") == 2


def test_publish_once_republishes_new_optics_sample():
    vehicle = FakeVehicle()
    mount = FakeMount(GimbalData(att=Attitude(0, 0, 0)))
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(mount, 1)], Mock())

    mount._optics = CameraMountOptics("1.0", "1.0", "a", 0.4, 0.2)
    publisher.publish_once()
    mount._optics = CameraMountOptics("1.0", "1.0", "b", 0.4, 0.2)
    publisher.publish_once()

    assert [msg.get_type() for msg in vehicle.messages].count("CAMERA_FOV_STATUS") == 2


def test_quaternion_uses_gimbal_sequence():
    vehicle = FakeVehicle()
    g_data = GimbalData(att=Attitude(0, 90, 0), g_seq="ZYX")
    publisher = GimbalTelemetryPublisher(vehicle, [_spec(FakeMount(g_data), 1)], Mock())

    publisher.publish_once()

    expected = [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    _assert_quat_close(vehicle.messages[0].q, expected)


def test_raw_profile_index_fallback_survives_skipped_devices():
    profile = {
        "devices": [
            {
                "name": "no_gimbal",
                "camera": {
                    "image_width": 640,
                    "image_height": 480,
                    "intrinsics": {
                        "zooms": {"1": {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}}
                    },
                },
            },
            {
                "name": "publishable",
                "camera": {
                    "image_width": 640,
                    "image_height": 480,
                    "intrinsics": {
                        "zooms": {"1": {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}}
                    },
                },
                "gimbal": {"camera_pitch": -30},
            },
        ]
    }
    logger = Mock()
    specs = vision_profiles.build_camera_mounts(profile, Mock(), logger, use_sim_gimbal=False)
    vehicle = FakeVehicle()
    publisher = GimbalTelemetryPublisher(vehicle, specs, logger)

    publisher.publish_once()

    assert len(specs) == 1
    assert vehicle.messages[0].gimbal_device_id == 2


def test_publish_once_emits_camera_fov_for_real_fixed_mount():
    """End-to-end (NavPy side): a REAL fixed mount (FixedGimbal + camera
    intrinsics) publishes CAMERA_FOV_STATUS each tick with FOV derived from the
    camera's intrinsics, so the GCS can render its footprint."""
    profile = {
        "devices": [
            {
                "name": "fixed_cam",
                "camera": {
                    "image_width": 1920,
                    "image_height": 1080,
                    "intrinsics": {
                        "zooms": {"1": {"fx": 2000.0, "fy": 2000.0, "cx": 960.0, "cy": 540.0}}
                    },
                },
                "gimbal": {"camera_pitch": -30},
            },
        ]
    }
    logger = Mock()
    specs = vision_profiles.build_camera_mounts(profile, Mock(), logger, use_sim_gimbal=False)
    vehicle = FakeVehicle()
    publisher = GimbalTelemetryPublisher(vehicle, specs, logger)

    publisher.publish_once()
    publisher.publish_once()

    fov_msgs = [m for m in vehicle.messages if m.get_type() == "CAMERA_FOV_STATUS"]
    assert len(fov_msgs) == 2   # static optics re-published every tick
    assert fov_msgs[0].hfov == pytest.approx(
        math.degrees(2 * math.atan(1920 / (2 * 2000.0))))
    assert fov_msgs[0].vfov == pytest.approx(
        math.degrees(2 * math.atan(1080 / (2 * 2000.0))))


def test_explicit_gimbal_device_id_publishes_from_mount_path():
    profile = {
        "devices": [
            {
                "name": "explicit",
                "camera": {
                    "image_width": 640,
                    "image_height": 480,
                    "intrinsics": {
                        "zooms": {"1": {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}}
                    },
                },
                "gimbal": {"camera_pitch": -30, "gimbal_device_id": 4},
            },
        ]
    }
    logger = Mock()
    specs = vision_profiles.build_camera_mounts(profile, Mock(), logger, use_sim_gimbal=False)
    vehicle = FakeVehicle()
    publisher = GimbalTelemetryPublisher(vehicle, specs, logger)

    publisher.publish_once()

    assert vehicle.messages[0].gimbal_device_id == 4


def test_publish_once_isolates_mount_read_failure():
    vehicle = FakeVehicle()
    good = FakeMount(GimbalData(att=Attitude(0, 0, 0)))
    bad = FakeMount(exc=RuntimeError("read failed"))
    publisher = GimbalTelemetryPublisher(
        vehicle,
        [_spec(bad, 1), _spec(good, 2)],
        Mock(),
    )

    publisher.publish_once()

    assert len(vehicle.messages) == 1
    assert vehicle.messages[0].gimbal_device_id == 2


def test_stop_preserves_thread_handle_when_join_times_out():
    class AliveThread:
        def __init__(self):
            self.join_timeout = None

        def join(self, timeout=None):
            self.join_timeout = timeout

        def is_alive(self):
            return True

    logger = Mock()
    publisher = GimbalTelemetryPublisher(FakeVehicle(), [], logger)
    thread = AliveThread()
    publisher._thread = thread

    stopped = publisher.stop()

    assert stopped is False
    assert publisher._thread is thread
    assert thread.join_timeout == 2.0
    logger.warning.assert_called_once()


def test_thread_start_failure_does_not_poison_stop_retry():
    failure = RuntimeError("publisher thread start failed")
    thread = Mock()
    thread.ident = None
    thread.start.side_effect = failure
    thread.is_alive.return_value = False
    publisher = GimbalTelemetryPublisher(FakeVehicle(), [], Mock())

    with patch(
        "navpy.modules.vision.gimbal_telemetry_publisher.threading.Thread",
        return_value=thread,
    ):
        with pytest.raises(RuntimeError) as raised:
            publisher.start()

    assert raised.value is failure
    assert publisher._thread is None
    assert publisher.stop() is True
    thread.join.assert_not_called()


def test_delayed_bootstrap_cannot_publish_after_restart():
    release_first = threading.Event()
    second_started = threading.Event()
    thread_ids: set[int] = set()
    publisher = GimbalTelemetryPublisher(FakeVehicle(), [], Mock(), rate_hz=100.0)

    def publish_once():
        thread_ids.add(threading.get_ident())
        second_started.set()

    publisher.publish_once = publish_once
    original_start = threading.Thread.start
    start_count = 0
    launcher = None

    def delay_first_start(thread):
        nonlocal start_count, launcher
        start_count += 1
        if start_count != 1:
            return original_start(thread)

        def launch_later():
            assert release_first.wait(timeout=1.0)
            original_start(thread)

        launcher = threading.Thread(target=launch_later, daemon=True)
        original_start(launcher)
        return None

    with patch.object(threading.Thread, "start", delay_first_start):
        publisher.start()
        assert publisher.stop() is True
        publisher.start()
        assert second_started.wait(timeout=1.0)
        release_first.set()
        launcher.join(timeout=1.0)
        time.sleep(0.03)
        assert publisher.stop() is True

    assert len(thread_ids) == 1


def test_publisher_worker_failure_is_persistent_health_failure():
    failure = RuntimeError("publisher worker failed")
    publisher = GimbalTelemetryPublisher(FakeVehicle(), [], Mock())
    publisher._run = Mock(side_effect=failure)

    publisher.start()
    thread = publisher._thread
    assert thread is not None
    thread.join(timeout=1.0)

    for _ in range(2):
        with pytest.raises(RuntimeError) as raised:
            publisher.raise_if_failed()
        assert raised.value is failure
    assert publisher.stop() is True


def test_publish_once_isolates_send_failure():
    vehicle = FakeVehicle(fail_first_send=True)
    g_data = GimbalData(att=Attitude(0, 0, 0))
    publisher = GimbalTelemetryPublisher(
        vehicle,
        [_spec(FakeMount(g_data), 1), _spec(FakeMount(g_data), 2)],
        Mock(),
    )

    publisher.publish_once()

    assert len(vehicle.messages) == 1
    assert vehicle.messages[0].gimbal_device_id == 2
