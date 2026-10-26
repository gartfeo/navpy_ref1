"""NavPy's camera components: heartbeats and camera-protocol answers.

docs/design/gcs_video_streams.md, section 1 and the test plan's NavPy unit
bullet; docs/design/gcs_preflight_camera_row.md for the detector readiness,
vendor and boot id. Video and camera status are operator display only; none
of this touches navigation.
"""

from __future__ import annotations

import threading
from types import MappingProxyType
from unittest.mock import Mock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav

from navpy.core.perception.profile.vision_profile_types import CameraMountSpec
from navpy.core.perception.vision_publisher_group import VisionPublisherGroup
from navpy.core.ports.camera_health import (
    NAVPY_CAMERA_VENDOR,
    NAVPY_SIM_CAMERA_VENDOR,
    CameraHealth,
)
from navpy.core.ports.subscription import Subscription
from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_HEARTBEAT_PERIOD_S,
)
from navpy.app.perception.camera_stream_composition import streaming_cameras
from navpy.plugins.vehicle.ardupilot import camera_stream_messages
from navpy.plugins.vehicle.ardupilot.camera_stream_publisher import CameraStreamPublisher

OWN = 42
GCS_SYSTEM = 255
GCS_COMPONENT = mav.MAV_COMP_ID_MISSIONPLANNER
WAIT_S = 5.0  # bounds a failing run only


def _stream(stream_id: int, name: str = "overlay") -> dict:
    return {
        "stream_id": stream_id,
        "name": name,
        "uri": f"rtsp://192.168.144.30:8554/s{stream_id}",
        "encoding": "H264" if stream_id == 1 else "H265",
        "resolution_h": 1280,
        "resolution_v": 720,
        "framerate": 30.0,
    }


def _spec(device_id: int, streams: list[dict] | None, name: str = "cam") -> CameraMountSpec:
    device: dict = {
        "name": name,
        "camera": {"image_width": 1920, "image_height": 1080},
    }
    if streams is not None:
        device["video_streams"] = streams
    return CameraMountSpec(
        mount=Mock(), device=device, gimbal_device_id=device_id,
        profile_device_index=device_id - 1,
    )


class FakeVehicle:
    def __init__(self) -> None:
        self.sent: list[tuple[mav.MAVLink_message, int | None]] = []
        self.callbacks: list = []
        self.sent_event = threading.Event()

    def send_mavlink_message(self, message, *, source_component=None) -> None:
        self.sent.append((message, source_component))
        self.sent_event.set()

    def on_message(self, message_name, callback) -> Subscription:
        assert message_name == "COMMAND_LONG"
        self.callbacks.append(callback)
        return Subscription(lambda: self.callbacks.remove(callback))


def _ready() -> bool:
    return True


def _publisher(specs, vehicle=None, health=None) -> tuple[CameraStreamPublisher, FakeVehicle]:
    vehicle = vehicle or FakeVehicle()
    if health is None:
        health = [CameraHealth(NAVPY_CAMERA_VENDOR, _ready) for _ in specs]
    return CameraStreamPublisher(
        vehicle, OWN, streaming_cameras(specs, health), Mock()
    ), vehicle


def _wire(message, system=GCS_SYSTEM, component=GCS_COMPONENT):
    sender = mav.MAVLink(None, srcSystem=system, srcComponent=component)
    return mav.MAVLink(None).decode(bytearray(message.pack(sender)))


def _request(message_id, *, param2=0.0, component=mav.MAV_COMP_ID_CAMERA, target=OWN):
    return _wire(mav.MAVLink_command_long_message(
        target, component, mav.MAV_CMD_REQUEST_MESSAGE, 0,
        message_id, param2, 0, 0, 0, 0, 0,
    ))


def _answer(publisher, vehicle, request):
    vehicle.sent.clear()
    publisher.answer(request)
    return [(message.get_type(), message, source) for message, source in vehicle.sent]


TWO_STREAMS = [_spec(1, [_stream(2, "raw"), _stream(1)]), _spec(2, None)]


def test_heartbeats_from_every_camera_mount_with_or_without_streams():
    publisher, vehicle = _publisher(
        [_spec(1, [_stream(1)]), _spec(2, None), _spec(3, [_stream(1)])]
    )

    publisher.send_heartbeats()

    assert [source for _, source in vehicle.sent] == [
        mav.MAV_COMP_ID_CAMERA, mav.MAV_COMP_ID_CAMERA2, mav.MAV_COMP_ID_CAMERA3,
    ]
    for heartbeat, _ in vehicle.sent:
        assert heartbeat.get_type() == "HEARTBEAT"
        assert heartbeat.type == mav.MAV_TYPE_CAMERA
        assert heartbeat.autopilot == mav.MAV_AUTOPILOT_INVALID


def test_heartbeat_status_is_each_mount_detector_readiness_at_send_time():
    first = {"ready": True}
    health = [
        CameraHealth(NAVPY_CAMERA_VENDOR, lambda: first["ready"]),
        CameraHealth(NAVPY_CAMERA_VENDOR, lambda: False),
    ]
    publisher, vehicle = _publisher([_spec(1, [_stream(1)]), _spec(2, None)], health=health)

    publisher.send_heartbeats()
    first["ready"] = False
    publisher.send_heartbeats()

    statuses = [
        _wire(message, OWN, source).system_status for message, source in vehicle.sent
    ]
    assert statuses == [
        mav.MAV_STATE_ACTIVE, mav.MAV_STATE_CRITICAL,
        mav.MAV_STATE_CRITICAL, mav.MAV_STATE_CRITICAL,
    ]


def test_heartbeats_carry_one_boot_id_per_publisher_in_custom_mode(monkeypatch):
    boot_ids = iter([0xFFFFFFFF, 7])
    monkeypatch.setattr(
        "navpy.plugins.vehicle.ardupilot.camera_stream_publisher.new_camera_boot_id",
        lambda: next(boot_ids),
    )
    first, first_vehicle = _publisher([_spec(1, None), _spec(2, None)])
    second, second_vehicle = _publisher([_spec(1, None)])

    first.send_heartbeats()
    first.send_heartbeats()
    second.send_heartbeats()

    assert [
        _wire(message, OWN, source).custom_mode for message, source in first_vehicle.sent
    ] == [0xFFFFFFFF] * 4  # uint32 range survives encoding
    assert [message.custom_mode for message, _ in second_vehicle.sent] == [7]


def test_boot_id_is_drawn_from_the_runtime_rng_and_never_zero(monkeypatch):
    calls = []

    class Rng:
        def randint(self, low, high):
            calls.append((low, high))
            return low

    monkeypatch.setattr(camera_stream_messages.rt, "rng", lambda: Rng())

    assert camera_stream_messages.new_camera_boot_id() == 1
    assert calls == [(1, 0xFFFFFFFF)]


def test_ack_comes_from_the_camera_and_targets_the_requester():
    publisher, vehicle = _publisher(TWO_STREAMS)

    replies = _answer(publisher, vehicle, _request(259))

    kind, ack, source = replies[0]
    assert kind == "COMMAND_ACK"
    assert source == mav.MAV_COMP_ID_CAMERA
    assert ack.command == mav.MAV_CMD_REQUEST_MESSAGE
    assert ack.result == mav.MAV_RESULT_ACCEPTED
    assert (ack.target_system, ack.target_component) == (GCS_SYSTEM, GCS_COMPONENT)
    assert all(source == mav.MAV_COMP_ID_CAMERA for _, _, source in replies)


def test_camera_information_announces_video_streams():
    publisher, vehicle = _publisher([_spec(1, [_stream(1)], name="siyi_zr10")])

    replies = _answer(publisher, vehicle, _request(259))

    assert [kind for kind, _, _ in replies] == ["COMMAND_ACK", "CAMERA_INFORMATION"]
    info = _wire(replies[1][1], OWN, mav.MAV_COMP_ID_CAMERA)  # survives encoding
    assert info.flags & mav.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
    assert bytes(info.model_name).rstrip(b"\0") == b"siyi_zr10"
    assert bytes(info.vendor_name).rstrip(b"\0") == b"NavPy"
    assert (info.resolution_h, info.resolution_v) == (1920, 1080)


def test_sim_detector_camera_names_the_sim_vendor():
    publisher, vehicle = _publisher(
        [_spec(1, None)], health=[CameraHealth(NAVPY_SIM_CAMERA_VENDOR, _ready)]
    )

    replies = _answer(publisher, vehicle, _request(259))

    info = _wire(replies[1][1], OWN, mav.MAV_COMP_ID_CAMERA)
    assert bytes(info.vendor_name).rstrip(b"\0") == b"NavPy sim"


def test_camera_without_streams_answers_information_and_denies_streams():
    publisher, vehicle = _publisher(TWO_STREAMS)

    info_replies = _answer(publisher, vehicle, _request(259, component=mav.MAV_COMP_ID_CAMERA2))
    stream_replies = _answer(publisher, vehicle, _request(269, component=mav.MAV_COMP_ID_CAMERA2))

    assert [kind for kind, _, _ in info_replies] == ["COMMAND_ACK", "CAMERA_INFORMATION"]
    info = _wire(info_replies[1][1], OWN, mav.MAV_COMP_ID_CAMERA2)
    assert info.flags & mav.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM == 0
    assert info.gimbal_device_id == 2
    assert [kind for kind, _, _ in stream_replies] == ["COMMAND_ACK"]
    assert stream_replies[0][1].result == mav.MAV_RESULT_DENIED
    assert all(source == mav.MAV_COMP_ID_CAMERA2 for _, _, source in info_replies + stream_replies)


def test_all_streams_on_param2_zero_each_with_count():
    publisher, vehicle = _publisher(TWO_STREAMS)

    replies = _answer(publisher, vehicle, _request(269, param2=0))

    assert [kind for kind, _, _ in replies] == [
        "COMMAND_ACK", "VIDEO_STREAM_INFORMATION", "VIDEO_STREAM_INFORMATION",
    ]
    streams = [_wire(message, OWN, source) for _, message, source in replies[1:]]
    assert [stream.stream_id for stream in streams] == [1, 2]
    assert [stream.count for stream in streams] == [2, 2]
    assert streams[0].uri == "rtsp://192.168.144.30:8554/s1"
    assert streams[0].name == "overlay"
    assert streams[0].type == mav.VIDEO_STREAM_TYPE_RTSP
    assert [stream.encoding for stream in streams] == [
        mav.VIDEO_STREAM_ENCODING_H264, mav.VIDEO_STREAM_ENCODING_H265,
    ]
    assert (streams[1].resolution_h, streams[1].resolution_v) == (1280, 720)
    assert streams[1].framerate == pytest.approx(30.0)


def test_one_stream_on_its_id():
    publisher, vehicle = _publisher(TWO_STREAMS)

    replies = _answer(publisher, vehicle, _request(269, param2=2))

    assert [kind for kind, _, _ in replies] == ["COMMAND_ACK", "VIDEO_STREAM_INFORMATION"]
    assert replies[0][1].result == mav.MAV_RESULT_ACCEPTED
    assert (replies[1][1].stream_id, replies[1][1].count) == (2, 2)


@pytest.mark.parametrize("param2", [3, 1.5])
def test_unknown_stream_id_is_denied_with_nothing_else(param2):
    publisher, vehicle = _publisher(TWO_STREAMS)

    replies = _answer(publisher, vehicle, _request(269, param2=param2))

    assert [kind for kind, _, _ in replies] == ["COMMAND_ACK"]
    assert replies[0][1].result == mav.MAV_RESULT_DENIED
    assert replies[0][2] == mav.MAV_COMP_ID_CAMERA


@pytest.mark.parametrize(
    "request_",
    [
        pytest.param(_request(259, component=mav.MAV_COMP_ID_CAMERA3), id="no-such-camera"),
        pytest.param(_request(259, target=OWN + 1), id="other-system"),
        pytest.param(_request(260), id="other-message"),
    ],
)
def test_requests_not_for_one_of_our_cameras_get_no_answer(request_):
    publisher, vehicle = _publisher(TWO_STREAMS)

    assert _answer(publisher, vehicle, request_) == []


def test_nothing_is_sent_or_subscribed_without_a_camera_mount():
    publisher, vehicle = _publisher([])

    publisher.start()
    publisher.send_heartbeats()
    publisher.answer(_request(259))

    assert vehicle.sent == [] and vehicle.callbacks == []
    assert publisher.stop() is True and publisher.is_quiescent


def test_worker_sends_heartbeats_and_answers_queued_requests():
    publisher, vehicle = _publisher([_spec(1, [_stream(1)])])
    publisher.start()
    try:
        assert vehicle.sent_event.wait(WAIT_S)  # the first heartbeat is immediate
        assert len(vehicle.callbacks) == 1
        vehicle.sent_event.clear()
        vehicle.callbacks[0](_request(269, param2=1))
        while not any(m.get_type() == "VIDEO_STREAM_INFORMATION" for m, _ in vehicle.sent):
            assert vehicle.sent_event.wait(WAIT_S)
            vehicle.sent_event.clear()
    finally:
        assert publisher.stop() is True
    assert publisher.is_quiescent
    assert vehicle.callbacks == []  # unsubscribed on stop
    publisher.raise_if_failed()


def test_heartbeat_period_is_the_shared_constant(monkeypatch):
    """The worker schedules the next heartbeat CAMERA_HEARTBEAT_PERIOD_S on."""
    publisher, vehicle = _publisher([_spec(1, [_stream(1)])])
    import navpy.plugins.vehicle.ardupilot.camera_stream_publisher as module

    just_before = 100.0 + CAMERA_HEARTBEAT_PERIOD_S - 1e-6
    on_time = 100.0 + CAMERA_HEARTBEAT_PERIOD_S
    clock = iter([100.0, 100.0, 100.0, just_before, just_before, on_time, on_time])
    stop = threading.Event()
    waits: list[float] = []
    monkeypatch.setattr(module.rt, "monotonic", lambda: next(clock))

    def take(stop_event, timeout_s):
        waits.append(timeout_s)
        if len(waits) == 3:
            stop.set()
        return ()

    monkeypatch.setattr(publisher, "_take_requests", take)
    publisher._run(stop)

    heartbeats = [m for m, _ in vehicle.sent if m.get_type() == "HEARTBEAT"]
    assert len(heartbeats) == 2  # at 100.0 and at exactly one period later
    assert waits[0] == pytest.approx(CAMERA_HEARTBEAT_PERIOD_S)
    assert waits[1] == pytest.approx(1e-6)  # none sent just before the period


def test_profile_without_gimbal_device_component_is_rejected():
    spec = CameraMountSpec(
        mount=Mock(), device=MappingProxyType({"name": "x", "video_streams": [_stream(1)]}),
        gimbal_device_id=None,
    )
    with pytest.raises(ValueError, match="no camera component"):
        streaming_cameras([spec], [CameraHealth(NAVPY_CAMERA_VENDOR, _ready)])


def test_mount_without_gimbal_device_component_or_streams_is_left_out():
    spec = CameraMountSpec(
        mount=Mock(), device=MappingProxyType({"name": "x"}), gimbal_device_id=None,
    )
    health = [CameraHealth(NAVPY_CAMERA_VENDOR, _ready)] * 2

    cameras = streaming_cameras([spec, _spec(2, None)], health)

    assert [camera.component_id for camera in cameras] == [mav.MAV_COMP_ID_CAMERA2]


def test_each_mount_needs_its_own_health_port():
    with pytest.raises(ValueError, match="2 camera mounts but 1 health ports"):
        streaming_cameras(
            [_spec(1, None), _spec(2, None)], [CameraHealth(NAVPY_CAMERA_VENDOR, _ready)]
        )


class _Member:
    def __init__(self, name, log, *, stop_result=True, stop_error=None, start_error=None):
        self.name, self.log = name, log
        self.stop_result, self.stop_error, self.start_error = stop_result, stop_error, start_error
        self.is_quiescent = True

    def start(self):
        self.log.append(("start", self.name))
        if self.start_error:
            raise self.start_error

    def stop(self):
        self.log.append(("stop", self.name))
        if self.stop_error:
            raise self.stop_error
        return self.stop_result

    def raise_if_failed(self):
        self.log.append(("health", self.name))


def test_publisher_group_starts_in_order_and_stops_all_in_reverse():
    log: list = []
    group = VisionPublisherGroup([_Member("gimbal", log), _Member("camera", log)])

    group.start()
    assert group.stop() is True
    group.raise_if_failed()

    assert log == [
        ("start", "gimbal"), ("start", "camera"),
        ("stop", "camera"), ("stop", "gimbal"),
        ("health", "gimbal"), ("health", "camera"),
    ]


def test_publisher_group_stops_every_member_after_a_failure():
    log: list = []
    error = RuntimeError("camera stuck")
    group = VisionPublisherGroup([
        _Member("gimbal", log, stop_result=False),
        _Member("camera", log, stop_error=error),
    ])

    with pytest.raises(RuntimeError, match="camera stuck"):
        group.stop()
    assert log == [("stop", "camera"), ("stop", "gimbal")]
    assert VisionPublisherGroup([_Member("gimbal", log, stop_result=False)]).stop() is False
