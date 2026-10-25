"""The camera's detector readiness changes no detection navigation reads.

docs/design/gcs_preflight_camera_row.md, Isolation: the assembled sim vision
controller runs the same scripted poses twice with the camera publisher
running, its detector readiness forced ready in one run and not ready in the
other. The heartbeats differ; the coordinator's detections, the only vision
output navigation reads, must not.
"""

from __future__ import annotations

import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega as mav

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.detect_request import DetectRequest
from navpy.core.ports.subscription import Subscription
from navpy.plugins.vehicle.ardupilot import camera_stream_publisher
from navpy.sim.world.fleet_dock_world import FLEET_DOCKS_ENV
from tests.conftest import create_mock_args

WAIT_S = 5.0  # bounds a failing run only
# Scripted (time_boot_s, lat, alt, pitch, roll): the dock is at 40.001, 44.0.
POSES = (
    (10.0, 39.999, 1100.0, 0.0, 0.0),
    (10.1, 39.9992, 1095.0, 2.0, -1.0),
    (10.2, 39.9994, 1090.0, -3.0, 4.0),
    (10.3, 39.9996, 1085.0, 5.0, 0.5),
)


def _vehicle() -> Mock:
    """Message subscriptions, sent messages and the poses the sim reads."""
    vehicle = Mock()
    vehicle.attitude = Attitude(0, 0, 0)
    vehicle.home_location = Location(40.0, 44.0, 1000.0)
    vehicle.mission_items_count = 0
    vehicle.get_param_or_default = Mock(side_effect=lambda name, default: default)
    vehicle.source_system = 3
    vehicle.target_system = 3
    vehicle.is_armed = True
    vehicle.air_speed = 20.0
    vehicle.is_simulated_autopilot = Mock(return_value=False)
    vehicle.attitude_sample = None
    vehicle.simulator_truth_pose = None
    vehicle.callbacks = {}
    vehicle.sent = []

    def on_message(message_name, callback):
        vehicle.callbacks.setdefault(message_name, []).append(callback)
        return Subscription(lambda: vehicle.callbacks[message_name].remove(callback))

    def send_mavlink_message(message, *, source_component=None):
        vehicle.sent.append((message, source_component))

    vehicle.on_message = on_message
    vehicle.send_mavlink_message = send_mavlink_message
    return vehicle


def _camera_statuses(vehicle) -> list[int]:
    return [
        message.system_status for message, source in list(vehicle.sent)
        if source == mav.MAV_COMP_ID_CAMERA and message.get_type() == "HEARTBEAT"
    ]


def _feed(vehicle, time_boot_s, lat, alt, pitch, roll) -> None:
    receipt_s = rt.time()
    attitude = Attitude(pitch, 0.0, roll)
    vehicle.attitude_sample = SimpleNamespace(
        attitude=attitude, time_boot_s=time_boot_s, receipt_time_s=receipt_s,
        body_rates_rad_s=(0.0, 0.0, 0.0),
    )
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=Location(lat, 44.0, alt, is_absolute=True),
        attitude=attitude, receipt_time_s=receipt_s,
    )
    for message_type in ("ATTITUDE", "SIM_STATE"):
        message = Mock()
        message.get_type.return_value = message_type
        message.time_boot_ms = int(round(time_boot_s * 1000))
        for callback in list(vehicle.callbacks.get(message_type, ())):
            callback(message)


def _detection(poi) -> tuple:
    pixel = poi.pixel
    return (
        poi.identity.obj_id, poi.identity.task_id, poi.classification.confidence,
        pixel.u_px, pixel.v_px, pixel.aircraft_pitch_deg, pixel.aircraft_roll_deg,
        pixel.source_timestamp_s, pixel.camera_to_body.elements,
    )


def _run(monkeypatch, ready: bool) -> tuple[list[tuple], list[int]]:
    from navpy.app.perception.vision_controller import VisionController

    monkeypatch.setattr(
        "navpy.app.perception.sim_detector_factory._sim_camera_ready", lambda: ready
    )
    vehicle = _vehicle()
    vision_args = SimpleNamespace(
        profile_name="ideal_360", detector_type="sim", model_path="", debug_show=False,
    )
    controller = VisionController(vehicle, create_mock_args(), vision_args, Mock())
    seen: list[tuple] = []
    controller.start()
    try:
        deadline = time.monotonic() + WAIT_S
        while not _camera_statuses(vehicle):  # the publisher is running
            assert time.monotonic() < deadline
            time.sleep(0.01)
        for pose in POSES:
            _feed(vehicle, *pose)
            deadline = time.monotonic() + WAIT_S
            while True:
                drained = controller.coordinator.drain_detection_events(DetectRequest())
                for publication in drained:
                    seen.append((
                        publication.source_timestamp_s,
                        publication.source_discontinuity,
                        tuple(_detection(poi) for poi in publication.detected_pois),
                    ))
                response = controller.coordinator.get_detect_data(DetectRequest())
                if seen and seen[-1][0] == pose[0]:
                    seen.append(tuple(_detection(poi) for poi in response.detected_pois))
                    break
                assert time.monotonic() < deadline, f"no detection for {pose}"
                time.sleep(0.01)
    finally:
        assert controller.stop() is True
    return seen, _camera_statuses(vehicle)


def test_detections_are_the_same_whether_the_camera_reports_ready_or_not(
    monkeypatch, tmp_path,
):
    docks = tmp_path / "docks.json"
    docks.write_text(json.dumps({"schema_version": 1, "docks": [{
        "name": "d1", "lat": 40.001, "lon": 44.0, "height_m": 0.0, "location_type": None,
    }]}), encoding="utf-8")
    monkeypatch.setenv(FLEET_DOCKS_ENV, str(docks))
    # Heartbeats between the scripted poses, so readiness is read during the run.
    monkeypatch.setattr(camera_stream_publisher, "CAMERA_HEARTBEAT_PERIOD_S", 0.01)

    ready_detections, ready_statuses = _run(monkeypatch, ready=True)
    not_ready_detections, not_ready_statuses = _run(monkeypatch, ready=False)

    assert set(ready_statuses) == {mav.MAV_STATE_ACTIVE}
    assert set(not_ready_statuses) == {mav.MAV_STATE_CRITICAL}
    assert len(ready_detections) == 2 * len(POSES)  # every pose was detected
    assert all(entry for entry in ready_detections)
    assert not_ready_detections == ready_detections
