from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_SIM_STATE,
)

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.sim.siyi_geo_pixel_source import (
    SIYI_GEO_POSE_MESSAGE_TYPES,
    SiyiGeoPixelSource,
)
from navpy.modules.vision.sim.siyi_pixel_diagnostics import SiyiSightRecorder
from scripts.siyi_pixel_pn_child import (
    _command_target_approach,
    _command_target_loiter,
)


def _message(name: str) -> object:
    return SimpleNamespace(get_type=lambda: name)


def test_siyi_source_uses_geo_for_pointing_but_delivers_only_finite_pixels() -> None:
    vehicle = Mock()
    vehicle.get_param_or_default.return_value = 50.0
    vehicle.air_speed = 35.0
    vehicle.attitude_sample = SimpleNamespace(
        attitude=Attitude(0.0, 0.0, 0.0),
        time_boot_s=10.0,
        receipt_time_s=100.0,
        body_rates_rad_s=(0.0, 0.0, 0.0),
    )
    location = Location(40.0, 44.0, 100.0, is_absolute=True)
    target = Location(40.001, 44.0, 100.0, is_absolute=True)
    vehicle.location.return_value = location
    mount = Mock(image_width=1920, image_height=1080)
    mount.capture_frame_state.return_value = SimpleNamespace(
        k=np.asarray([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]),
        gimbal_data=GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            setup=GimbalMountSetup(att=Attitude(90.0, 0.0, 90.0)),
            reference_aircraft_attitude=Attitude(0.0, 0.0, 0.0),
            max_detect_distance=5000.0,
            name="siyi_zr10",
        ),
    )
    mount.is_valid.return_value = True
    mount.gimbal.get_zoom_level.return_value = 3.0
    tracker = Mock(is_zoom_stable=True)
    tracker.status.geo.zoom_key = "3"
    geo_ref = Mock()
    geo_ref.calc_uv.return_value = (960.0, 540.0)
    delivered = []
    source = SiyiGeoPixelSource(
        vehicle,
        target,
        mount,
        tracker,
        geo_ref,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        min_pixels=1.0,
        deliver=lambda detection: delivered.append(detection) or True,
        wall_now_s=lambda: 100.0,
    )

    source._on_message(_message("ATTITUDE"))
    source._on_message(_message("GLOBAL_POSITION_INT"))
    assert source.ready
    source.activate()
    vehicle.attitude_sample.time_boot_s = 10.1
    vehicle.attitude_sample.receipt_time_s = 100.1
    source._on_message(_message("ATTITUDE"))
    source._on_message(_message("GLOBAL_POSITION_INT"))

    assert source.dispatch_available()
    detection = delivered[0]
    tracker.update_geo.assert_called()
    tracker.prepare_geo_acquisition.assert_called()
    assert detection.pixel.source_name == "siyi_zr10"
    assert source.latest_detection is detection
    assert detection.visual_detection().observation.camera_to_body is not None
    assert not hasattr(detection.visual_detection(), "target_location")
    assert source.metrics.sight_losses == 0


def test_siyi_source_counts_finite_camera_loss_after_activation() -> None:
    source = object.__new__(SiyiGeoPixelSource)
    source._lock = __import__("threading").RLock()
    source._sight = SiyiSightRecorder(None, None)
    source._sight._sight_losses = 2
    source._sight._first_sight_loss_distance_m = 37.0
    source._sight._current_sight_loss_run = 2
    source._sight._longest_sight_loss_run = 2
    source._sight._max_gimbal_age_s = 0.03
    source._sight._max_reference_attitude_delta_deg = 4.0
    source._sight._max_visual_truth_ray_error_deg = 0.2

    assert source.metrics.sight_losses == 2
    assert source.metrics.first_sight_loss_distance_m == 37.0
    assert source.metrics.longest_sight_loss_run == 2
    assert source.metrics.max_gimbal_age_s == 0.03
    assert source.metrics.max_reference_attitude_delta_deg == 4.0
    assert source.metrics.max_visual_truth_ray_error_deg == 0.2


def test_siyi_pixel_navigation_holds_target_centered_loiter_until_acquired() -> None:
    vehicle = Mock()
    vehicle.location.return_value = Location(1.0, 2.0, 140.0, False)
    vehicle.get_param_or_default.return_value = 90.0

    _command_target_loiter(vehicle, Location(3.0, 4.0, 60.0, True))

    commanded, radius = vehicle.goto_loiter.call_args.args
    assert commanded == Location(3.0, 4.0, 140.0, False)
    assert radius == 90.0


POSE_MESSAGE_ID_BY_NAME = {
    "ATTITUDE": MAVLINK_MSG_ID_ATTITUDE,
    "GLOBAL_POSITION_INT": MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    "SIM_STATE": MAVLINK_MSG_ID_SIM_STATE,
}


def test_siyi_start_only_subscribes_streams_it_actually_requests() -> None:
    """A subscribed message that is never streamed starves the event pair.

    SIM_STATE has no default Plane stream. This source renders from telemetry
    pose and never asks for one, so naming SIM_STATE among its pose messages
    would leave ``require_event_pair`` permanently unsatisfied and acquisition
    would time out having produced no frames at all.

    Asserting the subscription list against this source's own constant would be
    a tautology, so the check is subscribed-implies-requested: whatever the
    constant says, every name in it has to have a stream behind it.
    """
    vehicle = Mock()
    vehicle.get_param_or_default.return_value = 50.0
    source = SiyiGeoPixelSource(
        vehicle,
        Location(40.001, 44.0, 100.0, is_absolute=True),
        Mock(image_width=1920, image_height=1080),
        Mock(),
        Mock(),
        Mock(wall_period_for_scheduler_period=lambda value: value),
        min_pixels=1.0,
        deliver=lambda detection: True,
    )

    source.start()

    subscribed = [call.args[0] for call in vehicle.on_message.call_args_list]
    requested = {
        call.kwargs.get("p1")
        for call in vehicle.send_command_long.call_args_list
    }
    assert subscribed == list(SIYI_GEO_POSE_MESSAGE_TYPES)
    assert {POSE_MESSAGE_ID_BY_NAME[name] for name in subscribed} <= requested
    # This source renders from telemetry pose, so it must NOT pay for the
    # simulator truth stream.
    assert MAVLINK_MSG_ID_SIM_STATE not in requested


def test_siyi_pixel_navigation_uses_geo_only_for_pre_navigation_approach() -> None:
    vehicle = Mock()
    vehicle.location.return_value = Location(1.0, 2.0, 140.0, False)

    _command_target_approach(vehicle, Location(3.0, 4.0, 60.0, True))

    vehicle.goto.assert_called_once_with(Location(3.0, 4.0, 140.0, False))


def test_siyi_visible_frames_count_only_the_scoring_interval() -> None:
    """``projected_frames`` must span the same leg as ``delivered_frames``.

    ``scripts/siyi_pixel_pn_child.py`` publishes ``visible_frames`` as
    ``projected_frames``, and ``eval_observation_freshness`` divides
    ``delivered_frames`` by it and rejects the run below
    ``MIN_FRESH_OBSERVATION_FRACTION``. Deliveries only happen from
    ``dispatch_available`` once the source is active, so counting frames the
    camera rendered while it was still acquiring puts frames in the
    denominator that could never reach the numerator, and an accurate run is
    thrown out as host contention.
    """
    vehicle = Mock()
    vehicle.get_param_or_default.return_value = 50.0
    vehicle.air_speed = 35.0
    vehicle.attitude_sample = SimpleNamespace(
        attitude=Attitude(0.0, 0.0, 0.0),
        time_boot_s=10.0,
        receipt_time_s=100.0,
        body_rates_rad_s=(0.0, 0.0, 0.0),
    )
    location = Location(40.0, 44.0, 100.0, is_absolute=True)
    target = Location(40.001, 44.0, 100.0, is_absolute=True)
    vehicle.location.return_value = location
    mount = Mock(image_width=1920, image_height=1080)
    mount.capture_frame_state.return_value = SimpleNamespace(
        k=np.asarray([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]),
        gimbal_data=GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            setup=GimbalMountSetup(att=Attitude(90.0, 0.0, 90.0)),
            reference_aircraft_attitude=Attitude(0.0, 0.0, 0.0),
            max_detect_distance=5000.0,
            name="siyi_zr10",
        ),
    )
    mount.is_valid.return_value = True
    mount.gimbal.get_zoom_level.return_value = 3.0
    tracker = Mock(is_zoom_stable=True)
    tracker.status.geo.zoom_key = "3"
    geo_ref = Mock()
    geo_ref.calc_uv.return_value = (960.0, 540.0)
    delivered: list[object] = []
    source = SiyiGeoPixelSource(
        vehicle,
        target,
        mount,
        tracker,
        geo_ref,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        min_pixels=1.0,
        deliver=lambda detection: delivered.append(detection) or True,
        wall_now_s=lambda: 100.0,
    )

    # Acquisition: the camera renders the target, but navigation is not driving
    # yet and nothing can be delivered.
    source._on_message(_message("ATTITUDE"))
    source._on_message(_message("GLOBAL_POSITION_INT"))
    assert source.ready
    assert source.metrics.visible_frames == 0
    assert source.metrics.delivered_frames == 0

    source.activate()
    vehicle.attitude_sample.time_boot_s = 10.1
    vehicle.attitude_sample.receipt_time_s = 100.1
    source._on_message(_message("ATTITUDE"))
    source._on_message(_message("GLOBAL_POSITION_INT"))
    assert source.dispatch_available()

    metrics = source.metrics
    assert metrics.visible_frames == 1
    assert metrics.delivered_frames == 1
    assert metrics.delivered_frames / metrics.visible_frames == 1.0
