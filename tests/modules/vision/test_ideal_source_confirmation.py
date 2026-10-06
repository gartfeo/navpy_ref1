"""Exercise frame-associated gyro transport through ideal source rendering."""

import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.nav.vision_nav.confirmation import (
    FinalApproachConfirmation, FinalApproachConfirmationPorts,
)
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector, FinalApproachProjectionConfig,
)
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw
from navpy.modules.navigation.nav.vision_nav.law_config import (
    FixedFinalApproachLawConfigProvider, FinalApproachLawConfig,
)
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.detection_publication_store import DetectionPublicationStore
from navpy.modules.vision.sim.frame_generation_gate import FrameGenerationGate
from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSample
from navpy.modules.vision.sim.ideal_poi_projector import (
    IdealPoiProjector, UasFrameConvention,
)
from navpy.modules.vision.sim.pose_associator import PoseAssociator
from navpy.modules.vision.sim.pose_inbox import PoseInbox
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.sim.sim_detector_execution import SimDetectorExecution
from navpy.modules.vision.sim.sim_detector_state import ForcedGapState, SimCaptureState
from navpy.modules.vision.sim.sim_render_composition import build_pipeline
from navpy.modules.vision.sim.sim_poi_projector import SimPoiProjector
from navpy.modules.vision.sim.source_frame_coordinator import SourceFrameCoordinator
from navpy.modules.vision.simulation_object import SimulationObject


def _render_associated_frame(body_rates, compass_yaw):
    # Deliberately mutable to expose reuse of live telemetry after queuing.
    attitude_sample = SimpleNamespace(
        attitude=Attitude(2.0, compass_yaw, 10.0),
        time_boot_s=12.5, receipt_time_s=100.0,
        body_rates_rad_s=body_rates,
    )
    truth_pose = SimpleNamespace(
        location=Location(40.0, 44.0, 1000.0, is_absolute=True),
        attitude=Attitude(0.0, 0.0, 0.0), receipt_time_s=100.0,
    )
    associator = PoseAssociator(
        attitude_sample=lambda: attitude_sample, truth_pose=lambda: truth_pose,
        air_speed=lambda: 25.0, maximum_receipt_skew_s=lambda: 0.05,
        require_event_pair=True,
    )
    associator.note_event("ATTITUDE")
    associator.note_event("SIM_STATE")
    associated = associator.associate()
    assert associated is not None
    gate = FrameGenerationGate()
    pose = associator.build_sample(
        associated, timestamp_s=12.5,
        version=gate.token, sample_type=IdealPoseSample,
    )
    assert pose is not None
    if body_rates is not None:
        body_rates[:] = [8.0, 9.0, 10.0]
    attitude_sample.body_rates_rad_s = (11.0, 12.0, 13.0)

    projector = IdealPoiProjector(
        SimpleNamespace(read=lambda: GimbalData(Attitude(0, 0, 0), name="ideal")),
        FrameSize(2560, 1440), UasFrameConvention("ZYX", True), lambda: 12.5,
    )
    poi = SimulationObject(
        1, Location(40.001, 44.0, 900.0, is_absolute=True), 0,
    )
    dispatcher = SimPoiProjector(True, Mock(), projector)
    store = DetectionPublicationStore(source_driven=True, capacity=4)
    coordinator = SourceFrameCoordinator(
        gate=gate, inbox=PoseInbox(4), publications=store,
        record_outcome=Mock(), overload_warning=Mock(),
    )
    # Exercise the production composition and collection links too: a custom
    # render callback would miss a dropped gyro argument in either link.
    pipeline = build_pipeline(
        SimpleNamespace(
            mount=SimpleNamespace(
                image_width=2560, image_height=1440, get_k=Mock(),
                get_gimbal_data=Mock(), sync_zoom_from_hardware=Mock(),
            ),
            logger=Mock(),
        ),
        SimpleNamespace(ideal_360=True),
        SimpleNamespace(coordinator=coordinator),
        SimpleNamespace(
            capture=SimCaptureState(), gap=ForcedGapState(), evidence_recorder=None,
            tracking=SimpleNamespace(tracking_obj_id=None), projector=dispatcher,
            poi_provider=SimpleNamespace(snapshot=lambda: (poi,)),
            publications=SimpleNamespace(source_name=lambda pois: "ideal"),
        ),
    )

    errors = Mock()
    execution = SimDetectorExecution(
        coordinator=SimpleNamespace(
            stop_event=threading.Event(), wait_and_pop_pose=Mock(side_effect=[pose, None]),
        ),
        polling_pose_reader=Mock(), error=errors, cadence_lease=Mock(),
        scheduler_period_s=0.02, ideal_360=True,
        detect_pois=pipeline.detect_pois, record_outcome=Mock(),
    )
    execution.run()
    errors.assert_not_called()
    publications = store.drain()
    assert len(publications) == 1
    detections = publications[0].detected_pois
    assert len(detections) == 1
    return pose, detections[0]


def _confirmation():
    return FinalApproachConfirmation(FinalApproachConfirmationPorts(
        lock=threading.RLock(),
        projector=FinalApproachFrameProjector(FinalApproachProjectionConfig("ZYX", True)),
        epochs=SourceEpochLedger(),
        law=VisionNavLaw(FixedFinalApproachLawConfigProvider(
            FinalApproachLawConfig(-55.0, 20.0, 45.0, None, 0.0),
        )),
    ))


@pytest.mark.parametrize("compass_yaw", [0.0, 137.0])
def test_ideal_source_retains_frame_gyros_for_final_approach_confirmation(compass_yaw):
    pose, detection = _render_associated_frame([0.1, 0.2, 0.3], compass_yaw)
    # The real confirmation gate previously refused every ideal source frame.
    confirmation = _confirmation()
    assert confirmation.can_confirm_detection(detection)
    assert pose.navigation_attitude.yaw == 0.0
    assert pose.body_rates_rad_s == (0.1, 0.2, 0.3)
    expected = (0.2 * math.sin(math.radians(10)) + 0.3 * math.cos(math.radians(10))) / math.cos(math.radians(2))
    assert detection.pixel.aircraft_yaw_rate_rad_s == pytest.approx(expected)
    assert confirmation.record_final_approach_confirmed_detection(detection)


def test_ideal_source_without_frame_gyros_remains_unconfirmable():
    _pose, detection = _render_associated_frame(None, 137.0)
    assert detection.pixel.aircraft_yaw_rate_rad_s is None
    assert not _confirmation().can_confirm_detection(detection)
