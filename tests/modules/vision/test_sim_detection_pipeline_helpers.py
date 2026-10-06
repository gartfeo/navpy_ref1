from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Optional
from unittest.mock import Mock

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.sim_camera_ports import TrackingCameraPort
from navpy.modules.vision.sim.sim_detection_gap import ForcedDetectionGapPolicy
from navpy.modules.vision.sim.sim_detection_context import SimDetectionContext
from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_detector_state import ForcedGapState
from navpy.modules.vision.sim.sim_frame_timestamp import SimFrameTimestampResolver
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.sim_tracking_update import SimTrackingUpdater
from navpy.modules.vision.simulation_object import SimulationObject
from tests.detection_factory import make_detected_poi


@dataclass
class TrackingPort:
    tracking_obj_id: Optional[int]
    updates: list = field(default_factory=list)

    def apply_detection_update(self, poi, timestamp_s: float) -> None:
        self.updates.append((poi, timestamp_s))


def test_forced_gap_anchors_to_first_tracked_frame_and_uses_half_open_window():
    tracking = TrackingPort(7)
    state = ForcedGapState()
    policy = ForcedDetectionGapPolicy(
        tracking,
        state,
        specification=lambda: "1.0:2.0",
    )

    assert policy.commit(policy.plan(10.0)) is False
    assert state.anchor_timestamp_s == 10.0
    assert policy.commit(policy.plan(11.0)) is True
    assert policy.commit(policy.plan(12.999)) is True
    assert policy.commit(policy.plan(13.0)) is False


@pytest.mark.parametrize("specification", [None, "", "bad", "-1:2", "1:0"])
def test_forced_gap_rejects_missing_or_invalid_window(specification):
    state = ForcedGapState()
    policy = ForcedDetectionGapPolicy(
        TrackingPort(7),
        state,
        specification=lambda: specification,
    )

    assert policy.commit(policy.plan(10.0)) is False
    assert state.anchor_timestamp_s is None


def test_forced_gap_does_not_anchor_before_tracking_starts():
    tracking = TrackingPort(None)
    state = ForcedGapState()
    policy = ForcedDetectionGapPolicy(
        tracking,
        state,
        specification=lambda: "0:1",
    )

    assert policy.commit(policy.plan(10.0)) is False
    assert state.anchor_timestamp_s is None

    tracking.tracking_obj_id = 7
    assert policy.commit(policy.plan(11.0)) is True
    assert state.anchor_timestamp_s == 11.0


def test_tracking_updater_uses_exact_tracking_camera_and_diagnostic_ports():
    tracking = TrackingPort(None)
    camera = TrackingCameraPort(
        read_matrix=lambda: np.eye(3),
        read_gimbal=lambda: GimbalData(att=Attitude(0.0, 0.0, 0.0)),
    )
    diagnose = Mock(return_value="out_of_view")
    updater = SimTrackingUpdater(
        tracking=tracking,
        camera=camera,
        debug=Mock(),
        diagnose_track_loss=diagnose,
    )

    updater.update([], Location(40.0, 44.0, 1000.0), Attitude(0, 0, 0), 1.0)
    tracking.tracking_obj_id = 7
    updater.update([], Location(40.0, 44.0, 1000.0), Attitude(0, 0, 0), 2.0)

    diagnose.assert_called_once()
    assert tracking.updates == [(None, 2.0)]


def test_rejected_generation_has_no_confirmation_or_tracking_side_effects():
    class RejectedTransactions:
        generation = FrameGeneration(0, 0)

        @contextmanager
        def admit(self, generation):
            yield generation

        @contextmanager
        def reserve(self, _frame):
            yield Mock()

        @contextmanager
        def commit(self, _frame, _pending):
            yield None

    detection = make_detected_poi(x_error=10.0, y_error=20.0, k=None)
    capture = Mock()
    capture.wants_frame.return_value = False
    gap = Mock()
    gap.plan.return_value = object()
    tracking = Mock()
    poi = SimulationObject(7, Location(40.0, 44.0, 0.0), 2)
    context = SimDetectionContext(
        ideal_360=True,
        sync_camera_zoom=lambda: False,
        poi_snapshot=lambda: (poi,),
        project_poi=Mock(return_value=detection),
        timestamps=SimFrameTimestampResolver(
            lambda value, *, record_emitted: value,
        ),
        transactions=RejectedTransactions(),
    )
    pipeline = SimDetectionPipeline(
        context,
        confirmation_capture=capture,
        forced_gap_policy=gap,
        tracking_updater=tracking,
    )

    pipeline.detect_pois(
        Location(40.0, 44.0, 1000.0),
        Attitude(0.0, 0.0, 0.0),
        frame_timestamp_s=10.0,
        frame_epoch=0,
        frame_generation=FrameGeneration(0, 0),
    )

    capture.capture.assert_not_called()
    tracking.update.assert_not_called()
