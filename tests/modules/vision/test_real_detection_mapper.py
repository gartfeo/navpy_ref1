from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount_types import CameraMountFrameState
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.real_detector_state import ConfirmationFrameStore
from navpy.modules.vision.real_frame_association import RealFrameAssociation
from navpy.modules.vision.real_detection_mapper import (
    DetectedObjectMapper,
    DetectionMappingConfig,
)


def _track(obj_id: int, *, confirmed: bool) -> TrackedObject:
    return TrackedObject(
        id=obj_id,
        cx=320.0,
        cy=240.0,
        w=80.0,
        h=40.0,
        confidence=0.92,
        class_id=3,
        age=1,
        hits=1,
        missed=0,
        is_confirmed=confirmed,
        timestamp=10.0,
        vx=5.0,
        vy=-2.0,
    )


def _association() -> RealFrameAssociation:
    intrinsic = np.array(
        [
            [1000.0, 0.0, 320.0],
            [0.0, 1000.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    mount_state = CameraMountFrameState(
        gimbal_data=GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        k=intrinsic,
        dist=np.zeros(5, dtype=np.float32),
        gimbal_timestamp_s=None,
        gimbal_is_static=True,
        zoom_command="1.0",
        zoom_sample_id="static:1.0",
        zoom_sample_age_s=0.0,
    )
    return RealFrameAssociation(
        frame=np.zeros((480, 640, 3), dtype=np.uint8),
        frame_width=640,
        frame_height=480,
        frame_sequence=1,
        frame_timestamp_s=10.0,
        associated_at_s=10.01,
        uas_att=Attitude(1.0, 2.0, 3.0),
        uas_body_rates_rad_s=None,
        attitude_boot_time_s=9.0,
        attitude_receipt_time_s=9.99,
        mount_state=mount_state,
        c_g_loc=None,
        air_speed_mps=20.0,
        pose_is_frame_atomic=True,
        pose_status="real_frame_pose",
        pose_age_s=0.01,
    )


def test_all_mode_emits_only_confirmed_tracks_but_keeps_all_live_ids():
    store = Mock(spec=ConfirmationFrameStore)
    store.get.return_value = None
    mapper = DetectedObjectMapper(DetectionMappingConfig(output_mode="all"), store)
    confirmed = _track(1, confirmed=True)
    tentative = _track(2, confirmed=False)

    targets = mapper.convert([confirmed, tentative], None, _association())

    assert [target.identity.obj_id for target in targets] == [1]
    store.evict_except.assert_called_once_with({1, 2})


def test_locked_mode_emits_unconfirmed_lock_and_evicts_from_live_tracks():
    store = Mock(spec=ConfirmationFrameStore)
    store.get.return_value = None
    mapper = DetectedObjectMapper(
        DetectionMappingConfig(output_mode="locked"),
        store,
    )
    live = _track(1, confirmed=True)
    locked = _track(9, confirmed=False)

    targets = mapper.convert([live], locked, _association())

    assert [target.identity.obj_id for target in targets] == [9]
    store.evict_except.assert_called_once_with({1})


def test_missing_mount_returns_without_touching_confirmation_store():
    store = Mock(spec=ConfirmationFrameStore)
    mapper = DetectedObjectMapper(DetectionMappingConfig(output_mode="all"), store)

    targets = mapper.convert([], None, SimpleNamespace(mount_state=None))

    assert targets == []
    assert store.method_calls == []


def test_locked_mode_without_lock_returns_without_eviction():
    store = Mock(spec=ConfirmationFrameStore)
    mapper = DetectedObjectMapper(
        DetectionMappingConfig(output_mode="locked"),
        store,
    )

    targets = mapper.convert(
        [_track(1, confirmed=True)],
        None,
        SimpleNamespace(mount_state=object()),
    )

    assert targets == []
    assert store.method_calls == []
