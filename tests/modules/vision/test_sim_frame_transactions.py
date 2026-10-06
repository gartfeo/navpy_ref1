"""Deterministic concurrency tests for simulator frame transactions."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
)
from navpy.modules.vision.sim.frame_generation_gate import (
    FrameGeneration,
    FrameGenerationGate,
)
from navpy.modules.vision.sim.pose_inbox import PoseInbox
from navpy.modules.vision.sim.pose_frame_clock import PoseFrameClock
from navpy.modules.vision.sim.sim_detection_context import SimDetectionContext
from navpy.modules.vision.sim.sim_detection_gap import ForcedDetectionGapPolicy
from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_detector_state import ForcedGapState
from navpy.modules.vision.sim.sim_frame_timestamp import SimFrameTimestampResolver
from navpy.modules.vision.sim.sim_frame_transactions import DetectionFrameTransactions
from navpy.modules.vision.sim.sim_poi_catalog import SimPoiCatalog
from navpy.modules.vision.sim.source_frame_coordinator import SourceFrameCoordinator
from navpy.modules.vision.simulation_object import SimulationObject
from tests.detection_factory import make_detected_poi


LOCATION = Location(40.0, 44.0, 1000.0)
ATTITUDE = Attitude(0.0, 0.0, 0.0)
FIXTURE_LOCATION = SimulationObject(7, LOCATION, 2)


@dataclass
class TrackingStatus:
    tracking_obj_id: Optional[int]


class SignallingPublicationStore(DetectionPublicationStore):
    def __init__(self, entered: threading.Event) -> None:
        super().__init__(source_driven=True, capacity=1)
        self._entered = entered

    def reserve(self, invalidated, stopped):
        self._entered.set()
        return super().reserve(invalidated, stopped)


def _coordinator(store=None, *, inbox_capacity=4):
    outcomes = []
    store = store or DetectionPublicationStore(
        source_driven=True,
        capacity=4,
    )
    coordinator = SourceFrameCoordinator(
        gate=FrameGenerationGate(),
        inbox=PoseInbox(inbox_capacity),
        publications=store,
        record_outcome=lambda *args: outcomes.append(args),
        overload_warning=lambda _capacity: None,
    )
    return coordinator, store, outcomes


def _detection(obj_id=7):
    return make_detected_poi(
        obj_id=obj_id,
        x_error=10.0,
        y_error=20.0,
        k=None,
    )


def _pipeline(
    coordinator,
    update_poi,
    *,
    pois=(FIXTURE_LOCATION,),
    mount=None,
    tracking_id=7,
):
    capture = Mock()
    capture.wants_frame.return_value = False
    tracking = Mock()
    gap_state = ForcedGapState()
    gap = ForcedDetectionGapPolicy(
        TrackingStatus(tracking_id),
        gap_state,
        specification=lambda: "0:1",
    )
    transactions = DetectionFrameTransactions(
        coordinator,
        lambda _pois: "ideal",
    )
    context = SimDetectionContext(
        ideal_360=mount is None,
        sync_camera_zoom=(
            (lambda: False)
            if mount is None
            else mount.sync_zoom_from_hardware
        ),
        poi_snapshot=lambda: tuple(pois),
        project_poi=update_poi,
        timestamps=SimFrameTimestampResolver(
            lambda value, *, record_emitted: value,
        ),
        transactions=transactions,
    )
    pipeline = SimDetectionPipeline(
        context,
        confirmation_capture=capture,
        forced_gap_policy=gap,
        tracking_updater=tracking,
    )
    return pipeline, capture, tracking, gap_state


def _detect(pipeline, token, results, timestamp_s=10.0):
    results.append(pipeline.detect_pois(
        LOCATION,
        ATTITUDE,
        frame_timestamp_s=timestamp_s,
        frame_generation=token,
    ))


def test_reset_during_render_rejects_all_post_render_side_effects():
    coordinator, store, outcomes = _coordinator()
    render_started = threading.Event()
    release_render = threading.Event()

    def render(*_args, **_kwargs):
        render_started.set()
        assert release_render.wait(2.0)
        return _detection()

    pipeline, capture, tracking, gap_state = _pipeline(coordinator, render)
    token = coordinator.token
    results = []
    render_thread = threading.Thread(
        target=_detect,
        args=(pipeline, token, results),
    )
    render_thread.start()
    assert render_started.wait(2.0)

    reset_done = threading.Event()

    def reset():
        with coordinator.reset() as resetting:
            assert resetting
        reset_done.set()

    reset_thread = threading.Thread(target=reset)
    reset_thread.start()
    assert reset_done.wait(2.0), "render must not hold the reset gate"
    release_render.set()
    render_thread.join(2.0)
    reset_thread.join(2.0)

    assert results == [False]
    assert token.invalidated.is_set()
    assert gap_state.anchor_timestamp_s is None
    capture.capture.assert_not_called()
    tracking.update.assert_not_called()
    assert store.snapshot().detected_pois == ()
    assert store.drain() == []
    assert any(outcome[1] == "source_invalidated_dropped" for outcome in outcomes)


def test_stop_during_render_rejects_all_post_render_side_effects():
    coordinator, store, _outcomes = _coordinator()
    render_started = threading.Event()
    release_render = threading.Event()

    def render(*_args, **_kwargs):
        render_started.set()
        assert release_render.wait(2.0)
        return _detection()

    pipeline, capture, tracking, gap_state = _pipeline(coordinator, render)
    token = coordinator.token
    results = []
    render_thread = threading.Thread(
        target=_detect,
        args=(pipeline, token, results),
    )
    render_thread.start()
    assert render_started.wait(2.0)

    coordinator.stop()
    release_render.set()
    render_thread.join(2.0)

    assert results == [False]
    assert token.invalidated.is_set()
    assert gap_state.anchor_timestamp_s is None
    capture.capture.assert_not_called()
    tracking.update.assert_not_called()
    assert store.snapshot().detected_pois == ()


def test_pre_render_sensor_mutation_finishes_before_reset_clears_it():
    coordinator, _store, _outcomes = _coordinator()
    entered = threading.Event()
    release = threading.Event()
    state = {"zoom": 0}

    def sync_zoom():
        state["zoom"] = 1
        entered.set()
        assert release.wait(2.0)

    mount = SimpleNamespace(sync_zoom_from_hardware=sync_zoom)
    pipeline, _capture, _tracking, _gap = _pipeline(
        coordinator,
        Mock(return_value=None),
        mount=mount,
        tracking_id=None,
    )
    results = []
    render_thread = threading.Thread(
        target=_detect,
        args=(pipeline, coordinator.token, results),
    )
    render_thread.start()
    assert entered.wait(2.0)

    reset_done = threading.Event()

    def reset():
        with coordinator.reset() as resetting:
            assert resetting
            state["zoom"] = 0
        reset_done.set()

    reset_thread = threading.Thread(target=reset)
    reset_thread.start()
    assert not reset_done.wait(0.05)
    release.set()
    assert reset_done.wait(2.0)
    render_thread.join(2.0)
    reset_thread.join(2.0)

    assert state["zoom"] == 0


def test_clock_restart_aborts_old_generation_without_post_reset_mutation():
    coordinator, store, outcomes = _coordinator()
    clock = PoseFrameClock(
        reorder_tolerance_s=0.05,
        record_outcome=lambda *args: outcomes.append(args),
    )
    assert clock.advance(100.0, on_restart=lambda: None) == 100.0

    def reset_source():
        with coordinator.reset() as resetting:
            assert resetting
            clock.reset()

    render = Mock(return_value=_detection())
    capture = Mock()
    capture.wants_frame.return_value = False
    tracking = Mock()
    gap_state = ForcedGapState()
    gap = ForcedDetectionGapPolicy(
        TrackingStatus(None),
        gap_state,
        specification=lambda: None,
    )
    transactions = DetectionFrameTransactions(
        coordinator,
        lambda _pois: "sim",
    )
    context = SimDetectionContext(
        ideal_360=False,
        sync_camera_zoom=lambda: False,
        poi_snapshot=lambda: (FIXTURE_LOCATION,),
        project_poi=render,
        timestamps=SimFrameTimestampResolver(
            lambda value, *, record_emitted: clock.accept_attitude_timestamp(
                value,
                on_restart=reset_source,
                record_emitted=record_emitted,
            ),
        ),
        transactions=transactions,
    )
    pipeline = SimDetectionPipeline(
        context,
        confirmation_capture=capture,
        forced_gap_policy=gap,
        tracking_updater=tracking,
    )
    old = coordinator.token

    assert pipeline.detect_pois(
        LOCATION,
        ATTITUDE,
        attitude_time_boot_s=1.0,
        frame_generation=old,
    ) is False
    assert old.invalidated.is_set()
    assert clock.source_now_s is None
    assert clock.last_frame_s is None
    render.assert_not_called()

    assert pipeline.detect_pois(
        LOCATION,
        ATTITUDE,
        attitude_time_boot_s=1.02,
    ) is True
    assert clock.source_now_s == pytest.approx(1.02)
    assert clock.last_frame_s == pytest.approx(1.02)
    render.assert_called_once()
    assert len(store.snapshot().detected_pois) == 1


def test_reset_wakes_backpressured_reservation_before_render():
    reservation_entered = threading.Event()
    store = SignallingPublicationStore(reservation_entered)
    coordinator, _store, _outcomes = _coordinator(store)
    occupied = store.reserve(threading.Event(), threading.Event())
    assert occupied is not None
    assert store.publish(
        occupied,
        [],
        primary_poi=None,
        source_timestamp_s=1.0,
        source_receipt_timestamp_s=None,
        source_name="ideal",
        source_discontinuity=False,
    )
    reservation_entered.clear()

    render = Mock(return_value=_detection())
    pipeline, capture, tracking, _gap = _pipeline(coordinator, render)
    results = []
    render_thread = threading.Thread(
        target=_detect,
        args=(pipeline, coordinator.token, results),
    )
    render_thread.start()
    assert reservation_entered.wait(2.0)

    with coordinator.reset() as resetting:
        assert resetting
    render_thread.join(2.0)

    assert results == [False]
    render.assert_not_called()
    capture.capture.assert_not_called()
    tracking.update.assert_not_called()


def test_failed_reset_stays_closed_until_a_successful_retry():
    coordinator, _store, _outcomes = _coordinator()
    old = coordinator.token

    with pytest.raises(RuntimeError, match="reset failed"):
        with coordinator.reset() as resetting:
            assert resetting
            raise RuntimeError("reset failed")

    with coordinator.admission(old) as admitted:
        assert admitted is None

    with coordinator.reset() as resetting:
        assert resetting

    fresh = coordinator.token
    assert fresh is not old
    with coordinator.admission(fresh) as admitted:
        assert admitted is fresh


@dataclass(frozen=True)
class QueuedPose:
    timestamp_s: float
    generation: FrameGeneration
    source_discontinuity: bool = False


def test_source_pose_backlog_dispatches_latest_state_and_records_superseded():
    coordinator, _store, outcomes = _coordinator()
    generation = coordinator.token
    for timestamp_s in (1.0, 2.0, 3.0):
        assert coordinator.admit_pose(
            generation,
            lambda active, value=timestamp_s: QueuedPose(value, active),
        )

    selected = coordinator.wait_and_pop_pose()

    assert selected is not None
    assert selected.timestamp_s == 3.0
    assert [
        timestamp_s
        for timestamp_s, outcome, _frame_ts in outcomes
        if outcome == "source_superseded"
    ] == [1.0, 2.0]


def test_source_pose_backlog_preserves_each_discontinuity_before_latest_state():
    coordinator, _store, outcomes = _coordinator(inbox_capacity=5)
    generation = coordinator.token
    poses = (
        QueuedPose(1.0, generation),
        QueuedPose(2.0, generation, source_discontinuity=True),
        QueuedPose(3.0, generation),
        QueuedPose(4.0, generation, source_discontinuity=True),
        QueuedPose(5.0, generation),
    )
    for pose in poses:
        assert coordinator.admit_pose(
            generation,
            lambda _active, value=pose: value,
        )

    selected = [
        coordinator.wait_and_pop_pose(),
        coordinator.wait_and_pop_pose(),
        coordinator.wait_and_pop_pose(),
    ]

    assert [pose.timestamp_s for pose in selected if pose is not None] == [
        2.0,
        4.0,
        5.0,
    ]
    assert [
        timestamp_s
        for timestamp_s, outcome, _frame_ts in outcomes
        if outcome == "source_superseded"
    ] == [1.0, 3.0]


def test_old_callback_is_rejected_through_reset_and_fresh_epoch_reopens():
    coordinator, _store, _outcomes = _coordinator()
    old = coordinator.token
    built = Mock()

    with coordinator.reset() as resetting:
        assert resetting
        assert coordinator.admit_pose(old, built) is False
        assert coordinator.token is old
    built.assert_not_called()

    fresh = coordinator.token
    assert fresh.epoch == old.epoch + 1
    assert coordinator.admit_pose(
        fresh,
        lambda active: QueuedPose(2.0, active),
    ) is True
    queued = coordinator.wait_and_pop_pose()
    assert queued.generation is fresh


def test_poi_mutation_does_not_change_in_flight_frame_snapshot():
    first_poi = SimpleNamespace(uid=1)
    second_poi = SimpleNamespace(uid=2)

    class Provider:
        def __init__(self):
            self.pois = [first_poi]

        def set_sim_poi(self, *_args, **_kwargs):
            self.pois.append(second_poi)

        def refresh(self):
            self.pois = list(self.pois)

    catalog = SimPoiCatalog(Provider())
    coordinator, store, _outcomes = _coordinator()
    render_started = threading.Event()
    release_render = threading.Event()
    seen = []

    def render(_location, poi, _attitude, **_kwargs):
        seen.append(poi)
        if len(seen) == 1:
            render_started.set()
            assert release_render.wait(2.0)
        return _detection(poi.uid)

    capture = Mock()
    capture.wants_frame.return_value = False
    tracking = Mock()
    gap_state = ForcedGapState()
    gap = ForcedDetectionGapPolicy(
        TrackingStatus(None),
        gap_state,
        specification=lambda: None,
    )
    transactions = DetectionFrameTransactions(
        coordinator,
        lambda _pois: "ideal",
    )
    context = SimDetectionContext(
        ideal_360=True,
        sync_camera_zoom=lambda: False,
        poi_snapshot=catalog.snapshot,
        project_poi=render,
        timestamps=SimFrameTimestampResolver(
            lambda value, *, record_emitted: value,
        ),
        transactions=transactions,
    )
    pipeline = SimDetectionPipeline(
        context,
        confirmation_capture=capture,
        forced_gap_policy=gap,
        tracking_updater=tracking,
    )

    results = []
    frame_thread = threading.Thread(
        target=_detect,
        args=(pipeline, coordinator.token, results),
    )
    frame_thread.start()
    assert render_started.wait(2.0)
    catalog.set_sim_poi(2, LOCATION)
    release_render.set()
    frame_thread.join(2.0)

    assert results == [True]
    assert seen == [first_poi]
    assert pipeline.detect_pois(
        LOCATION,
        ATTITUDE,
        frame_timestamp_s=11.0,
        frame_generation=coordinator.token,
    )
    assert seen == [first_poi, first_poi, second_poi]
    assert len(store.snapshot().detected_pois) == 2
