"""Exclusive source-publication lease contracts."""

from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.modules.vision.detection_aggregation import DetectionAggregator
from navpy.modules.vision.detection_identity_registry import (
    DetectionIdentityRegistry,
)
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detection_event_lease import (
    DetectionLeaseDispatch,
)
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
)
from tests.detection_factory import make_detected_target


def _publish(
    store: DetectionPublicationStore,
    timestamp_s: float,
    *,
    obj_id: int = 7,
    detected: bool = True,
    source_discontinuity: bool = False,
) -> None:
    slot = store.reserve(threading.Event(), threading.Event())
    assert slot is not None
    target = make_detected_target(
        obj_id=obj_id,
        x_error=timestamp_s,
        y_error=0.0,
        k=None,
        timestamp=timestamp_s,
    )
    targets = [target] if detected else []
    assert store.publish(
        slot,
        targets,
        primary_target=target if detected else None,
        source_timestamp_s=timestamp_s,
        source_receipt_timestamp_s=None,
        source_name="ideal_360",
        source_discontinuity=source_discontinuity,
    )


def _timestamps(publications) -> list[float]:
    return [publication.source_timestamp_s for publication in publications]


def _dispatch_one(lease):
    delivered = []
    result = lease.wait_and_dispatch(
        lambda publication: delivered.append(publication) or True
    )
    assert result is True
    return delivered[0]


def test_nonblocking_lease_reports_empty_accept_reject_and_closed() -> None:
    store = DetectionPublicationStore(source_driven=True, capacity=2)
    lease = store.open_event_lease()

    assert lease.dispatch_available(lambda _publication: True) is (
        DetectionLeaseDispatch.EMPTY
    )
    _publish(store, 1.0)
    assert lease.dispatch_available(lambda _publication: True) is (
        DetectionLeaseDispatch.ACCEPTED
    )
    _publish(store, 2.0)
    assert lease.dispatch_available(lambda _publication: False) is (
        DetectionLeaseDispatch.REJECTED
    )
    lease.close()
    assert lease.dispatch_available(lambda _publication: True) is (
        DetectionLeaseDispatch.CLOSED
    )


def test_lease_atomically_claims_prefix_and_coalesces_to_latest_publication():
    store = DetectionPublicationStore(source_driven=True, capacity=4)
    _publish(store, 1.0)
    lease = store.open_event_lease()
    _publish(store, 2.0)

    assert store.drain() == []
    assert _dispatch_one(lease).source_timestamp_s == 2.0
    lease.close()


def test_terminal_lease_dispatches_latest_publication_from_backlog():
    store = DetectionPublicationStore(source_driven=True, capacity=4)
    lease = store.open_event_lease()
    _publish(store, 1.0)
    _publish(store, 2.0)
    _publish(store, 3.0)

    assert _dispatch_one(lease).source_timestamp_s == 3.0
    lease.close()


def test_terminal_lease_preserves_discontinuity_before_latest_state():
    store = DetectionPublicationStore(source_driven=True, capacity=4)
    lease = store.open_event_lease()
    _publish(store, 1.0)
    _publish(store, 2.0, detected=False, source_discontinuity=True)
    _publish(store, 3.0)

    reset = _dispatch_one(lease)
    latest = _dispatch_one(lease)

    assert reset.source_timestamp_s == 2.0
    assert reset.source_discontinuity is True
    assert reset.detected_targets == ()
    assert latest.source_timestamp_s == 3.0
    lease.close()


def test_terminal_lease_preserves_each_discontinuity_before_latest_state():
    store = DetectionPublicationStore(source_driven=True, capacity=5)
    lease = store.open_event_lease()
    _publish(store, 1.0)
    _publish(store, 2.0, detected=False, source_discontinuity=True)
    _publish(store, 3.0)
    _publish(store, 4.0, detected=False, source_discontinuity=True)
    _publish(store, 5.0)

    first_reset = _dispatch_one(lease)
    second_reset = _dispatch_one(lease)
    latest = _dispatch_one(lease)

    assert first_reset.source_timestamp_s == 2.0
    assert first_reset.source_discontinuity is True
    assert second_reset.source_timestamp_s == 4.0
    assert second_reset.source_discontinuity is True
    assert latest.source_timestamp_s == 5.0
    lease.close()


def test_terminal_lease_dispatches_latest_empty_state():
    store = DetectionPublicationStore(source_driven=True, capacity=3)
    lease = store.open_event_lease()
    _publish(store, 1.0)
    _publish(store, 2.0, detected=False)

    latest = _dispatch_one(lease)

    assert latest.source_timestamp_s == 2.0
    assert latest.detected_targets == ()
    lease.close()


def test_close_restores_unconsumed_events_and_future_normal_order():
    store = DetectionPublicationStore(source_driven=True, capacity=4)
    _publish(store, 1.0)
    lease = store.open_event_lease()
    _publish(store, 2.0)

    lease.close()
    _publish(store, 3.0)

    assert lease.closed is True
    assert _timestamps(store.drain()) == [1.0, 2.0, 3.0]


def test_close_wakes_waiter_without_delivering_restored_events_twice():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    lease = store.open_event_lease()
    drained = []
    waiting = threading.Thread(
        target=lambda: drained.append(
            lease.wait_and_dispatch(lambda publication: True)
        ),
    )
    waiting.start()

    lease.close()
    waiting.join(1.0)

    assert not waiting.is_alive()
    assert drained == [None]
    assert store.drain() == []


def test_source_stop_closes_empty_lease_and_wakes_waiter():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    lease = store.open_event_lease()
    drained = []
    waiting = threading.Thread(
        target=lambda: drained.append(
            lease.wait_and_dispatch(lambda publication: True)
        ),
    )
    waiting.start()

    dropped = store.stop()
    waiting.join(1.0)

    assert not waiting.is_alive()
    assert lease.closed is True
    assert drained == [None]
    assert dropped == []

    with pytest.raises(RuntimeError, match="stopped"):
        store.open_event_lease()


def test_lease_queue_remains_bounded_by_publication_capacity():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    lease = store.open_event_lease()
    _publish(store, 1.0)
    entered = threading.Event()
    completed = threading.Event()

    def publish_second() -> None:
        entered.set()
        _publish(store, 2.0)
        completed.set()

    producer = threading.Thread(target=publish_second)
    producer.start()
    assert entered.wait(1.0)
    assert not completed.wait(0.05)

    assert _timestamps([_dispatch_one(lease)]) == [1.0]
    assert completed.wait(1.0)
    assert _timestamps([_dispatch_one(lease)]) == [2.0]
    lease.close()
    producer.join(1.0)


def test_store_rejects_polling_and_duplicate_leases():
    polling = DetectionPublicationStore(source_driven=False, capacity=1)
    with pytest.raises(RuntimeError, match="polling"):
        polling.open_event_lease()

    source = DetectionPublicationStore(source_driven=True, capacity=1)
    lease = source.open_event_lease()
    with pytest.raises(RuntimeError, match="already leased"):
        source.open_event_lease()
    lease.close()


class _ChildLease:
    def __init__(self, publications) -> None:
        self._publications = tuple(publications)
        self.closed = False

    def wait_and_dispatch(self, handler):
        if self.closed or not self._publications:
            return None
        publication = self._publications[0]
        self._publications = self._publications[1:]
        return handler(publication)

    def dispatch_available(self, handler):
        if self.closed:
            return DetectionLeaseDispatch.CLOSED
        if not self._publications:
            return DetectionLeaseDispatch.EMPTY
        publication = self._publications[0]
        self._publications = self._publications[1:]
        return (
            DetectionLeaseDispatch.ACCEPTED
            if handler(publication)
            else DetectionLeaseDispatch.REJECTED
        )

    def close(self) -> None:
        self.closed = True


def _source_child(lease) -> SimpleNamespace:
    return SimpleNamespace(
        has_source_driven_detection_events=True,
        open_detection_event_lease=Mock(return_value=lease),
    )


def test_aggregator_wraps_single_source_lease_with_identity_normalization():
    target = make_detected_target(
        obj_id=4,
        x_error=0.0,
        y_error=0.0,
        k=None,
        timestamp=1.0,
    )
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    _publish(store, 1.0, obj_id=4)
    child_lease = _ChildLease(store.drain())
    child = _source_child(child_lease)
    aggregator = DetectionAggregator(
        [child],
        DetectionIdentityRegistry(),
    )

    lease = aggregator.open_detection_event_lease(DetectRequest())

    assert lease is not None
    normalized = []
    assert lease.dispatch_available(
        lambda publication: normalized.append(publication) or True
    ) is DetectionLeaseDispatch.ACCEPTED
    assert len(normalized) == 1
    normalized_target = normalized[0].detected_targets[0]
    assert normalized_target.identity.obj_id == target.identity.obj_id
    assert normalized_target.identity.task_id is not None
    assert aggregator.target_uses_source_driven_events(normalized_target) is True
    child.open_detection_event_lease.assert_called_once()
    lease.close()
    assert child_lease.closed is True


def test_aggregator_rejects_ambiguous_multi_source_lease():
    aggregator = DetectionAggregator(
        [_source_child(_ChildLease(())), _source_child(_ChildLease(()))],
        DetectionIdentityRegistry(),
    )

    with pytest.raises(RuntimeError, match="exactly one"):
        aggregator.open_detection_event_lease(DetectRequest())


def test_reset_cannot_overtake_transactional_lease_dispatch():
    store = DetectionPublicationStore(source_driven=True, capacity=2)
    _publish(store, 1.0)
    lease = store.open_event_lease()
    entered = threading.Event()
    release = threading.Event()
    delivered = []

    def handle(publication) -> bool:
        entered.set()
        assert release.wait(1.0)
        delivered.append(publication.source_timestamp_s)
        return True

    consumer = threading.Thread(
        target=lambda: lease.wait_and_dispatch(handle),
    )
    consumer.start()
    assert entered.wait(1.0)
    reset_done = threading.Event()

    def reset() -> None:
        store.clear(mark_discontinuity=True)
        reset_done.set()

    resetting = threading.Thread(target=reset)
    resetting.start()
    assert not reset_done.wait(0.05)
    release.set()
    consumer.join(1.0)
    resetting.join(1.0)

    assert delivered == [1.0]
    assert reset_done.is_set()


def test_in_flight_dispatch_does_not_block_the_next_publication():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    _publish(store, 1.0)
    lease = store.open_event_lease()
    entered = threading.Event()
    release = threading.Event()

    def handle(_publication) -> bool:
        entered.set()
        assert release.wait(1.0)
        return True

    consumer = threading.Thread(
        target=lambda: lease.wait_and_dispatch(handle),
    )
    consumer.start()
    assert entered.wait(1.0)
    published = threading.Event()
    producer = threading.Thread(
        target=lambda: (_publish(store, 2.0), published.set()),
    )
    producer.start()

    assert published.wait(1.0)
    release.set()
    consumer.join(1.0)
    producer.join(1.0)
    assert _dispatch_one(lease).source_timestamp_s == 2.0
    lease.close()


def test_reset_keeps_lease_and_marks_exactly_next_event_discontinuous():
    store = DetectionPublicationStore(source_driven=True, capacity=2)
    lease = store.open_event_lease()
    _publish(store, 1.0)

    dropped = store.clear(mark_discontinuity=True)
    _publish(store, 2.0)
    publication = _dispatch_one(lease)

    assert _timestamps(dropped) == [1.0]
    assert lease.closed is False
    assert publication.source_timestamp_s == 2.0
    assert publication.source_discontinuity is True
    lease.close()


def test_lease_open_notifies_reset_handler_when_clear_preceded_lease():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    store.clear(mark_discontinuity=True)
    reset_handler = Mock()

    lease = store.open_event_lease(reset_handler)

    reset_handler.assert_called_once_with()
    lease.close()


def test_clear_hides_pre_reset_snapshot_before_reset_callback_returns():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    lease = store.open_event_lease()
    _publish(store, 1.0)
    lease.close()
    reset_entered = threading.Event()
    release_reset = threading.Event()

    def reset_handler() -> None:
        reset_entered.set()
        assert release_reset.wait(1.0)

    lease = store.open_event_lease(reset_handler)
    reset_done = threading.Event()
    resetting = threading.Thread(
        target=lambda: (
            store.clear(mark_discontinuity=True),
            reset_done.set(),
        ),
    )
    resetting.start()
    assert reset_entered.wait(1.0)

    try:
        assert store.snapshot().detected_targets == ()
    finally:
        release_reset.set()
    resetting.join(1.0)
    assert reset_done.is_set()
    lease.close()


def test_source_stop_invokes_lease_reset_handler_before_returning():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    reset_handler = Mock()
    lease = store.open_event_lease(reset_handler)
    _publish(store, 1.0)

    store.stop()

    reset_handler.assert_called_once_with()
    assert lease.closed is True


def test_stop_racing_open_never_leaves_a_live_lease():
    for _attempt in range(20):
        store = DetectionPublicationStore(source_driven=True, capacity=1)
        start = threading.Event()
        opened = []
        rejected = []

        def open_lease() -> None:
            start.wait()
            try:
                opened.append(store.open_event_lease())
            except RuntimeError as error:
                rejected.append(error)

        opener = threading.Thread(target=open_lease)
        stopper = threading.Thread(
            target=lambda: (start.wait(), store.stop()),
        )
        opener.start()
        stopper.start()
        start.set()
        opener.join(1.0)
        stopper.join(1.0)

        assert not opener.is_alive()
        assert not stopper.is_alive()
        assert bool(opened) != bool(rejected)
        if opened:
            assert opened[0].closed is True
        else:
            assert "stopped" in str(rejected[0])
