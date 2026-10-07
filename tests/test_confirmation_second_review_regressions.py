"""Consolidated second-review regressions for confirmation and swarm ownership."""

from __future__ import annotations

import ast
import threading
from pathlib import Path
from unittest.mock import Mock

import pytest

from gcs.backend.routes.task_confirm import _response_meta
from gcs.backend.task_confirm_images import ActiveImage
from gcs.backend.task_confirm_listener import TaskConfirmListener
from gcs.backend.task_confirm_rounds import ConfirmationRoundRegistry, RequestAction
from gcs.backend.task_confirm_transport import parse_round_uid
from navpy.exception_groups import ExceptionGroup
from navpy.modules.comm.messages.available_task_msg import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import (
    MsgType,
    TaskDispatchStatus,
    TaskTypeMsgData,
)
from navpy.modules.common.models.location import Location
from navpy.modules.nav.confirmation_dependencies import FreshnessGate
from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationResponseKind,
)
from navpy.modules.nav.peer_poi_dispatch import (
    PeerPoiDispatchPorts,
    PeerPoiDispatchWorker,
)
from navpy.modules.nav.self_assignment_publisher import SelfAssignmentPublisher
from navpy.modules.nav.confirmation_coordinator import (
    ConfirmationCoordinator,
)
from navpy.modules.nav.confirmation_media import ConfirmationMedia
from navpy.modules.nav.confirmation_manager_state import ConfirmationManagerState, ConfirmationStatus
from navpy.modules.swarm.task_actor_state import create_task_state
from navpy.modules.swarm.task_assignment_planner import MinimumEtaAssignmentPlanner
from tests.detection_factory import make_detected_poi


REPO_ROOT = Path(__file__).resolve().parents[1]


class _InlineThread:
    def __init__(self, *, target, args, daemon) -> None:
        self._target = target
        self._args = args

    def start(self) -> None:
        self._target(*self._args)


@pytest.mark.parametrize(
    ("auto_confirm", "network"),
    [(True, object()), (False, None)],
    ids=["auto-confirm", "no-network"],
)
def test_new_review_invalidates_prior_recall_authority(
    auto_confirm: bool,
    network: object | None,
) -> None:
    state = ConfirmationManagerState()
    poi = make_detected_poi(obj_id=7, task_id=7)
    old_ref = ConfirmationRequestRef(101, 1)
    old_worker = state.start_review(poi, threading.Event)
    old_round = state.begin_round(poi, old_worker, threading.Event, old_ref)
    assert old_round is not None
    assert state.complete_round(old_round, ConfirmationStatus.CONFIRMED)
    state.finish_worker(old_worker)

    coordinator = ConfirmationCoordinator(
        sys_id=1,
        state=state,
        network=lambda: network,
        auto_confirm=lambda: auto_confirm,
        assignment=Mock(),
        round_runner=Mock(),
        logger=Mock(),
        event_factory=threading.Event,
        thread_factory=lambda **kwargs: _InlineThread(**kwargs),
    )
    coordinator.review([poi])
    assert state.registry.status_by_id(7) is ConfirmationStatus.CONFIRMED

    result = state.resolve_response(7, False, old_ref)

    assert result is ConfirmationResponseKind.LATE_OR_DUPLICATE
    assert state.registry.status_by_id(7) is ConfirmationStatus.CONFIRMED


def _task(task_id: int) -> TaskMsgData:
    return TaskMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(1.0, 2.0, 3.0),
    )


def _reserve(task_id: int = 7, peer_id: int = 2):
    _, auction, roster, _ = create_task_state(threading.RLock())
    assert roster.discover_peer(peer_id, lambda *_: None)
    dispatch, generation = auction.register(_task(task_id))
    dispatch.on_peer_available(
        peer_id,
        TaskHandleMsgData(task_id=task_id, time_in_min=1.0),
    )
    reservations = auction.plan_and_reserve(
        MinimumEtaAssignmentPlanner(),
        generation,
    )
    assert len(reservations) == 1
    return auction, roster, dispatch


def test_auction_rejects_unassigned_responses() -> None:
    _, auction, roster, _ = create_task_state(threading.RLock())
    assert roster.discover_peer(2, lambda *_: None)
    dispatch, _ = auction.register(_task(7))

    assert not auction.accept(7, 2)
    assert auction.reject(7, 2).kind == "missing"
    assert dispatch.status is TaskDispatchStatus.AVAILABLE
    assert dispatch.assigned_peer is None


def test_auction_rejects_wrong_reserved_peer() -> None:
    auction, roster, dispatch = _reserve()
    assert roster.discover_peer(3, lambda *_: None)

    assert not auction.accept(7, 3)
    assert auction.reject(7, 3).kind == "missing"
    assert dispatch.status is TaskDispatchStatus.CONFIRMING
    assert dispatch.assigned_peer == 2


def test_auction_rejects_stale_response_after_reset_and_id_reuse() -> None:
    auction, _, _ = _reserve()
    auction.reset(Mock())
    _, _, roster, _ = create_task_state(threading.RLock())
    del roster  # control against accidentally using a different state
    assert auction._store.peers.add(3)
    dispatch, generation = auction.register(_task(7))
    dispatch.on_peer_available(3, TaskHandleMsgData(task_id=7, time_in_min=1.0))
    assert auction.plan_and_reserve(MinimumEtaAssignmentPlanner(), generation)

    assert not auction.accept(7, 2)
    assert auction.reject(7, 2).kind == "missing"
    assert dispatch.status is TaskDispatchStatus.CONFIRMING
    assert dispatch.assigned_peer == 3


def test_auction_accept_and_reject_are_single_use() -> None:
    auction, _, dispatch = _reserve()
    assert auction.accept(7, 2)
    assert not auction.accept(7, 2)
    assert auction.reject(7, 2).kind == "missing"
    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    assert dispatch.assigned_peer == 2


def test_auction_reject_is_single_use() -> None:
    auction, _, dispatch = _reserve()
    assert auction.reject(7, 2).kind != "missing"
    assert auction.reject(7, 2).kind == "missing"
    assert dispatch.status is TaskDispatchStatus.AVAILABLE
    assert dispatch.assigned_peer is None


class _ReleaseBarrierLock:
    def __init__(self, after_release) -> None:
        self._lock = threading.Lock()
        self._after_release = after_release

    def __enter__(self):
        self._lock.acquire()
        return self

    def __exit__(self, *_exc_info):
        self._lock.release()
        callback, self._after_release = self._after_release, None
        if callback is not None:
            callback()
        return False


def _image_listener(uid: str = "1:1"):
    listener = TaskConfirmListener(Mock())
    listener._rounds.classify(1, 7, uid, 1.0, 120.0)
    listener._images.current_pois[1] = ActiveImage(7, uid)
    reassembler = Mock()
    reassembler.on_chunk.return_value = "aW1hZ2U="
    listener._images.reassemblers[1] = reassembler
    listener._images._is_companion = lambda *_: True
    broadcasts: list[dict] = []
    listener._images._broadcast = broadcasts.append
    return listener, broadcasts


def test_completed_image_is_rejected_after_round_closes() -> None:
    listener, broadcasts = _image_listener()
    listener._rounds.mark_decided(1, 7, True, "1:1", 2.0)

    listener._images.on_chunk(1, object())

    assert broadcasts == []
    assert (1, 7) not in listener._rounds._image_received


def test_image_acceptance_does_not_mark_replacement_round_at_lock_barrier() -> None:
    listener, _ = _image_listener()
    listener._rounds._lock = _ReleaseBarrierLock(
        lambda: listener._rounds.classify(1, 7, "1:2", 2.0, 120.0)
    )

    listener._images.on_chunk(1, object())

    assert listener._rounds.open_uid(1, 7) == "1:2"
    assert (1, 7) not in listener._rounds._image_received


def test_peer_notify_oserror_keeps_poi_retryable() -> None:
    statuses: set[int] = set()
    failed = threading.Event()
    succeeded = threading.Event()
    attempts = 0

    def notify(_pois) -> None:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise OSError("radio unavailable")
        succeeded.set()

    worker = PeerPoiDispatchWorker(PeerPoiDispatchPorts(
        active_poi=lambda: None,
        resolve_location=lambda _poi: Location(1.0, 2.0, 3.0),
        status_exists=lambda poi: poi.identity.task_id in statuses,
        mark_notified=lambda poi: statuses.add(poi.identity.task_id),
        notify_pois=notify,
        warn=lambda _message: None,
        error=lambda _message, _error: failed.set(),
    ))
    worker.start()
    try:
        worker.submit([make_detected_poi(obj_id=7, task_id=7)])
        assert failed.wait(1.0)
        worker.submit([make_detected_poi(obj_id=7, task_id=7)])
        assert succeeded.wait(1.0)
        assert attempts == 2
        assert statuses == {7}
    finally:
        worker.stop()


def test_uncorrelated_decision_does_not_clear_round_opened_during_send_window() -> None:
    registry = ConfirmationRoundRegistry()
    assert registry.classify(1, 7, "1:1", 1.0, 120.0)[0] is RequestAction.POPUP
    barrier = threading.Barrier(2)

    def open_replacement() -> None:
        barrier.wait()
        registry.classify(1, 7, "1:2", 2.0, 120.0)
        barrier.wait()

    thread = threading.Thread(target=open_replacement)
    thread.start()
    barrier.wait()
    barrier.wait()
    registry.mark_decided(1, 7, True, None, 3.0)
    thread.join(timeout=1.0)

    assert registry.open_uid(1, 7) == "1:2"


def test_late_older_decision_does_not_overwrite_newer_retained_decision() -> None:
    registry = ConfirmationRoundRegistry()
    registry.classify(1, 7, "1:1", 1.0, 120.0)
    registry.classify(1, 7, "1:2", 2.0, 120.0)
    registry.mark_decided(1, 7, False, "1:2", 3.0)

    # Round A's slower x3 route burst finishes after B is already decided.
    registry.mark_decided(1, 7, True, "1:1", 4.0)
    action, is_confirmed = registry.classify(1, 7, "1:2", 5.0, 120.0)

    assert action is RequestAction.ANSWER
    assert is_confirmed is False


def test_legacy_route_without_round_uid_uses_uncorrelated_meta() -> None:
    assert _response_meta(None) is None


@pytest.mark.parametrize("uid", ["01:002", "00:1", "1:00", "١:٢"])
def test_noncanonical_round_uids_are_rejected(uid: str) -> None:
    with pytest.raises(ValueError):
        parse_round_uid(uid)


def test_registry_rejects_noncanonical_round_uid() -> None:
    with pytest.raises(ValueError):
        ConfirmationRoundRegistry().classify(1, 7, "01:002", 1.0, 120.0)


def test_freshness_programmer_error_propagates() -> None:
    gate = FreshnessGate(Mock())
    gate.set_check(Mock(side_effect=TypeError("bad freshness signature")))
    with pytest.raises(TypeError, match="bad freshness signature"):
        gate.is_fresh(7)


def test_self_assignment_programmer_error_propagates() -> None:
    network = Mock()
    network.broadcast.side_effect = TypeError("bad broadcast signature")
    publisher = SelfAssignmentPublisher(
        sys_id=1,
        is_simulation=False,
        network=lambda: network,
        logger=Mock(),
    )
    poi = make_detected_poi(
        obj_id=7,
        task_id=7,
        p_t_g_l=Location(1.0, 2.0, 3.0),
    )
    with pytest.raises(TypeError, match="bad broadcast signature"):
        publisher.publish(poi)


def test_confirmation_artifact_programmer_error_propagates() -> None:
    network = Mock()
    network.send_image.return_value = 1
    logger = Mock(log_path="logs")
    media = ConfirmationMedia(
        sys_id=1,
        network=lambda: network,
        logger=logger,
        thumbnail_builder=lambda **_kwargs: "aW1hZ2U=",
        artifact_saver=Mock(side_effect=TypeError("bad artifact signature")),
    )
    poi = make_detected_poi(
        obj_id=7,
        task_id=7,
        detection_frame=object(),
        bbox_cxcywh=(1.0, 1.0, 1.0, 1.0),
    )
    with pytest.raises(TypeError, match="bad artifact signature"):
        media.send(poi, 7)


def test_auction_cleanup_attempts_all_and_surfaces_exception_group() -> None:
    _, auction, _, _ = create_task_state(threading.RLock())
    first, _ = auction.register(_task(1))
    second, _ = auction.register(_task(2))
    first.shutdown = Mock(side_effect=TypeError("first"))
    second.shutdown = Mock(side_effect=OSError("second"))

    with pytest.raises(ExceptionGroup) as raised:
        auction.reset(Mock())

    first.shutdown.assert_called_once_with()
    second.shutdown.assert_called_once_with()
    assert {type(error) for error in raised.value.exceptions} == {TypeError, OSError}


@pytest.mark.parametrize(
    ("relative", "forbidden"),
    [
        ("src/navpy/modules/nav/confirmation_dependencies.py", {"NetworkAbc"}),
        (
            "src/navpy/modules/nav/confirmation_round_runner.py",
            {"NetworkAbc", "ConfirmationManagerState"},
        ),
        (
            "src/navpy/modules/nav/confirmation_coordinator.py",
            {"NetworkAbc", "ConfirmationManagerState"},
        ),
        (
            "src/navpy/modules/nav/confirmation_inbound.py",
            {"ConfirmationManagerState"},
        ),
        ("src/navpy/modules/nav/self_assignment_publisher.py", {"NetworkAbc"}),
        ("src/navpy/modules/swarm/task_messaging.py", {"NetworkAbc"}),
        ("src/navpy/modules/swarm/swarm_presence.py", {"IVehicle"}),
        ("src/navpy/modules/swarm/task_capability.py", {"IVehicle"}),
    ],
)
def test_internal_owners_depend_on_narrow_ports(
    relative: str,
    forbidden: set[str],
) -> None:
    tree = ast.parse((REPO_ROOT / relative).read_text(encoding="utf-8"))
    imported = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert imported.isdisjoint(forbidden)


def test_task_actor_has_no_network_clock_reachthrough() -> None:
    source = (REPO_ROOT / "src/navpy/modules/swarm/task_actor.py").read_text(
        encoding="utf-8"
    )
    assert "network.message_filter.offset_estimator" not in source
