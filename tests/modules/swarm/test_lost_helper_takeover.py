"""A CONFIRMED task whose helper is lost: kept (policy keep) or re-auctioned
(policy auto), see core/swarm/task_lost_helper.py.

The bus cases run the loss-matrix swarm (three real TaskActors on one
in-memory MAVLink bus, virtual time). A killed node sends and hears nothing
from then on. That bus shares one boot id across its nodes (the process
singleton), so a restarted boot is covered at the coordinator level.
"""

import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.link.task_assignment_msg import TaskAssignRequestMsg
from navpy.core.link.task_availability_msg import AvailableTaskResponseMsg
from navpy.core.link.task_message_data import TaskMsgData
from navpy.core.link.types import TaskDispatchStatus, TaskTypeMsgData
from navpy.core.swarm.task_actor_slots import SlotState
from navpy.core.swarm.task_lost_helper import (
    LOST_HELPER_KEPT_LOG,
    LOST_HELPER_RELEASED_LOG,
    LostHelperPolicy,
)
from navpy.core.swarm.task_msg_refs import MsgRef
from navpy.core.swarm.task_peer_roster import PEER_SILENCE_EXPIRY_S
from navpy.core.swarm.task_peer_status import PeerStatusCoordinator
from navpy.core.swarm.task_state_composition import create_task_state
from tests.detection_factory import make_detected_poi
from tests.modules.swarm.test_task_assign_loss_matrix import (
    FAR_PEER,
    HEARTBEAT_S,
    NEAR_PEER,
    OWNER,
    POI,
    TASK,
    Swarm,
    VirtualClock,
    _confirmed_to,
)

KEEP, AUTO = LostHelperPolicy.KEEP, LostHelperPolicy.AUTO
FREE, BUSY = SwarmNodeState.FREE, SwarmNodeState.BUSY
# The owner's first heartbeat tick after the killed helper's TTL ran out.
SILENT_AT_S = PEER_SILENCE_EXPIRY_S + HEARTBEAT_S


@pytest.fixture
def make_swarm():
    clock = VirtualClock()
    made = []
    with (
        patch("threading.Timer", side_effect=clock.timer),
        patch(
            "navpy.core.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            threading.TIMEOUT_MAX,
        ),
    ):
        def make(policy):
            swarm = Swarm(clock, policy)
            swarm.dead = []
            made.append(swarm)
            return swarm

        try:
            yield make
        finally:
            for swarm in made:
                swarm.close()
                for actor in swarm.dead:
                    actor.shutdown()


def kill(swarm, node_id):
    """Hard kill: from now on the node sends and hears nothing, and sends
    no check-out. Copies it already queued are still delivered."""
    swarm.vehicles[node_id].send_mavlink_message = lambda _mav_msg: None
    swarm.networks.pop(node_id)
    swarm.dead.append(swarm.actors.pop(node_id))


def rejoin(swarm, node_id):
    actor = swarm.join(node_id)
    swarm.clock.every(HEARTBEAT_S, actor._presence.heartbeat)
    swarm.drain()
    return actor


def released(swarm, reason):
    swarm.loggers[OWNER].warning.assert_any_call(
        LOST_HELPER_RELEASED_LOG.format(task=TASK, peer=NEAR_PEER, reason=reason)
    )


# --- Bus: the owner of a task loses its flying helper ------------------------


def test_auto_re_auctions_a_killed_helpers_task_and_the_other_peer_takes_it(
    make_swarm,
):
    swarm = make_swarm(AUTO)
    swarm.detect()
    assert _confirmed_to(swarm, NEAR_PEER)

    kill(swarm, NEAR_PEER)
    swarm.run_until(PEER_SILENCE_EXPIRY_S)
    assert swarm.dispatch().status is TaskDispatchStatus.CONFIRMED
    assert swarm.dispatch().assigned_peer == NEAR_PEER

    swarm.run_until(SILENT_AT_S)

    released(swarm, "silent")
    # The remaining peer bid again on the re-advertised task and flies it.
    assert swarm.sent(AvailableTaskResponseMsg, FAR_PEER, since_s=SILENT_AT_S)
    assert _confirmed_to(swarm, FAR_PEER)


def test_keep_leaves_a_killed_helpers_task_with_it(make_swarm):
    swarm = make_swarm(KEEP)
    swarm.detect()
    kill(swarm, NEAR_PEER)

    swarm.run_until(SILENT_AT_S + 5.0)

    dispatch = swarm.dispatch()
    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    assert dispatch.assigned_peer == NEAR_PEER
    assert swarm.assigned() == set()
    assert not swarm.sent(TaskAssignRequestMsg, OWNER, since_s=0.5)
    swarm.loggers[OWNER].warning.assert_any_call(
        LOST_HELPER_KEPT_LOG.format(task=TASK, peer=NEAR_PEER, reason="silent")
    )


def test_auto_takes_the_task_back_at_once_on_check_out(make_swarm):
    swarm = make_swarm(AUTO)
    swarm.detect()

    swarm.actors[NEAR_PEER].checkout()
    kill(swarm, NEAR_PEER)
    swarm.drain()
    released(swarm, "checked out")

    swarm.run_until(0.1)  # the re-advert's timer; no silence wait
    assert _confirmed_to(swarm, FAR_PEER)


def test_auto_never_takes_back_a_task_its_helper_reported_done(make_swarm):
    swarm = make_swarm(AUTO)
    swarm.detect()
    swarm.actors[NEAR_PEER].clear_selected_poi()  # nav finished the task
    swarm.run_until(HEARTBEAT_S)  # its next heartbeat reports FREE

    kill(swarm, NEAR_PEER)
    swarm.run_until(SILENT_AT_S + HEARTBEAT_S)

    dispatch = swarm.dispatch()
    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    assert dispatch.assigned_peer == NEAR_PEER
    assert swarm.assigned() == set()
    for call in swarm.loggers[OWNER].warning.call_args_list:
        assert "lost" not in call.args[0]


def test_a_killed_helper_that_comes_back_is_heard_again_and_wins_a_task(
    make_swarm,
):
    swarm = make_swarm(AUTO)
    swarm.detect()
    kill(swarm, NEAR_PEER)
    swarm.run_until(SILENT_AT_S)
    assert _confirmed_to(swarm, FAR_PEER)

    back = rejoin(swarm, NEAR_PEER)
    swarm.run_until(SILENT_AT_S + HEARTBEAT_S)
    peers = swarm.owner._auction_state._store.peers
    assert not peers.status(NEAR_PEER).silent

    swarm.owner.notify_pois([
        make_detected_poi(task_id=TASK + 1, obj_id=2, p_t_g_l=POI, class_id=0),
    ])
    swarm.drain()

    second = swarm.owner._auction_state.lookup(TASK + 1)
    assert second.status is TaskDispatchStatus.CONFIRMED
    assert second.assigned_peer == NEAR_PEER
    assert back._selection.state() is SlotState.ASSIGNED



def _task(task_id):
    return TaskMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(1.0, 2.0, 3.0),
    )


def test_auto_never_takes_back_a_live_helpers_task(make_swarm):
    # The helper's own heartbeats keep the boot that acked it heard.
    swarm = make_swarm(AUTO)
    swarm.detect()
    assert _confirmed_to(swarm, NEAR_PEER)

    swarm.run_until(4 * SILENT_AT_S)

    dispatch = swarm.dispatch()
    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    assert dispatch.assigned_peer == NEAR_PEER
    _no_loss_logged(swarm.loggers[OWNER])


# --- Coordinator: one boot of the helper holds the task ----------------------
#
# The bus shares one boot id across its nodes, so boots are covered here:
# the owner's real task state with a fake clock, presence mocked.


class _Clock:
    def __init__(self) -> None:
        self.now_s = 100.0

    def __call__(self) -> float:
        return self.now_s


def _owner(policy):
    clock = _Clock()
    _, auction, rebroadcast_state, _ = create_task_state(
        threading.RLock(), clock,
    )
    for peer in (2, 3):
        rebroadcast_state.discover_peer(peer, lambda *_: None)
    rebroadcast = Mock()
    rebroadcast.peer_heard.return_value = False
    rebroadcast.peer_checked_out.return_value = False
    rebroadcast.expire_silent_peers.return_value = False
    replan, logger = Mock(), Mock()
    status = PeerStatusCoordinator(
        auction, rebroadcast, replan, logger, lambda: policy,
    )
    return SimpleNamespace(
        status=status, auction=auction, replan=replan, logger=logger,
        clock=clock,
    )


def _reserve(owner, task_id, peer_id, boot_id, floor_seq=5):
    """Task reserved to the peer and acked by its boot (CONFIRMING with a
    floor), as the owner's confirmation sets it on the first ack."""
    dispatch, _ = owner.auction.register(_task(task_id))
    owner.auction.observe_peer(  # its bid
        peer_id, FREE, MsgRef(peer_id, boot_id, floor_seq - 1),
    )
    floor = MsgRef(peer_id, boot_id, floor_seq)
    dispatch.assigned_peer = peer_id
    dispatch.status = TaskDispatchStatus.CONFIRMING
    dispatch.attempt.floor = floor
    dispatch.attempt.holder_heard = floor
    dispatch.attempt.holder_heard_s = owner.clock()
    return dispatch


def _confirm(owner, task_id, peer_id, boot_id, floor_seq=5):
    dispatch = _reserve(owner, task_id, peer_id, boot_id, floor_seq)
    dispatch.attempt.note_holder_accept(
        MsgRef(peer_id, boot_id, floor_seq + 1),
    )
    dispatch.peer_accept(peer_id)
    owner.auction.observe_peer(  # its accepted step-4 copy
        peer_id, BUSY, MsgRef(peer_id, boot_id, floor_seq + 1),
    )
    return dispatch


def _beat(owner, peer_id, boot_id, seq, state=BUSY):
    owner.status.observe_peer(peer_id, state, MsgRef(peer_id, boot_id, seq))


def _wait(owner, seconds):
    owner.clock.now_s += seconds
    owner.status.check_silence()


def _no_loss_logged(logger):
    for call in logger.warning.call_args_list:
        assert "lost" not in call.args[0]


def _lost_log(template, task, reason):
    return template.format(task=task, peer=2, reason=reason)


AFTER_EXPIRY_S = PEER_SILENCE_EXPIRY_S + 0.1


def test_auto_takes_a_restarted_helpers_task_back_once():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)

    # Boot 7 is gone; the new boot 8 talks (FREE: it holds nothing).
    for seq in range(1, 7):
        _beat(owner, 2, 8, seq, FREE)
        _wait(owner, 1.0)

    assert dispatch.status is TaskDispatchStatus.AVAILABLE
    assert dispatch.assigned_peer is None
    owner.replan.assert_called()
    owner.logger.warning.assert_called_once_with(
        _lost_log(LOST_HELPER_RELEASED_LOG, 9, "restarted"),
    )
    _wait(owner, AFTER_EXPIRY_S)
    assert owner.logger.warning.call_count == 1


def test_keep_logs_a_lost_helper_once_and_keeps_its_task():
    owner = _owner(KEEP)
    dispatch = _confirm(owner, 9, 2, boot_id=7)

    _wait(owner, AFTER_EXPIRY_S)
    _wait(owner, AFTER_EXPIRY_S)

    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    assert dispatch.assigned_peer == 2
    owner.logger.warning.assert_called_once_with(
        _lost_log(LOST_HELPER_KEPT_LOG, 9, "silent"),
    )
    owner.replan.assert_not_called()


def test_a_helper_boot_heard_within_the_ttl_keeps_its_task():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)

    for seq in range(10, 30):
        _wait(owner, PEER_SILENCE_EXPIRY_S - 0.5)
        _beat(owner, 2, 7, seq)

    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    _no_loss_logged(owner.logger)


def test_late_copies_of_the_helper_boot_do_not_keep_it_alive():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)
    _beat(owner, 2, 7, 20)  # its last heartbeat

    _wait(owner, 3.0)
    _beat(owner, 2, 7, 19)  # a late copy of an older one
    _wait(owner, PEER_SILENCE_EXPIRY_S - 3.0 + 0.1)

    assert dispatch.status is TaskDispatchStatus.AVAILABLE


def test_a_late_copy_of_an_unseen_older_boot_spares_the_live_boots_task():
    # Review round 2, finding 1: the roster record may flip to the older
    # boot (as before this change), but the holder boot is still heard.
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=8)

    _beat(owner, 2, 6, 900)  # never seen before, delayed
    for seq in range(10, 20):
        _wait(owner, 1.0)
        _beat(owner, 2, 8, seq)

    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    _no_loss_logged(owner.logger)
    assert owner.auction._store.peers.status(2).order == MsgRef(2, 8, 19)


def test_a_late_check_out_of_an_older_boot_spares_the_live_boots_task():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=8)

    owner.status.peer_checked_out(2, MsgRef(2, 7, 30))

    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    _no_loss_logged(owner.logger)
    # Review round 2, finding 3: the stale check-out silenced the peer, yet
    # the live boot dying right after is still caught.
    _wait(owner, AFTER_EXPIRY_S)
    assert dispatch.status is TaskDispatchStatus.AVAILABLE
    owner.logger.warning.assert_called_once_with(
        _lost_log(LOST_HELPER_RELEASED_LOG, 9, "silent"),
    )


def test_a_check_out_before_the_ack_spares_the_task():
    # The same boot checked out and rejoined (a new actor in one process)
    # before it acked this reservation.
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7, floor_seq=50)

    owner.status.peer_checked_out(2, MsgRef(2, 7, 40))

    assert dispatch.status is TaskDispatchStatus.CONFIRMED


def test_a_check_out_before_a_newer_acceptance_spares_the_task():
    # Review round 4, finding 1: ack 5 and acceptance 6, check-out 7, then
    # a new actor of the same boot accepted again (11, REPEAT) and flies
    # it; the check-out arrives late.
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)
    dispatch.attempt.note_holder_accept(MsgRef(2, 7, 11))

    owner.status.peer_checked_out(2, MsgRef(2, 7, 7))

    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    _no_loss_logged(owner.logger)


def test_auto_takes_the_task_back_at_once_on_its_boots_check_out():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)

    owner.status.peer_checked_out(2, MsgRef(2, 7, 30))

    assert dispatch.status is TaskDispatchStatus.AVAILABLE
    owner.replan.assert_called_once_with()
    owner.logger.warning.assert_called_once_with(
        _lost_log(LOST_HELPER_RELEASED_LOG, 9, "checked out"),
    )


def test_a_check_out_without_a_uid_only_silences():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)

    owner.status.peer_checked_out(2, None)

    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    _wait(owner, AFTER_EXPIRY_S)  # its boot falls silent instead
    assert dispatch.status is TaskDispatchStatus.AVAILABLE


@pytest.mark.parametrize("policy", [KEEP, AUTO])
def test_a_check_out_while_confirming_to_that_boot(policy):
    owner = _owner(policy)
    acked = _reserve(owner, 9, 2, boot_id=7)
    attempt = acked.attempt.number

    owner.status.peer_checked_out(2, MsgRef(2, 7, 30))

    if policy is KEEP:
        assert acked.status is TaskDispatchStatus.CONFIRMING
        _no_loss_logged(owner.logger)
        return
    # Released under a new attempt: a late answer of that boot no longer
    # matches the reservation (peer and attempt), so it cannot confirm it.
    assert acked.status is TaskDispatchStatus.AVAILABLE
    assert acked.attempt.number == attempt + 1
    assert acked.attempt.floor is None


def test_a_reservation_no_boot_acked_is_never_taken_back():
    # Review round 2, finding 5: the reservation may be the new boot's.
    owner = _owner(AUTO)
    _confirm(owner, 9, 2, boot_id=7)
    fresh, _ = owner.auction.register(_task(10))
    fresh.assigned_peer = 2
    fresh.status = TaskDispatchStatus.CONFIRMING

    owner.status.peer_checked_out(2, MsgRef(2, 7, 30))
    _wait(owner, AFTER_EXPIRY_S)

    assert fresh.status is TaskDispatchStatus.CONFIRMING
    assert fresh.assigned_peer == 2


def test_silence_leaves_a_confirming_reservation_to_the_confirm_timers():
    owner = _owner(AUTO)
    acked = _reserve(owner, 9, 2, boot_id=7)

    _wait(owner, AFTER_EXPIRY_S)

    assert acked.status is TaskDispatchStatus.CONFIRMING
    _no_loss_logged(owner.logger)


def test_a_restart_first_heard_in_a_bid_is_still_caught():
    # Review round 2, finding 4: no heartbeat of the old boot ever passed
    # the coordinator; the holder boot's own silence decides.
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)

    owner.auction.observe_peer(2, FREE, MsgRef(2, 8, 1))  # new boot's bid
    _wait(owner, AFTER_EXPIRY_S)

    assert dispatch.status is TaskDispatchStatus.AVAILABLE
    owner.logger.warning.assert_called_once_with(
        _lost_log(LOST_HELPER_RELEASED_LOG, 9, "restarted"),
    )


def test_a_late_free_report_behind_a_newer_busy_still_spares_the_task():
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)
    _beat(owner, 2, 7, 8)  # BUSY with a newer task

    # FREE sent after the acceptance, delivered behind the BUSY: the roster
    # ignores it, but it still proves the helper let task 9 go.
    owner.auction.observe_peer(2, FREE, MsgRef(2, 7, 7))
    _wait(owner, AFTER_EXPIRY_S)

    assert dispatch.attempt.helper_freed()
    assert dispatch.status is TaskDispatchStatus.CONFIRMED
    _no_loss_logged(owner.logger)


def test_a_late_free_older_than_the_acceptance_does_not_spare_the_task():
    # Review round 3, finding 2: the helper dropped an earlier WAITING slot
    # (FREE seq 8), took the task again (accepted seq 10, ASSIGNED); the
    # FREE arrives only now. It is older than the acceptance it flies.
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7, floor_seq=9)

    owner.auction.observe_peer(2, FREE, MsgRef(2, 7, 8))
    assert not dispatch.attempt.helper_freed()
    _wait(owner, AFTER_EXPIRY_S)

    assert dispatch.status is TaskDispatchStatus.AVAILABLE


def test_an_acceptance_after_a_free_makes_the_helper_hold_the_task_again():
    # FREE seq 7, then the helper re-entered WAITING through a late request
    # copy and its accepted seq 9 got APPLIED (verdict REPEAT): it flies.
    owner = _owner(AUTO)
    dispatch = _confirm(owner, 9, 2, boot_id=7)
    owner.auction.observe_peer(2, FREE, MsgRef(2, 7, 7))
    assert dispatch.attempt.helper_freed()

    dispatch.attempt.note_holder_accept(MsgRef(2, 7, 9))
    _wait(owner, AFTER_EXPIRY_S)

    assert not dispatch.attempt.helper_freed()
    assert dispatch.status is TaskDispatchStatus.AVAILABLE
