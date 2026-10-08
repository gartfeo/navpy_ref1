"""Loss matrix of the acknowledged task assignment (design step 5).

Three TaskActors talk over real NetworkMavlink + MessageFilter on a
zero-latency in-memory bus, in virtual time. Each case loses one kind of
message (or makes a peer busy) and checks after every step: at most one
ASSIGNED helper per owner task, it is the owner's CONFIRMED peer, and no
first request copy goes to a peer the owner knows is busy.
See docs/design/swarm-task-assignment-ack.md, "Loss and busy cases".
"""

import itertools
import threading
from collections import Counter, deque
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from navpy.modules.comm.messages.msg_abc import MsgRegistry
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.task_assignment_msg import (
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
)
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskRequestMsg,
    AvailableTaskResponseMsg,
)
from navpy.modules.comm.messages.types import MsgType, TaskDispatchStatus
from navpy.modules.comm.network_mavlink import NetworkMavlink
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_actor_slots import SlotState
from tests.detection_factory import make_detected_poi

OWNER, NEAR_PEER, FAR_PEER = 1, 2, 3
TASK = 2
POI = Location(40.2967648, 44.4332133, 1339.7, is_absolute=True)
LOCATIONS = {
    OWNER: Location(40.2961244, 44.4334705, 1339.7),
    NEAR_PEER: Location(40.2967648, 44.4332133, 1339.7),
    FAR_PEER: Location(40.2974053, 44.4329561, 1339.7),
}
AWAY = Location(40.3261244, 44.4634705, 1339.7)  # farther than FAR_PEER
HEARTBEAT_S = 1.0


class VirtualTimer:
    def __init__(self, clock, interval, function, args=None, kwargs=None):
        self.clock = clock
        self.interval = interval
        self.function = function
        self.args = args or ()
        self.daemon = False
        self.due_s = None
        self.alive = False
        self.order = next(clock.orders)

    def start(self):
        self.due_s = self.clock.now_s + self.interval
        self.alive = True
        self.clock.timers.append(self)

    def cancel(self):
        self.alive = False

    def is_alive(self):
        return self.alive


class VirtualClock:
    """Monotonic time plus threading.Timer, fired in due order."""

    def __init__(self):
        self.now_s = 0.0
        self.timers = []
        self.orders = itertools.count()

    def __call__(self):
        return self.now_s

    def timer(self, interval, function, args=None, kwargs=None):
        return VirtualTimer(self, interval, function, args, kwargs)

    def next_due(self):
        alive = [timer for timer in self.timers if timer.alive]
        self.timers = alive
        return min(alive, key=lambda t: (t.due_s, t.order), default=None)

    def every(self, period_s, action):
        def tick():
            action()
            self.timer(period_s, tick).start()

        self.timer(period_s, tick).start()


class Vehicle:
    def __init__(self, node_id, bus):
        self.source_system = node_id
        self._location = LOCATIONS[node_id]
        self._bus = bus
        self.ground_speed = 20.0
        self.wind = SimpleNamespace(speed=0.0, direction=0.0)
        self.battery_level = 100.0

    def on_message(self, name, callback):
        return MagicMock()

    def send_mavlink_message(self, mav_msg):
        self._bus.send(self.source_system, mav_msg)

    def location(self, _relative=False):
        return self._location


def _decode(sender_id, mav_msg):
    mav_msg._header.srcSystem = sender_id
    return MsgRegistry.get_class_by_mav_id(mav_msg.get_msgId()).from_mavlink(mav_msg)


class Swarm:
    """Owner 1 and peers 2 (nearest the POI) and 3 on one lossy bus."""

    def __init__(self, clock):
        self.clock = clock
        self.pending = deque()
        self.log = []  # (time_s, sender, message, dropped)
        self.drop = lambda _sender, _message: False
        self.networks = {}
        self.vehicles = {}
        self.loggers = {}
        self.actors = {}
        for node_id in (OWNER, NEAR_PEER, FAR_PEER):
            vehicle = Vehicle(node_id, self)
            logger = MagicMock()
            network = NetworkMavlink(node_id, vehicle, logger)
            actor = TaskActor(vehicle, network, logger, monotonic_s=clock)
            actor.start()
            network.set_listener(actor)
            self.vehicles[node_id] = vehicle
            self.loggers[node_id] = logger
            self.networks[node_id] = network
            self.actors[node_id] = actor
        self.beat()
        for actor in self.actors.values():
            clock.every(HEARTBEAT_S, actor._presence.heartbeat)

    @property
    def owner(self):
        return self.actors[OWNER]

    def dispatch(self):
        return self.owner._auction_state.lookup(TASK)

    def beat(self):
        for actor in self.actors.values():
            actor._presence.heartbeat()
        self.drain()

    def send(self, sender_id, mav_msg):
        message = _decode(sender_id, mav_msg)
        if sender_id == OWNER and isinstance(message, TaskAssignRequestMsg):
            self._check_first_request_goes_to_a_free_peer(message)
        self.pending.append((sender_id, mav_msg, message))

    def drain(self):
        while self.pending:
            sender_id, mav_msg, message = self.pending.popleft()
            dropped = self.drop(sender_id, message)
            self.log.append((self.clock.now_s, sender_id, message, dropped))
            if dropped:
                continue
            target = getattr(mav_msg, "target_system", 0)
            for node_id, network in self.networks.items():
                if target in (0, node_id):
                    network._on_mavlink(mav_msg)
            self.check()

    def detect(self):
        self.owner.notify_pois([
            make_detected_poi(task_id=TASK, obj_id=1, p_t_g_l=POI, class_id=0),
        ])
        self.drain()

    def run_until(self, end_s):
        while (timer := self.clock.next_due()) is not None and timer.due_s <= end_s:
            self.clock.now_s = timer.due_s
            timer.alive = False
            timer.function(*timer.args)
            self.drain()
            self.check()
        self.clock.now_s = end_s

    def state(self, node_id):
        return self.actors[node_id]._selection.state()

    def assigned(self):
        return {
            node_id
            for node_id, actor in self.actors.items()
            if actor._selection.state() is SlotState.ASSIGNED
        }

    def check(self):
        """The design invariant; also enforced after every bus delivery."""
        held = [
            (node_id, actor._selection.held())
            for node_id, actor in self.actors.items()
            if actor._selection.state() is SlotState.ASSIGNED
        ]
        per_task = Counter(
            (task.owner.owner_id, task.task.task_id) for _, task in held
        )
        assert all(count <= 1 for count in per_task.values()), held
        for node_id, task in held:
            dispatch = self.actors[task.owner.owner_id]._auction_state.lookup(
                task.task.task_id,
            )
            assert dispatch.status is TaskDispatchStatus.CONFIRMED, held
            assert dispatch.assigned_peer == node_id, held

    def _check_first_request_goes_to_a_free_peer(self, request):
        dispatch = self.owner._auction_state.lookup(request.task.task_id)
        if dispatch.attempt.sends == 0:
            known_busy = self.owner._auction_state._store.peers.busy()
            assert request.receiver_id not in known_busy

    def sent(self, kind, sender=None, since_s=0.0, **fields):
        return [
            message
            for time_s, node_id, message, _dropped in self.log
            if time_s >= since_s
            and isinstance(message, kind)
            and (sender is None or node_id == sender)
            and all(getattr(message, k) == v for k, v in fields.items())
        ]

    def close(self):
        for actor in self.actors.values():
            actor.shutdown()


def _kind(message, kind, sender, sender_id, **fields):
    return (
        isinstance(message, kind)
        and sender_id == sender
        and all(getattr(message, k) == v for k, v in fields.items())
    )


def drop_first(count, kind, sender, **fields):
    """Lose the first ``count`` messages of one kind from one sender."""
    remaining = [count]

    def drop(sender_id, message):
        if remaining[0] and _kind(message, kind, sender, sender_id, **fields):
            remaining[0] -= 1
            return True
        return False

    return drop


def drop_while(clock, end_s, kind, sender, **fields):
    return lambda sender_id, message: (
        clock.now_s < end_s and _kind(message, kind, sender, sender_id, **fields)
    )


@pytest.fixture
def swarm():
    clock = VirtualClock()
    with (
        patch("threading.Timer", side_effect=clock.timer),
        patch(
            "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            threading.TIMEOUT_MAX,
        ),
    ):
        swarm = Swarm(clock)
        try:
            yield swarm
        finally:
            swarm.close()


def _confirmed_to(swarm, peer_id):
    dispatch = swarm.dispatch()
    return (
        dispatch.status is TaskDispatchStatus.CONFIRMED
        and dispatch.assigned_peer == peer_id
        and swarm.assigned() == {peer_id}
    )


# --- No loss ----------------------------------------------------------------


def test_without_loss_the_nearest_peer_is_assigned_at_once(swarm):
    swarm.detect()

    assert _confirmed_to(swarm, NEAR_PEER)
    swarm.loggers[NEAR_PEER].info.assert_any_call(
        f"Task {TASK} assigned by owner {OWNER}"
    )


# --- Loss of each message and ack -------------------------------------------


def test_lost_advert_is_re_advertised(swarm):
    swarm.drop = drop_first(1, AvailableTaskRequestMsg, OWNER)

    swarm.detect()
    assert swarm.assigned() == set()
    swarm.run_until(0.5)

    assert _confirmed_to(swarm, NEAR_PEER)


def test_lost_bid_is_re_advertised(swarm):
    swarm.drop = drop_first(1, AvailableTaskResponseMsg, NEAR_PEER)

    swarm.detect()
    swarm.run_until(0.5)

    assert _confirmed_to(swarm, NEAR_PEER)


def test_lost_request_copy_is_resent(swarm):
    swarm.drop = drop_first(1, TaskAssignRequestMsg, OWNER)

    swarm.detect()
    assert swarm.state(NEAR_PEER) is SlotState.EMPTY
    swarm.run_until(2.0)

    assert _confirmed_to(swarm, NEAR_PEER)


def test_every_request_copy_lost_releases_with_nobody_waiting(swarm):
    swarm.drop = drop_while(swarm.clock, 9.0, TaskAssignRequestMsg, OWNER)

    swarm.detect()
    swarm.run_until(8.9)
    assert swarm.state(NEAR_PEER) is SlotState.EMPTY
    assert swarm.dispatch().status is TaskDispatchStatus.CONFIRMING
    swarm.run_until(10.0)

    assert _confirmed_to(swarm, NEAR_PEER)
    assert len(swarm.sent(TaskAssignRequestMsg, OWNER)) == 4  # 3 + retry


def test_lost_request_ack_is_answered_processing_until_acked_again(swarm):
    swarm.drop = drop_first(
        1, SwarmAckMsg, NEAR_PEER,
        ref_msg_type=MsgType.TASK_ASSIGN_REQUEST.value,
    )

    swarm.detect()
    assert swarm.dispatch().status is TaskDispatchStatus.CONFIRMING
    assert swarm.sent(SwarmAckMsg, OWNER, status=ACK_STATUS_PROCESSING)
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    swarm.run_until(2.0)

    assert _confirmed_to(swarm, NEAR_PEER)


def test_lost_response_copy_is_recovered_by_the_next(swarm):
    swarm.drop = drop_first(1, TaskAssignResponseMsg, NEAR_PEER)

    swarm.detect()
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    swarm.run_until(2.0)

    assert _confirmed_to(swarm, NEAR_PEER)


def test_every_response_lost_never_flies_two_uavs(swarm):
    """The original defect: the waiting helper does not fly, the owner
    releases, and the task goes to the other peer."""
    swarm.drop = lambda sender_id, message: _kind(
        message, TaskAssignResponseMsg, NEAR_PEER, sender_id,
    )

    swarm.detect()
    swarm.run_until(8.9)
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    assert swarm.assigned() == set()
    swarm.vehicles[NEAR_PEER]._location = AWAY
    swarm.run_until(12.0)

    assert _confirmed_to(swarm, FAR_PEER)
    assert swarm.state(NEAR_PEER) is SlotState.EMPTY


def test_lost_applied_is_recovered_by_the_next_copy(swarm):
    swarm.drop = drop_first(1, SwarmAckMsg, OWNER, status=ACK_STATUS_APPLIED)

    swarm.detect()
    assert swarm.dispatch().status is TaskDispatchStatus.CONFIRMED
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    swarm.run_until(2.0)

    assert _confirmed_to(swarm, NEAR_PEER)


def test_every_applied_lost_strands_the_task_without_duplicating_it(swarm):
    swarm.drop = lambda sender_id, message: _kind(
        message, SwarmAckMsg, OWNER, sender_id, status=ACK_STATUS_APPLIED,
    )

    swarm.detect()
    swarm.run_until(17.9)
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    swarm.run_until(18.0)

    assert swarm.state(NEAR_PEER) is SlotState.EMPTY
    assert swarm.assigned() == set()
    assert swarm.dispatch().status is TaskDispatchStatus.CONFIRMED
    swarm.loggers[NEAR_PEER].error.assert_called_once()


# --- Busy peers, fuel and silence ---------------------------------------------


def test_busy_peer_is_skipped(swarm):
    swarm.actors[NEAR_PEER].set_approaching(True)
    swarm.beat()

    swarm.detect()

    assert _confirmed_to(swarm, FAR_PEER)
    assert swarm.sent(AvailableTaskResponseMsg, NEAR_PEER) == []


def test_all_peers_busy_get_no_adverts_until_one_is_free(swarm):
    for peer_id in (NEAR_PEER, FAR_PEER):
        swarm.actors[peer_id].set_approaching(True)
    swarm.beat()

    swarm.detect()
    swarm.run_until(3.0)
    assert swarm.sent(AvailableTaskRequestMsg, OWNER, since_s=0.5) == []
    swarm.actors[FAR_PEER].set_approaching(False)
    swarm.run_until(4.0)

    assert _confirmed_to(swarm, FAR_PEER)


def test_own_final_approach_while_waiting_rejects_and_the_owner_retries(swarm):
    swarm.drop = lambda sender_id, message: (
        _kind(message, TaskAssignResponseMsg, NEAR_PEER, sender_id, is_accepted=True)
    )
    swarm.detect()
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    swarm.run_until(1.5)

    swarm.actors[NEAR_PEER].set_approaching(True)
    swarm.run_until(1.5)  # the deferred first reject copy

    assert swarm.state(NEAR_PEER) is SlotState.EMPTY
    assert swarm.sent(TaskAssignResponseMsg, NEAR_PEER, is_accepted=False)
    assert _confirmed_to(swarm, FAR_PEER)


def test_peer_without_fuel_declines_and_is_left_out(swarm):
    swarm.vehicles[NEAR_PEER].battery_level = 10.0

    swarm.detect()

    (decline,) = swarm.sent(AvailableTaskResponseMsg, NEAR_PEER)
    assert decline.tasks[0].time_in_min == -1.0
    assert _confirmed_to(swarm, FAR_PEER)


def test_lost_release_advert_still_reaches_the_waiting_peer(swarm):
    # Peer 3 cannot fly the task, so it stays open after the release: the
    # waiting peer is the only candidate and must hear the re-advert.
    swarm.vehicles[FAR_PEER].battery_level = 10.0
    lose_release_adverts = drop_first(2, AvailableTaskRequestMsg, OWNER)
    swarm.drop = lambda sender_id, message: (
        swarm.clock.now_s < 9.0
        and _kind(message, TaskAssignResponseMsg, NEAR_PEER, sender_id)
    ) or (swarm.clock.now_s >= 9.0 and lose_release_adverts(sender_id, message))
    swarm.detect()

    swarm.run_until(9.0)
    assert swarm.dispatch().status is TaskDispatchStatus.AVAILABLE
    assert swarm.state(NEAR_PEER) is SlotState.WAITING
    swarm.run_until(9.5)

    # Advertised again to the released, still-busy peer: it stops waiting
    # long before its own 18 s expiry, bids, and gets the task.
    assert _confirmed_to(swarm, NEAR_PEER)


def test_silent_peer_is_skipped_and_revived_by_its_next_beat(swarm):
    swarm.drop = lambda sender_id, _message: sender_id == FAR_PEER

    swarm.detect()
    swarm.run_until(5.0)
    assert swarm.dispatch().status is TaskDispatchStatus.AVAILABLE
    swarm.run_until(6.0)

    assert _confirmed_to(swarm, NEAR_PEER)
    assert FAR_PEER in swarm.owner._auction_state._store.peers.busy()
    swarm.drop = lambda _sender, _message: False
    swarm.run_until(7.0)
    assert FAR_PEER not in swarm.owner._auction_state._store.peers.busy()
