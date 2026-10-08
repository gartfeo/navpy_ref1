"""In-memory MAVLink regression for the three-UAV task auction."""

import threading
from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE,
)

from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.comm.network_mavlink import NetworkMavlink
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_actor_slots import SlotState
from navpy.modules.swarm.task_dispatch import TaskDispatch
from tests.detection_factory import make_detected_poi
from tests.modules.swarm.test_task_assign_confirmation import FakeTimers


@pytest.fixture(autouse=True)
def explicit_heartbeats():
    """Heartbeats carry node state: tests send them explicitly instead of
    letting the 1 Hz thread inject wall-clock reports."""
    with patch(
        "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
        threading.TIMEOUT_MAX,
    ):
        yield


class _MavlinkBus:
    def __init__(self):
        self.networks = []
        self.pending = deque()
        # Best-effort transport: drop(sender_id, mav_msg) -> True loses it.
        self.drop = lambda _sender_id, _mav_msg: False

    def enqueue(self, sender, mav_msg):
        self.pending.append((sender, mav_msg))

    def drain(self):
        while self.pending:
            sender, mav_msg = self.pending.popleft()
            if self.drop(sender.source_system, mav_msg):
                continue
            mav_msg._header.srcSystem = sender.source_system
            for network in self.networks:
                target_system = getattr(mav_msg, "target_system", 0)
                if target_system not in (0, network._vehicle.source_system):
                    continue
                network._on_mavlink(mav_msg)


class _Vehicle:
    def __init__(self, node_id, location, bus):
        self.source_system = node_id
        self._location = location
        self._bus = bus
        self.ground_speed = 20.0
        self.wind = SimpleNamespace(speed=0.0, direction=0.0)
        self.battery_level = 100.0
        self._callbacks = {}

    def on_message(self, name, callback):
        self._callbacks[name] = callback

    def send_mavlink_message(self, mav_msg):
        self._bus.enqueue(self, mav_msg)

    def location(self, _absolute=False):
        return self._location


def _poi(task_id, location):
    return make_detected_poi(
        task_id=task_id,
        obj_id=task_id - 1,
        p_t_g_l=location,
        class_id=0,
    )


def test_owner_dispatches_exact_second_and_third_pois_to_distinct_peers():
    """Exercise NetworkMavlink -> TaskActor using live GCS vehicle sysids."""
    poi_2 = Location(40.2967648, 44.4332133, 1339.7, is_absolute=True)
    poi_3 = Location(40.2974053, 44.4329561, 1339.7, is_absolute=True)
    bus = _MavlinkBus()
    vehicles = [
        _Vehicle(1, Location(40.2961244, 44.4334705, 1339.7), bus),
        _Vehicle(2, poi_2, bus),
        _Vehicle(3, poi_3, bus),
    ]
    actors = []
    networks = []

    for vehicle in vehicles:
        logger = MagicMock()
        # Swarm identity is the plain vehicle sysid: the companion shares
        # it with its autopilot (component 191 tells them apart), and
        # production keys both the network node id and TaskActor.id off it.
        network = NetworkMavlink(vehicle.source_system, vehicle, logger)
        actor = TaskActor(vehicle, network, logger)
        actor.start()
        network.set_listener(actor)
        bus.networks.append(network)
        networks.append(network)
        actors.append(actor)

    peer_2_id = vehicles[1].source_system
    peer_3_id = vehicles[2].source_system

    try:
        for actor in actors:
            actor._presence.heartbeat()
        bus.drain()
        # Peers are known by their sysids, and the owner must not see itself.
        assert actors[0]._rebroadcast.known_peers() == {peer_2_id, peer_3_id}

        # Keep production's asynchronous auction deterministic in-memory: all
        # response packets are drained before running one global selection.
        with (
            patch.object(TaskDispatch, "start_rebroadcast", return_value=None),
            patch.object(TaskDispatch, "start_peer_select_timer", return_value=None),
        ):
            actors[0].notify_pois([
                _poi(2, poi_2),
                _poi(3, poi_3),
            ])
            bus.drain()
            actors[0]._auction._select_peer_for_task(
                2,
                actors[0]._auction_state.current_generation(),
            )
            bus.drain()

        peer_tasks = {
            actor.id: actor.selected_poi()
            for actor in actors[1:]
        }
        # Task ids stay the mission's 2/3, independent of the node identity.
        assert {task.task_id for task in peer_tasks.values()} == {2, 3}
        assert peer_tasks[peer_2_id].task_id == 2
        assert peer_tasks[peer_3_id].task_id == 3
        assert (
            peer_tasks[peer_2_id].location.lat,
            peer_tasks[peer_2_id].location.lng,
            peer_tasks[peer_2_id].location.alt,
        ) == pytest.approx((poi_2.lat, poi_2.lng, poi_2.alt))
        assert (
            peer_tasks[peer_3_id].location.lat,
            peer_tasks[peer_3_id].location.lng,
            peer_tasks[peer_3_id].location.alt,
        ) == pytest.approx((poi_3.lat, poi_3.lng, poi_3.alt))

        dispatch_2 = actors[0]._auction_state.lookup(2)
        dispatch_3 = actors[0]._auction_state.lookup(3)
        assert dispatch_2.status == TaskDispatchStatus.CONFIRMED
        assert dispatch_3.status == TaskDispatchStatus.CONFIRMED
        assert {dispatch_2.assigned_peer, dispatch_3.assigned_peer} == {
            peer_2_id, peer_3_id,
        }
    finally:
        for actor in actors:
            actor.reset()


def _three_uav_swarm(bus, owner_loc, peer_2_loc, peer_3_loc):
    vehicles = [
        _Vehicle(1, owner_loc, bus),
        _Vehicle(2, peer_2_loc, bus),
        _Vehicle(3, peer_3_loc, bus),
    ]
    actors = []
    for vehicle in vehicles:
        logger = MagicMock()
        network = NetworkMavlink(vehicle.source_system, vehicle, logger)
        actor = TaskActor(vehicle, network, logger)
        actor.start()
        network.set_listener(actor)
        bus.networks.append(network)
        actor._presence.heartbeat()
        actors.append(actor)
    bus.drain()
    return vehicles, actors


def _is_assign_response_from(sender_id, mav_msg, peer_id):
    return (
        sender_id == peer_id
        and mav_msg.get_msgId() == MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE
    )


def _holders(actors, task_id):
    """Peers nav would fly the task on (ASSIGNED)."""
    return {
        actor.id
        for actor in actors[1:]
        if (selected := actor.selected_poi()) is not None
        and selected.task_id == task_id
    }


def _waiting(actors, task_id):
    """Peers holding the task offer, invisible to nav until applied."""
    return {
        actor.id
        for actor in actors[1:]
        if actor._selection.state() is SlotState.WAITING
        and actor._selection.held().task.task_id == task_id
    }


def test_lost_assign_response_is_recovered_by_the_next_response_copy():
    owner_loc = Location(40.2961244, 44.4334705, 1339.7)
    poi = Location(40.2967648, 44.4332133, 1339.7, is_absolute=True)
    bus = _MavlinkBus()
    timers = FakeTimers()
    _, actors = _three_uav_swarm(
        bus, owner_loc, poi, Location(40.2974053, 44.4329561, 1339.7),
    )
    dropped = []

    def drop_first_response_from_2(sender_id, mav_msg):
        if not dropped and _is_assign_response_from(sender_id, mav_msg, 2):
            dropped.append(mav_msg)
            return True
        return False

    bus.drop = drop_first_response_from_2
    try:
        with (
            timers.patch(),
            patch.object(TaskDispatch, "start_rebroadcast", return_value=None),
            patch.object(TaskDispatch, "start_peer_select_timer", return_value=None),
        ):
            actors[0].notify_pois([_poi(2, poi)])
            bus.drain()
            dispatch = actors[0]._auction_state.lookup(2)
            assert len(dropped) == 1
            assert dispatch.status == TaskDispatchStatus.CONFIRMING
            # Peer 2 waits for the owner's confirmation and does not fly.
            assert _waiting(actors, 2) == {2}
            assert _holders(actors, 2) == set()

            # The owner's acked request slot sends nothing; the peer's
            # next response copy confirms.
            timers.fire_pending()
            bus.drain()

        assert dispatch.status == TaskDispatchStatus.CONFIRMED
        assert dispatch.assigned_peer == 2
        assert _holders(actors, 2) == {2}
        assert timers.fire_pending() == 0
    finally:
        for actor in actors:
            actor.reset()


def test_peer_stranded_by_lost_responses_releases_task_before_reassignment():
    """Peer 2 accepted but every reply was lost; the task then goes to 3."""
    owner_loc = Location(40.2961244, 44.4334705, 1339.7)
    poi = Location(40.2967648, 44.4332133, 1339.7, is_absolute=True)
    bus = _MavlinkBus()
    timers = FakeTimers()
    vehicles, actors = _three_uav_swarm(
        bus, owner_loc, poi, Location(40.2974053, 44.4329561, 1339.7),
    )
    bus.drop = lambda sender_id, mav_msg: _is_assign_response_from(
        sender_id, mav_msg, 2,
    )
    try:
        with (
            timers.patch(),
            patch.object(TaskDispatch, "start_peer_select_timer", return_value=None),
        ):
            actors[0].notify_pois([_poi(2, poi)])
            bus.drain()
            dispatch = actors[0]._auction_state.lookup(2)
            assert dispatch.assigned_peer == 2
            assert _waiting(actors, 2) == {2}
            assert _holders(actors, 2) == set()

            # Peer 2 is now farther away; its replies stay lost throughout.
            vehicles[1]._location = Location(40.3261244, 44.4634705, 1339.7)
            for _ in range(20):
                if dispatch.status == TaskDispatchStatus.CONFIRMED:
                    break
                # The task is never held by two peers at once.
                assert len(_holders(actors, 2)) <= 1
                assert timers.fire_pending()
                bus.drain()

        assert dispatch.status == TaskDispatchStatus.CONFIRMED
        assert dispatch.assigned_peer == 3
        assert _holders(actors, 2) == {3}
    finally:
        for actor in actors:
            actor.reset()
