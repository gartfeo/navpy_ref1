"""In-memory MAVLink regression for the three-UAV task auction."""

from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.comm.network_mavlink import NetworkMavlink
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_dispatch import TaskDispatch
from tests.detection_factory import make_detected_poi


class _MavlinkBus:
    def __init__(self):
        self.networks = []
        self.pending = deque()

    def enqueue(self, sender, mav_msg):
        self.pending.append((sender, mav_msg))

    def drain(self):
        while self.pending:
            sender, mav_msg = self.pending.popleft()
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
        for actor, network in zip(actors, networks):
            network.broadcast(SwarmHeartbeatMsg(actor.id))
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
