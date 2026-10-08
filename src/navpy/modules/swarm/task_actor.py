"""Thin application boundary for swarm task participation."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.swarm.swarm_presence import PeerStatusPorts, SwarmPresence
from navpy.modules.swarm.task_ack_routing import TaskAckRouter
from navpy.modules.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.modules.swarm.task_assign_reply import AssignReplies
from navpy.modules.swarm.task_state_composition import create_task_state
from navpy.modules.swarm.task_assignment_planner import (
    MinimumEtaAssignmentPlanner,
)
from navpy.modules.swarm.task_auction_coordinator import TaskAuctionCoordinator
from navpy.modules.swarm.task_capability import (
    TaskCapabilityEvaluator,
    TaskParticipationCoordinator,
)
from navpy.modules.swarm.task_messaging import (
    TaskMessageRouter,
    TaskMessageSender,
)
from navpy.modules.swarm.task_peer_status import PeerStatusCoordinator
from navpy.modules.swarm.task_ports import (
    ClockOffsetResetPort,
    MessageClockReset,
)
from navpy.modules.swarm.task_rebroadcast import TaskRebroadcastCoordinator
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.utils.fly_estimator import FlyEstimator


class TaskActor(ListenerAbc):
    """Compose the task services used by NavController and the network."""

    def __init__(
        self,
        vehicle: IVehicle,
        network: NetworkAbc,
        logger: ILogger,
        clock_reset: ClockOffsetResetPort | None = None,
        monotonic_s: Callable[[], float] = time.monotonic,
    ) -> None:
        self.id: int = vehicle.source_system
        self.vehicle = vehicle
        self.network = network
        self.logger = logger
        lock = threading.RLock()
        selection, auction_state, rebroadcast_state, confirmation = (
            create_task_state(lock, monotonic_s)
        )
        sender = TaskMessageSender(self.id, network, logger)
        rebroadcast = TaskRebroadcastCoordinator(
            auction_state,
            rebroadcast_state,
            sender,
            logger,
        )
        auction = TaskAuctionCoordinator(
            auction_state,
            confirmation,
            MinimumEtaAssignmentPlanner(),
            sender,
            rebroadcast,
            logger,
        )
        participation = TaskParticipationCoordinator(
            self.id,
            lock,
            selection,
            AssignReplies(
                lock, selection, sender, ASSIGN_ACK_TIMING.response, logger,
            ),
            TaskCapabilityEvaluator(
                vehicle,
                lambda location: FlyEstimator.time_to_fly(vehicle, location),
            ),
            sender,
            logger,
        )
        presence = SwarmPresence(
            self.id,
            vehicle,
            sender,
            lock,
            selection.node_state,
            _peer_status_ports(
                rebroadcast,
                PeerStatusCoordinator(auction_state, rebroadcast, auction.replan),
            ),
            logger,
        )
        acks = TaskAckRouter(
            {
                MsgType.TASK_ASSIGN_REQUEST: auction.on_request_ack,
                MsgType.TASK_ASSIGN_RESPONSE: participation.on_response_ack,
            },
            logger,
        )
        router = TaskMessageRouter(
            self.id,
            presence.is_started,
            auction_state.admits_peer,
            {
                MsgType.SWARM_HEARTBEAT: presence.on_heartbeat,
                MsgType.AVAILABLE_TASK_REQUEST: participation.on_available_request,
                MsgType.AVAILABLE_TASK_RESPONSE: auction.on_available_response,
                MsgType.TASK_ASSIGN_REQUEST: participation.on_assign_request,
                MsgType.TASK_ASSIGN_RESPONSE: auction.on_assign_response,
                MsgType.CHECK_IN: presence.on_checkin,
                MsgType.CHECK_OUT: presence.on_checkout,
                MsgType.SWARM_ACK: acks.route,
            },
            logger,
        )

        self._selection = selection
        self._participation = participation
        self._auction_state = auction_state
        self._rebroadcast = rebroadcast
        self._auction = auction
        self._presence = presence
        self._router = router
        self._clock_reset = clock_reset or MessageClockReset.from_network(network)
        # A replacement actor must not inherit timing estimates from the prior
        # network session before NavNetworkRuntime performs its first checkin.
        self._clock_reset.reset()

    def start(self) -> None:
        self._presence.start()

    def raise_if_failed(self) -> None:
        self._presence.raise_if_failed()

    def on_message(self, message: MsgABC) -> None:
        self._router.route(message)

    def notify_pois(self, pois: list[DetectedObject]) -> None:
        self._rebroadcast.notify_pois(pois)

    def checkin(self) -> None:
        self._presence.checkin()

    def checkout(self) -> None:
        self._presence.checkout()

    def selected_poi(self) -> Optional[TaskAssignMsgData]:
        """The peer task nav may fly: only one the owner applied (ASSIGNED)."""
        return self._selection.selected()

    def has_selected_pois(self) -> bool:
        return self.selected_poi() is not None

    def set_approaching(self, approaching: bool) -> None:
        """Nav publishes whether it flies a final approach (BUSY)."""
        self._participation.set_approaching(approaching)

    def clear_selected_poi(self) -> None:
        """Nav ended its task; a WAITING offer is kept."""
        self._selection.release_assigned()

    def reset(self) -> None:
        self._presence.stop()
        self._auction_state.reset(self.logger)
        self._selection.release_assigned()
        self._clock_reset.reset()

    def shutdown(self) -> None:
        try:
            self._presence.stop()
            self.checkout()
        finally:
            self._participation.close()
            self._auction_state.shutdown(self.logger)


def _peer_status_ports(
    rebroadcast: TaskRebroadcastCoordinator,
    status: PeerStatusCoordinator,
) -> PeerStatusPorts:
    return PeerStatusPorts(
        discovered=rebroadcast.peer_discovered,
        reported=status.observe_peer,
        checked_out=status.peer_checked_out,
        tick=status.check_silence,
    )
