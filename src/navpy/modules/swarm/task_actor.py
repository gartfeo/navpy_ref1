"""Thin application boundary for swarm task participation."""

from __future__ import annotations

import threading
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.swarm.swarm_presence import SwarmPresence
from navpy.modules.swarm.task_ack_routing import TaskAckRouter
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
    ) -> None:
        self.id: int = vehicle.source_system
        self.vehicle = vehicle
        self.network = network
        self.logger = logger
        lock = threading.RLock()
        selection, auction_state, rebroadcast_state, confirmation = (
            create_task_state(lock)
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
            selection,
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
            rebroadcast.peer_discovered,
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
                MsgType.SWARM_ACK: TaskAckRouter({}, logger).route,
            },
            logger,
        )

        self._selection = selection
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
        return self._selection.selected()

    def has_selected_pois(self) -> bool:
        return self.selected_poi() is not None

    def clear_selected_poi(self) -> None:
        self._selection.clear()

    def reset(self) -> None:
        self._presence.stop()
        self._auction_state.reset(self.logger)
        self._selection.clear()
        self._clock_reset.reset()

    def shutdown(self) -> None:
        try:
            self._presence.stop()
            self.checkout()
        finally:
            self._auction_state.shutdown(self.logger)
