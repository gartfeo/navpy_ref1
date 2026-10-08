"""Network-bound navigation session and bounded NAV peer submission."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.location import Location
from navpy.modules.nav.confirm_override_listener import ConfirmOverrideListener
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.failure_health import ComponentFailureLatch
from navpy.modules.nav.nav_network_shutdown import (
    disconnect_nav_network_session,
)
from navpy.modules.nav.poi_selection import PoiSelector
from navpy.modules.nav.nav_state import ConfirmOverrideInbox
from navpy.modules.nav.peer_poi_dispatch import (
    PeerPoiDispatchPorts,
    PeerPoiDispatchWorker,
)
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_ports import MessageClockReset
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detector_ports import SimulationControlPort
from navpy.modules.vision.models.detect_data import DetectedObject


class NavNetworkRuntime:
    """Own TaskActor, confirmation listener, and peer worker lifetimes."""

    def __init__(
        self,
        vehicle: IVehicle,
        simulation: SimulationControlPort,
        ground_location: Callable[..., Optional[Location]],
        confirmation_manager: ConfirmationManager,
        overrides: ConfirmOverrideInbox,
        logger: ILogger,
    ) -> None:
        self._vehicle = vehicle
        self._simulation = simulation
        self._ground_location = ground_location
        self._confirmation_manager = confirmation_manager
        self._overrides = overrides
        self._logger = logger
        self._network: Optional[NetworkAbc] = None
        self._override_listener: Optional[ConfirmOverrideListener] = None
        self.task_actor: Optional[TaskActor] = None
        self.peer_dispatch: Optional[PeerPoiDispatchWorker] = None
        self._health = ComponentFailureLatch()
        self._published: tuple[Optional[TaskActor], bool] = (None, False)

    def set_network(self, network: Optional[NetworkAbc]) -> None:
        self._disconnect()
        if network is None:
            return
        actor = TaskActor(
            self._vehicle,
            network,
            self._logger,
            clock_reset=MessageClockReset.from_network(network),
        )
        override_listener = ConfirmOverrideListener(
            self._vehicle.source_system,
            self._overrides.request,
            self._logger,
        )
        self._network = network
        self.task_actor = actor
        self._override_listener = override_listener
        network.set_listener(actor)
        self._confirmation_manager.set_network(network)
        network.set_listener(self._confirmation_manager)
        network.set_listener(override_listener)
        worker = PeerPoiDispatchWorker(
            PeerPoiDispatchPorts(
                active_poi=lambda: self._confirmation_manager.active_poi,
                resolve_location=self._resolve_peer_location,
                status_exists=lambda poi: (
                    self._confirmation_manager.get_status(poi) is not None
                ),
                mark_notified=lambda poi: self._confirmation_manager.update_status(
                    poi,
                    ConfirmationStatus.PEER_NOTIFIED,
                ),
                notify_pois=actor.notify_pois,
                warn=lambda message: self._logger.warning(message, key="nav"),
                error=lambda message, error: self._logger.error(
                    f"{message}: {error}",
                    error,
                ),
            )
        )
        self.peer_dispatch = worker
        worker.start()

    def has_task_actor(self) -> bool:
        return self.task_actor is not None

    def selected_poi(self) -> Optional[TaskAssignMsgData]:
        if self.task_actor is None:
            return None
        return self.task_actor.selected_poi()

    def notify_pois(self, pois: list[DetectedObject]) -> None:
        if self.task_actor is not None:
            self.task_actor.notify_pois(pois)

    def submit_nav_peers(self, pois: list[DetectedObject]) -> None:
        if self.peer_dispatch is not None and pois:
            self.peer_dispatch.submit(pois)

    def publish_approaching(self, approaching: bool) -> None:
        """Tell the swarm whether nav flies a final approach (BUSY).

        Called every nav cycle; only a change, or a new actor, takes the
        actor's lock.
        """
        actor = self.task_actor
        if actor is None or self._published == (actor, approaching):
            return
        self._published = (actor, approaching)
        actor.set_approaching(approaching)

    def start_task_actor(self) -> None:
        if self.task_actor is not None:
            self.task_actor.start()

    def reset_task_actor(self) -> None:
        if self.task_actor is not None:
            self.task_actor.reset()

    def clear_selected_poi(self) -> None:
        if self.task_actor is not None:
            self.task_actor.clear_selected_poi()

    def reset_peer_dispatch(self) -> None:
        if self.peer_dispatch is not None:
            self.peer_dispatch.reset()

    def stop(self) -> None:
        self._disconnect()

    def raise_if_failed(self) -> None:
        self._health.raise_if_failed((self.peer_dispatch, self.task_actor))

    def _disconnect(self) -> None:
        disconnect_nav_network_session(
            self.peer_dispatch,
            self.task_actor,
            self._network,
            self._override_listener,
            self._confirmation_manager,
            self._health,
        )
        self.peer_dispatch = None
        self.task_actor = None
        self._network = None
        self._override_listener = None

    def _resolve_peer_location(
        self,
        poi: DetectedObject,
    ) -> Optional[Location]:
        if (
            self._simulation.is_simulation
            and poi.geo.truth_poi_location is not None
        ):
            return poi.geo.truth_poi_location
        return self._ground_location(poi, allow_fallback=False)


class NavPeerSubmission:
    """Submit at most the worker-bound peer snapshot from one NAV tick."""

    def __init__(
        self,
        selector: PoiSelector,
        detections: DetectionSnapshot,
        submit: Callable[[list[DetectedObject]], None],
    ) -> None:
        self._selector = selector
        self._detections = detections
        self._submit = submit

    def submit(self) -> None:
        selection = self._detections.selection()
        _, peers = self._selector.select(
            list(selection.pois),
            selection.primary_poi,
        )
        if peers:
            # The live-demo fleet has exactly three UAVs: one owner and at
            # most two peers.  Keep NAV limited to one bounded worker submit.
            self._submit(peers[:2])


__all__ = ["NavPeerSubmission", "NavNetworkRuntime"]
