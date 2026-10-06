from __future__ import annotations

import threading
import time
from typing import Callable, List, Optional

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.nav.confirmation_image_artifacts import (
    save_confirmation_image_artifacts,
)
from navpy.modules.nav.confirmation_dependencies import (
    ConfirmationFailurePolicy,
    ConfirmationNetworkSlot,
    FreshnessGate,
)
from navpy.modules.nav.confirmation_round_runner import ConfirmationRoundRunner
from navpy.modules.nav.self_assignment_publisher import SelfAssignmentPublisher
from navpy.modules.nav.confirmation_coordinator import (
    ConfirmationCoordinator,
)
from navpy.modules.nav.confirmation_inbound import (
    ConfirmationInboundHandler,
)
from navpy.modules.nav.confirmation_media import (
    ConfirmationMedia,
)
from navpy.modules.nav.confirmation_manager_state import ConfirmationManagerState, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.utils.image_utils import create_confirmation_thumbnail


RESEND_INTERVAL_S = 2.0


class ConfirmationManager(ListenerAbc):
    """Thin confirmation façade composed from focused state and services."""

    def __init__(
        self,
        sys_id: int,
        args: NavArgs,
        logger: ILogger,
        *,
        is_simulation: bool = False,
        freshness_check: Optional[Callable[[Optional[int]], bool]] = None,
    ) -> None:
        self._sys_id = sys_id
        self._args = args
        self._logger = logger
        self._state = ConfirmationManagerState()
        self._network_slot = ConfirmationNetworkSlot()
        self._freshness = FreshnessGate(logger)
        if freshness_check is not None:
            self._freshness.set_check(freshness_check)
        self._failure_policy = ConfirmationFailurePolicy(
            lambda: bool(getattr(args, "is_confirm_on_fail", False)),
            self._freshness,
        )
        self._media = ConfirmationMedia(
            sys_id=sys_id,
            network=self._network_slot.get,
            logger=logger,
            thumbnail_builder=lambda **kwargs: create_confirmation_thumbnail(
                **kwargs
            ),
            artifact_saver=lambda **kwargs: save_confirmation_image_artifacts(
                **kwargs
            ),
        )
        self._assignment = SelfAssignmentPublisher(
            sys_id=sys_id,
            is_simulation=is_simulation,
            network=self._network_slot.get,
            logger=logger,
        )
        self._round_runner = ConfirmationRoundRunner(
            sys_id=sys_id,
            state=self._state,
            network=self._network_slot.get,
            wait_time_s=lambda: args.confirm_wait_time_sec,
            resend_interval_s=lambda: RESEND_INTERVAL_S,
            failure_policy=self._failure_policy,
            media=self._media,
            logger=logger,
            monotonic=lambda: time.monotonic(),
            event_factory=lambda: threading.Event(),
        )
        self._coordinator = ConfirmationCoordinator(
            sys_id=sys_id,
            state=self._state,
            network=self._network_slot.get,
            auto_confirm=lambda: bool(getattr(args, "is_auto_confirm", False)),
            assignment=self._assignment,
            round_runner=self._round_runner,
            logger=logger,
            event_factory=lambda: threading.Event(),
            thread_factory=lambda **kwargs: threading.Thread(**kwargs),
        )
        self._inbound = ConfirmationInboundHandler(
            sys_id=sys_id,
            state=self._state,
            media=self._media,
            logger=logger,
        )

    def set_network(self, network: Optional[NetworkAbc]) -> None:
        self._network_slot.set(network)

    def set_poi_freshness_check(
        self,
        check: Callable[[Optional[int]], bool],
    ) -> None:
        self._freshness.set_check(check)

    def reset(self) -> None:
        self._state.reset()

    @property
    def active_poi(self) -> Optional[DetectedObject]:
        return self._state.registry.active_poi

    def set_active_poi(self, poi: Optional[DetectedObject]) -> None:
        self._state.registry.set_active_poi(poi)

    def clear_active_poi(self) -> None:
        self._state.registry.set_active_poi(None)

    def update_status(self, poi: DetectedObject, status: ConfirmationStatus) -> None:
        self._state.registry.update_status(poi, status)

    def get_status(self, poi: DetectedObject) -> Optional[ConfirmationStatus]:
        return self._state.registry.get_status(poi)

    def clear_status(self, poi: DetectedObject) -> None:
        self._state.registry.clear_status(poi)

    def review(self, pois: List[DetectedObject]) -> None:
        self._coordinator.review(pois)

    def on_message(self, msg: MsgABC) -> None:
        self._inbound.on_message(msg)

    def get_status_on_fail(
        self,
        poi_id: Optional[int] = None,
    ) -> ConfirmationStatus:
        return self._failure_policy.status(poi_id)
