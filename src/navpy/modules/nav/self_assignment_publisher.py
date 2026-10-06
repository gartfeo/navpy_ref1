"""Network publication of this UAV's selected target."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_assignment_msg import TaskAssignRequestMsg
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import class_to_task_type
from navpy.modules.nav.confirmation_ports import ConfirmationMessageBroadcaster
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id


class SelfAssignmentPublisher:
    """Publish this UAV's selected target independently of review policy."""

    def __init__(
        self,
        *,
        sys_id: int,
        is_simulation: bool,
        network: Callable[[], Optional[ConfirmationMessageBroadcaster]],
        logger: ILogger,
    ) -> None:
        self._sys_id = sys_id
        self._is_simulation = is_simulation
        self._network = network
        self._logger = logger

    def publish(self, target: DetectedObject) -> None:
        target_id = get_target_task_id(target)
        network = self._network()
        if target_id is None or network is None:
            return
        source = target.geo.projected_target_location
        if self._is_simulation and target.geo.truth_target_location is not None:
            source = target.geo.truth_target_location
        location = (
            LocationMsgData(source.lat, source.lng, source.alt)
            if source is not None
            else LocationMsgData(0.0, 0.0, 0.0)
        )
        class_id = target.classification.class_id
        task = TaskAssignMsgData(
            task_id=target_id,
            task_type=class_to_task_type(class_id),
            location=location,
            class_id=class_id,
        )
        message = TaskAssignRequestMsg(
            sender_id=self._sys_id,
            receiver_id=self._sys_id,
            task=task,
        )
        try:
            network.broadcast(message)
        except OSError as exc:
            self._logger.error(
                f"Failed to broadcast self-assignment for T{target_id}: {exc}",
                exc,
            )


__all__ = ["SelfAssignmentPublisher"]
