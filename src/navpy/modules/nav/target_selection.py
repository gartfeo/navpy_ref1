"""Selection of one local target and any peer candidates."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.modules.nav.confirmation_policy import TargetRetryPolicy
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id
from navpy.modules.vision.target_priority import (
    find_target_by_task_id,
    prioritize_targets,
)


class TargetSelector:
    """Split detections into one local candidate and peer candidates."""

    def __init__(
        self,
        confirmation_manager: ConfirmationManager,
        retry: TargetRetryPolicy,
        peer_available: Callable[[], bool],
    ) -> None:
        self._confirmation_manager = confirmation_manager
        self._retry = retry
        self._peer_available = peer_available

    def select(
        self,
        available_targets: list[DetectedObject],
        primary_target: Optional[DetectedObject] = None,
    ) -> tuple[Optional[DetectedObject], list[DetectedObject]]:
        if not available_targets:
            return None, []
        active = self._confirmation_manager.active_target
        if active is not None:
            own = find_target_by_task_id(
                available_targets,
                get_target_task_id(active),
            )
            peers = [
                target
                for target in available_targets
                if self._peer_available() and target is not own
            ]
            return own, peers
        own = None
        peers: list[DetectedObject] = []
        permanent = {
            ConfirmationStatus.PEER_NOTIFIED,
            ConfirmationStatus.REJECTED,
            ConfirmationStatus.CONFIRMED,
        }
        for target in prioritize_targets(available_targets, primary_target):
            status = self._confirmation_manager.get_status(target)
            exhausted = (
                status is ConfirmationStatus.TIMEOUT_REJECTED
                and not self._retry.can_reask(target)
            )
            if status in permanent or exhausted:
                if self._peer_available():
                    peers.append(target)
                continue
            if self._retry.is_in_target_cooldown(target):
                continue
            if own is None:
                own = target
            elif self._peer_available():
                peers.append(target)
        return own, peers


__all__ = ["TargetSelector"]
