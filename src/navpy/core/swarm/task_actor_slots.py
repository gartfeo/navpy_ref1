"""Thread-safe slot for the one peer task this UAV holds (helper side)."""

from __future__ import annotations

import itertools
from collections.abc import Callable, Collection
from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.link.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
)
from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.link.task_message_data import TaskAssignMsgData
from navpy.core.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.core.swarm.task_msg_refs import MsgRef, OwnerRef, RecentRefs


class SlotState(Enum):
    EMPTY = "empty"
    # Offered by a step 3; invisible to nav until the owner applies step 4.
    WAITING = "waiting"
    # The owner applied this UAV's step 4: nav may fly the task.
    ASSIGNED = "assigned"


@dataclass(frozen=True)
class HeldTask:
    """A peer task held by this UAV, as its owner's step 3 offered it."""

    task: TaskAssignMsgData
    request: MsgRef  # newest accepted step-3 copy
    token: int  # identifies one WAITING; binds its step-4 repeat

    @property
    def owner(self) -> OwnerRef:
        return self.request.owner

    @property
    def key(self) -> tuple[int, int]:
        """(owner, task): what a reject repeat answers."""
        return (self.request.sender_id, self.task.task_id)


class RequestKind(Enum):
    WAIT = "wait"  # entered WAITING: start the step-4 repeat
    REPEAT = "repeat"  # copy of the held offer: one more step 4
    REJECT = "reject"  # reject repeat for (owner, task)


@dataclass(frozen=True)
class RequestDecision:
    kind: RequestKind
    held: Optional[HeldTask] = None
    reason: str = ""


class AckKind(Enum):
    NONE = "none"
    PROGRESS = "progress"  # PROCESSING from the offer's owner boot
    ASSIGNED = "assigned"
    DROPPED = "dropped"


class SelectedTaskSlot:
    """Own the single peer task this UAV holds: EMPTY, WAITING or ASSIGNED.

    Only an ASSIGNED task is visible to nav. Transitions follow the helper
    slot table in docs/design/swarm-task-assignment-ack.md.
    """

    def __init__(
        self,
        lock: rt.RLock,
        monotonic_s: Callable[[], float] = rt.monotonic,
        reply_retention_s: float = ASSIGN_ACK_TIMING.response.retention_s,
    ) -> None:
        self._lock = lock
        self._new_replies = lambda: RecentRefs(reply_retention_s, monotonic_s)
        self._state = SlotState.EMPTY
        self._held: Optional[HeldTask] = None
        self._replies = self._new_replies()
        self._approaching = False
        self._docked = False
        # Nav's display bits for the heartbeat byte (nav_display.py); read
        # with node_state() under the shared actor lock.
        self.display_bits = 0
        self._tokens = itertools.count(1)

    def on_request(
        self,
        task: TaskAssignMsgData,
        request: MsgRef,
        flyable: bool,
    ) -> RequestDecision:
        """Decide on one step-3 copy that carries a UID."""
        with self._lock:
            held = self._held
            same_offer = (
                held is not None
                and held.owner == request.owner
                and held.task.task_id == task.task_id
            )
            if self._state is SlotState.ASSIGNED:
                if same_offer:
                    return RequestDecision(RequestKind.REPEAT, held)
                return RequestDecision(
                    RequestKind.REJECT, reason="holding another task",
                )
            if self._state is SlotState.WAITING:
                if same_offer:
                    if request.follows(held.request):
                        held = self._held = replace(held, request=request)
                    return RequestDecision(RequestKind.REPEAT, held)
                if (
                    held.owner.owner_id != request.sender_id
                    or held.owner.boot_id == request.boot_id
                ):
                    return RequestDecision(
                        RequestKind.REJECT, reason="waiting for another task",
                    )
                self._clear()  # the owner restarted: its old offer is gone
            if self._docked:
                return RequestDecision(RequestKind.REJECT, reason="docked")
            if self._approaching:
                return RequestDecision(
                    RequestKind.REJECT, reason="flying a final approach",
                )
            if not flyable:
                return RequestDecision(RequestKind.REJECT, reason="cannot fly it")
            self._held = HeldTask(task, request, next(self._tokens))
            self._state = SlotState.WAITING
            return RequestDecision(RequestKind.WAIT, self._held)

    def record_reply(self, token: int, reply: MsgRef) -> bool:
        """Remember a step-4 copy sent for the WAITING task ``token``."""
        with self._lock:
            if self.waiting(token) is None:
                return False
            self._replies.add(reply)
            return True

    def on_ack(
        self,
        acked: MsgRef,
        status: int,
        ack: MsgRef,
    ) -> tuple[AckKind, Optional[HeldTask]]:
        """Apply the owner's ack of one of the WAITING task's step-4 copies."""
        with self._lock:
            held = self._held
            if self._state is not SlotState.WAITING or acked not in self._replies:
                return AckKind.NONE, None
            if status == ACK_STATUS_APPLIED and ack.owner == held.owner:
                self._state = SlotState.ASSIGNED
                self._replies = self._new_replies()
                return AckKind.ASSIGNED, held
            if status == ACK_STATUS_PROCESSING and ack.owner == held.owner:
                return AckKind.PROGRESS, held
            if (
                status == ACK_STATUS_RECEIVED
                and ack.sender_id == held.owner.owner_id
            ):
                self._clear()
                return AckKind.DROPPED, held
            return AckKind.NONE, None

    def on_advert(
        self,
        advert: Optional[MsgRef],
        task_ids: Collection[int],
    ) -> Optional[HeldTask]:
        """Drop WAITING when its owner advertises the task again, later."""
        with self._lock:
            held = self._held
            if (
                self._state is not SlotState.WAITING
                or advert is None
                or held.task.task_id not in task_ids
                or not advert.follows(held.request)
            ):
                return None
            self._clear()
            return held

    def set_participation(
        self,
        approaching: bool,
        docked: bool,
        display_bits: int = 0,
    ) -> Optional[HeldTask]:
        """Track nav's approach and dock; becoming busy drops a WAITING task.

        Both flags and the display bits change in one update, so NAV ->
        DOCKED is never FREE and a beat never mixes two nav states.
        """
        with self._lock:
            started = (approaching or docked) and not self._busy()
            self._approaching = approaching
            self._docked = docked
            self.display_bits = display_bits
            if not started or self._state is not SlotState.WAITING:
                return None
            held = self._held
            self._clear()
            return held

    def expire(self, token: int) -> Optional[HeldTask]:
        with self._lock:
            held = self.waiting(token)
            if held is not None:
                self._clear()
            return held

    def release_assigned(self) -> Optional[HeldTask]:
        """Nav ended the task; a WAITING offer is kept."""
        with self._lock:
            if self._state is not SlotState.ASSIGNED:
                return None
            held = self._held
            self._clear()
            return held

    def waiting(self, token: int) -> Optional[HeldTask]:
        with self._lock:
            held = self._held
            if self._state is SlotState.WAITING and held.token == token:
                return held
            return None

    def held(self) -> Optional[HeldTask]:
        """The WAITING or ASSIGNED task, for diagnostics."""
        with self._lock:
            return self._held

    def selected(self) -> Optional[TaskAssignMsgData]:
        with self._lock:
            if self._state is not SlotState.ASSIGNED:
                return None
            return self._held.task

    def state(self) -> SlotState:
        with self._lock:
            return self._state

    def node_state(self) -> SwarmNodeState:
        """BUSY while holding a task or flying a final approach."""
        with self._lock:
            if self._state is not SlotState.EMPTY or self._busy():
                return SwarmNodeState.BUSY
            return SwarmNodeState.FREE

    def _busy(self) -> bool:
        return self._approaching or self._docked

    def _clear(self) -> None:
        self._state = SlotState.EMPTY
        self._held = None
        self._replies = self._new_replies()


__all__ = [
    "AckKind",
    "HeldTask",
    "RequestDecision",
    "RequestKind",
    "SelectedTaskSlot",
    "SlotState",
]
