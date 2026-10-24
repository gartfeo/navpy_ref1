"""Helper step-4 repeats: answer an owner until it acknowledges.

The accept repeat answers the WAITING task and is bound to its slot token;
reject repeats answer other offers, keyed by (owner, task). Each repeats
every interval and ends after a silence without owner progress (a
PROCESSING ack of one of its copies). Each scheduled accept copy follows a
fresh ack of the held step-3 copy, so an owner whose acks were all lost
still gets its floor. Both share the actor lock with the slot, so every
copy is stamped in order with the slot's transitions and the node's state
reports.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
)
from navpy.core.link.types import MsgType
from navpy.core.swarm.task_ack_timing import HandshakeTiming
from navpy.core.swarm.task_actor_slots import HeldTask, SelectedTaskSlot
from navpy.core.swarm.task_messaging import TaskMessageSender
from navpy.core.swarm.task_msg_refs import MsgRef, RecentRefs


@dataclass(eq=False)
class _Repeat:
    refs: RecentRefs
    progress_s: float  # when the owner last made progress
    timer: Optional[rt.Timer] = None

    @classmethod
    def begin(cls, timing: HandshakeTiming, monotonic_s: Callable[[], float]) -> _Repeat:
        return cls(RecentRefs(timing.retention_s, monotonic_s), monotonic_s())

    def cancel(self) -> None:
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None


class AssignReplies:
    """Repeat this UAV's step-4 answers until the owner answers."""

    def __init__(
        self,
        lock: rt.RLock,
        slot: SelectedTaskSlot,
        sender: TaskMessageSender,
        timing: HandshakeTiming,
        logger: ILogger,
        monotonic_s: Callable[[], float] = rt.monotonic,
    ) -> None:
        self._lock = lock
        self._slot = slot
        self._sender = sender
        self._timing = timing
        self._monotonic_s = monotonic_s
        self._logger = logger
        self._accept: Optional[tuple[int, _Repeat]] = None
        self._rejects: dict[tuple[int, int], _Repeat] = {}
        self._closed = False

    def start_accept(self, held: HeldTask) -> None:
        """Answer the WAITING task now, then every interval until it ends.

        The first copy follows the step-3 ack just sent, so it needs no
        re-ack.
        """
        with self._lock:
            self._stop_accept()
            self._stop_reject(held.key)
            if self._closed:
                return
            repeat = _Repeat.begin(self._timing, self._monotonic_s)
            self._accept = (held.token, repeat)
            self._arm(
                repeat,
                lambda: self._accept_tick(held.token, repeat),
                self._timing.interval_s,
            )
            self._send_accept(held)

    def accept_progress(self, token: int) -> None:
        """The owner acked a copy as PROCESSING: it is still answering."""
        with self._lock:
            if self._accept is not None and self._accept[0] == token:
                self._accept[1].progress_s = self._monotonic_s()

    def answer_again(self, held: HeldTask) -> None:
        """One extra accept copy for an owner that repeated its offer."""
        with self._lock:
            if not self._closed:
                self._send_accept(held)

    def stop_accept(self) -> None:
        with self._lock:
            self._stop_accept()

    def start_reject(self, owner_id: int, task_id: int) -> None:
        """Reject an offer every interval; a repeated offer gets a copy now.

        The first copy goes out from the repeat's timer thread, so a caller
        such as the nav thread never sends.
        """
        key = (owner_id, task_id)
        with self._lock:
            if self._closed:
                return
            repeat = self._rejects.get(key)
            if repeat is not None:
                self._send_reject(key, repeat)
                return
            repeat = self._rejects[key] = _Repeat.begin(self._timing, self._monotonic_s)
            self._arm(repeat, lambda: self._reject_tick(key, repeat), 0.0)

    def on_reject_ack(self, acked: MsgRef, status: int, sender_id: int) -> bool:
        """Stop the reject repeat the ack names; True when it named one.

        RECEIVED or APPLIED ends it; PROCESSING from its owner is progress.
        """
        with self._lock:
            for key, repeat in self._rejects.items():
                if acked not in repeat.refs:
                    continue
                if status in (ACK_STATUS_RECEIVED, ACK_STATUS_APPLIED):
                    self._stop_reject(key)
                elif status == ACK_STATUS_PROCESSING and sender_id == key[0]:
                    repeat.progress_s = self._monotonic_s()
                return True
            return False

    def close(self) -> None:
        """Cancel every repeat for good (actor shutdown)."""
        with self._lock:
            self._closed = True
            self._stop_accept()
            for key in list(self._rejects):
                self._stop_reject(key)

    def _accept_tick(self, token: int, repeat: _Repeat) -> None:
        with self._lock:
            if self._closed or self._accept != (token, repeat):
                return
            held = self._slot.waiting(token)
            if held is None:
                self._accept = None
                return
            if self._monotonic_s() - repeat.progress_s < self._timing.silence_s:
                self._arm(
                    repeat,
                    lambda: self._accept_tick(token, repeat),
                    self._timing.interval_s,
                )
                self._sender.ack_ref(
                    held.request, MsgType.TASK_ASSIGN_REQUEST, ACK_STATUS_RECEIVED,
                )
                self._send_accept(held)
                return
            self._accept = None
            if self._slot.expire(token) is not None:
                self._logger.error(
                    f"Task {held.task.task_id} from owner "
                    f"{held.owner.owner_id} not applied within "
                    f"{self._timing.silence_s:g}s of its last progress; "
                    "dropping it."
                )

    def _reject_tick(self, key: tuple[int, int], repeat: _Repeat) -> None:
        with self._lock:
            if self._closed or self._rejects.get(key) is not repeat:
                return
            if self._monotonic_s() - repeat.progress_s < self._timing.silence_s:
                self._arm(
                    repeat,
                    lambda: self._reject_tick(key, repeat),
                    self._timing.interval_s,
                )
                self._send_reject(key, repeat)
                return
            del self._rejects[key]

    def _stop_accept(self) -> None:
        if self._accept is not None:
            self._accept[1].cancel()
            self._accept = None

    def _stop_reject(self, key: tuple[int, int]) -> None:
        repeat = self._rejects.pop(key, None)
        if repeat is not None:
            repeat.cancel()

    def _send_accept(self, held: HeldTask) -> None:
        reply = self._sender.assignment_response(
            held.owner.owner_id, held.task.task_id, True,
        )
        if reply is not None:
            self._slot.record_reply(held.token, reply)

    def _send_reject(self, key: tuple[int, int], repeat: _Repeat) -> None:
        reply = self._sender.assignment_response(key[0], key[1], False)
        if reply is not None:
            repeat.refs.add(reply)

    @staticmethod
    def _arm(repeat: _Repeat, tick: Callable[[], None], delay_s: float) -> None:
        timer = rt.Timer(delay_s, tick)
        timer.daemon = True
        repeat.timer = timer
        timer.start()


__all__ = ["AssignReplies"]
