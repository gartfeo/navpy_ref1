"""The own companion's swarm node state, from its SWARM_HEARTBEAT.

The beat's ``state`` says whether the companion can take part in task
auctions (FREE) or holds a peer's task or flies a final approach (BUSY); see
docs/design/swarm-task-assignment-ack.md. Every companion's beat can reach
every vehicle link, so only the own companion's counts. The newest beat by
``(boot, seq)`` wins, a new boot replaces the record, and the state is stale
once the beat outlives its own TTL.
"""
from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.comm.messages.swarm_heartbeat_msg import (
    SwarmHeartbeatMsg,
    SwarmNodeState,
)
from gcs.backend.companion_identity import is_from_companion


@dataclass(frozen=True)
class _Beat:
    state: SwarmNodeState
    boot_id: int
    msg_seq: int
    expires_s: float


class CompanionSwarmState:
    """Newest swarm heartbeat of one vehicle's own companion."""

    def __init__(
        self,
        sys_id: int,
        monotonic_s: Callable[[], float] = time.monotonic,
    ) -> None:
        self._sys_id = sys_id
        self._monotonic_s = monotonic_s
        self._lock = threading.Lock()
        self._beat: Optional[_Beat] = None

    def on_heartbeat(self, msg: MAVLink_message) -> None:
        if not is_from_companion(msg, self._sys_id):
            return
        beat = SwarmHeartbeatMsg.from_mavlink(msg)
        meta = beat.meta
        with self._lock:
            current = self._beat
            if (
                current is not None
                and current.boot_id == meta.boot_id
                and meta.msg_seq <= current.msg_seq
            ):
                return
            self._beat = _Beat(
                SwarmNodeState.from_code(beat.state),
                meta.boot_id,
                meta.msg_seq,
                self._monotonic_s() + meta.ttl_ms / 1000.0,
            )

    def snapshot(self) -> Optional[dict]:
        """``{state, boot, seq, stale}``, or None before the first beat."""
        with self._lock:
            beat = self._beat
        if beat is None:
            return None
        return {
            "state": beat.state.name,
            "boot": beat.boot_id,
            "seq": beat.msg_seq,
            "stale": self._monotonic_s() > beat.expires_s,
        }


__all__ = ["CompanionSwarmState"]
