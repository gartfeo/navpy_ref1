"""The own companion's swarm node state, from its SWARM_HEARTBEAT.

The beat's ``state`` says whether the companion can take part in task
auctions (FREE) or holds a peer's task or flies a final approach (BUSY); see
docs/design/swarm-task-assignment-ack.md. Every companion's beat can reach
every vehicle link, so only the own companion's counts. The newest beat by
``(boot, seq)`` wins, a new boot replaces the record, a beat from a boot it
replaced is dropped, and the state is stale once the beat outlives its own
TTL. Bits 1-7 of the byte carry NavPy's display phase and last outcome
(docs/design/gcs_mission_phase.md); they are shown, never acted on.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
from navpy.core.link.swarm_heartbeat_msg import (
    SwarmHeartbeatMsg,
    SwarmNodeState,
)
from navpy.core.flow.nav_display import DecodedNavDisplay, decode_state_bits
from gcs.backend.companion_identity import is_from_companion


@dataclass(frozen=True)
class _Beat:
    state: SwarmNodeState
    display: DecodedNavDisplay
    boot_id: int
    msg_seq: int
    expires_s: float


class CompanionSwarmState:
    """Newest swarm heartbeat of one vehicle's own companion."""

    def __init__(
        self,
        sys_id: int,
        monotonic_s: Callable[[], float] = rt.monotonic,
    ) -> None:
        self._sys_id = sys_id
        self._monotonic_s = monotonic_s
        self._lock = rt.Lock()
        self._beat: Optional[_Beat] = None
        # Boots a newer boot replaced: their late beats are dropped.
        self._retired_boots: set[int] = set()
        # Seq of the current boot's newest APPROACH beat: the frontend labels
        # only assignment slots the helper answered after it.
        self._approach_seq: Optional[int] = None

    def on_heartbeat(self, msg: MAVLink_message) -> None:
        if not is_from_companion(msg, self._sys_id):
            return
        beat = SwarmHeartbeatMsg.from_mavlink(msg)
        meta = beat.meta
        with self._lock:
            current = self._beat
            if meta.boot_id in self._retired_boots:
                return
            if current is not None and current.boot_id == meta.boot_id:
                if meta.msg_seq <= current.msg_seq:
                    return
            elif current is not None:
                self._retired_boots.add(current.boot_id)
                self._approach_seq = None
            self._beat = _Beat(
                SwarmNodeState.from_code(beat.state),
                decode_state_bits(beat.state),
                meta.boot_id,
                meta.msg_seq,
                self._monotonic_s() + meta.ttl_ms / 1000.0,
            )
            if self._beat.display.phase == "APPROACH":
                self._approach_seq = meta.msg_seq

    def snapshot(self) -> Optional[dict]:
        """``{state, phase, outcome, fail_reason, approach_seq, boot, seq,
        stale}``, or None before the first beat. ``phase`` is None from an
        older companion that sends no display bits."""
        with self._lock:
            beat = self._beat
            approach_seq = self._approach_seq
        if beat is None:
            return None
        return {
            "state": beat.state.name,
            "phase": beat.display.phase,
            "outcome": beat.display.outcome,
            "fail_reason": beat.display.fail_reason,
            "approach_seq": approach_seq,
            "boot": beat.boot_id,
            "seq": beat.msg_seq,
            "stale": self._monotonic_s() > beat.expires_s,
        }


__all__ = ["CompanionSwarmState"]
