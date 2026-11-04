"""A replay's stop at tick N (``deterministic_dev_sim.md`` step 12).

The run ends right after tick N's reply: no finalization, drain or park.
The journal's last record is the state snapshot, a run fact like the rest:
the tick and phase, each SITL's frame N (its clock and port state; the
frame's hash is already in tick N's record) and every member as the
roster holds it. The coordinator then stops everything by ownership.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.step_wire import Frame

#: The outcome of a run that a stop at tick N ended.
STOPPED = "stopped"


def stop_record(k: int, phase: str, frames: Sequence[Frame], roster: Roster) -> dict[str, Any]:
    return {"stopped_at": k, "phase": phase, "snapshot": {
        "sitls": [{"source_us": frame.source_us, "helpers_quiet": frame.helpers_quiet,
                   "out_of_loop_advances": frame.out_of_loop_advances,
                   "ports": [[port.fifo_bytes, port.read_buffer_bytes, port.egress_dropped_bytes]
                             for port in frame.ports]} for frame in frames],
        "members": [{"ordinal": member.ordinal, "role": member.spawn.role,
                     "generation": member.spawn.generation, "live": member.live,
                     "ready": member.ready} for member in roster.members],
    }}


__all__ = ["STOPPED", "stop_record"]
