"""Did the swarm's peers find each other again after a scheduled fault? (step 7)

A ``(receiver, sender)`` pair is owed recovery when both had checked in by the heal:
``receiver`` hears ``sender``, sent after the heal, by the deadline. That needs no
earlier contact: a pair that first met across the fault must still meet.

The roster's lines (``task_peer_roster``) show the product processed that, where a
delivery alone would not. Any silence it logged since it last heard the sender is
over by the deadline. And a pair that was working when the fault began, with a gap
long enough to pass the roster's expiry for certain, logged that silence.

All of it counts only what this receiver's and this sender's generations did: a new
generation starts with a fresh roster, and owes nothing for the last one's peers.
"""

from __future__ import annotations

from collections.abc import Iterable

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.swarm.swarm_heartbeat_runtime import HEARTBEAT_INTERVAL_S
from navpy.core.swarm.task_peer_roster import (
    PEER_HEARD_AGAIN_LOG,
    PEER_SILENCE_EXPIRY_S,
    PEER_SILENT_LOG,
)
from navpy.sim.lockstep.fault_schedule import PRESENCE_MARKER
from scripts.eval_gcs_demo_fault_evidence import FaultRun
from scripts.eval_gcs_demo_fault_schedule import FaultSpan

#: What a roster counts as hearing a peer (``SwarmPresence``).
PEER_IDS = (mavlink.MAVLINK_MSG_ID_SWARM_HEARTBEAT, mavlink.MAVLINK_MSG_ID_CHECK_IN)
#: A gap this long passes the roster's expiry for certain: the expiry, one check period
#: (the check runs on the node's own heartbeat), and one more for a timer the host model delayed.
SILENCE_PROOF_S = PEER_SILENCE_EXPIRY_S + 2 * HEARTBEAT_INTERVAL_S


def both_ways(uav: int, others: Iterable[int]) -> list[tuple[int, int]]:
    """Every ``(receiver, sender)`` pair between ``uav`` and ``others``."""
    others = list(others)
    return [*((other, uav) for other in others), *((uav, other) for other in others)]


def heard_errors(run: FaultRun, span: FaultSpan, pairs: Iterable[tuple[int, int]]) -> list[str]:
    """Each pair heard, sent after the heal, by the deadline."""
    by = run.by_tick(span)
    return [f"fault {span.id}: {receiver} did not hear peer {sender} by tick {by}" for receiver, sender in pairs
            if run.heard(receiver, sender, PEER_IDS, span.heal_k, by) is None]


def silence_over_errors(run: FaultRun, span: FaultSpan, receiver: int, sender: int) -> list[str]:
    """A silence ``receiver``'s generation logged since it last heard ``sender`` is over by the deadline."""
    by = run.by_tick(span)
    before, _after, _gap = run.gap_s([d.k for d in run.received(receiver, sender, PEER_IDS)], span.heal_k)
    since = max(before or 0, run.spawned_at(receiver, by))
    silent_line = PEER_SILENT_LOG.format(sender)
    silent = [k for k, line in run.lines[receiver] if since < k <= by and silent_line in line]
    if silent and run.line_at(receiver, PEER_HEARD_AGAIN_LOG.format(sender), silent[-1] - 1, by) is None:
        return [f"fault {span.id}: {receiver} did not log peer {sender} heard again by tick {by}"]
    return []


def _working_pair(run: FaultRun, receiver: int, sender: int, start_k: int) -> bool:
    """``receiver``'s generation had started its presence and heard ``sender``'s before the fault."""
    spawned = run.spawned_at(receiver, start_k)
    started = next((k for k, line in run.lines[receiver] if k > spawned and PRESENCE_MARKER in line), None)
    sender_spawned = run.spawned_at(sender, start_k)
    return started is not None and any(
        started < d.k < start_k and d.sent_k > sender_spawned and d.source[0] == sender and d.msgid in PEER_IDS
        for d in run.delivered.get(receiver, ()))


def _silence_owed_errors(run: FaultRun, span: FaultSpan, receiver: int, sender: int) -> list[str]:
    """A working pair's gap past the roster's expiry for certain logged its silence."""
    before, after, gap = run.gap_s([d.k for d in run.received(receiver, sender, PEER_IDS)], span.heal_k)
    if before is None or after is None or gap <= SILENCE_PROOF_S:
        return []
    if not _working_pair(run, receiver, sender, span.start_k):
        return []
    if run.line_at(receiver, PEER_SILENT_LOG.format(sender), before, after) is None:
        return [f"fault {span.id}: {receiver} logged no silence of peer {sender} in its {gap:.1f} s gap "
                f"(ticks {before}..{after})"]
    return []


def peer_errors(run: FaultRun, span: FaultSpan, pairs: Iterable[tuple[int, int]]) -> list[str]:
    """Every pair that had checked in by the heal is back by the deadline (module docstring)."""
    owed = [(r, s) for r, s in pairs if run.present(r, span.heal_k) and run.present(s, span.heal_k)]
    errors = heard_errors(run, span, owed)
    for receiver, sender in owed:
        errors += _silence_owed_errors(run, span, receiver, sender)
        errors += silence_over_errors(run, span, receiver, sender)
    return errors


__all__ = ["PEER_IDS", "SILENCE_PROOF_S", "both_ways", "heard_errors", "peer_errors", "silence_over_errors"]
