"""Role checks of each gate profile: owner dispatch or its absence, and peers."""

from __future__ import annotations

import re
from collections.abc import Callable

from scripts.eval_gcs_demo_gate_profiles import (
    CERTIFIED_LOCKSTEP_PROFILE,
    CERTIFIED_PROFILE,
    OWNER_ONLY_SLICE_PROFILE,
)


_ASSIGNED_RE = re.compile(r"\bTask \d+ assigned by owner \d+\b")
# Logged when the peer approach is dispatched; a GUIDED_LOITER alone can be a
# free peer's default delivery hub return, before any assignment.
PEER_APPROACH_MARKER = "Peer navigation started"
#: NavPy log markers of task dispatch, the auction or peer flight (label,
#: pattern). The certified owner must show its dispatch (the first two); an
#: owner-only slice must show none of them. Every line the dispatch, auction
#: and peer-participation modules log is covered; a test enumerates their
#: logging calls, so a new line fails until it is listed here.
PEER_ASSIGNMENT_MARKERS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("task rebroadcast", re.compile(r"\bRebroadcast task \d+")),  # task_rebroadcast.py
    ("task accepted by a peer", re.compile(r"\bTask \d+ accepted by \d+\b")),  # task_auction_coordinator.py
    ("peer task offer", re.compile(r"\bTask \d+ can be handled by \d+")),
    ("assignment request ack", re.compile(r"\bTask \d+: request acked by \d+")),
    ("task declined by a peer", re.compile(r"\bTask \d+ declined by \d+")),
    ("task rejected by a peer", re.compile(r"\bTask \d+ rejected by \d+")),
    ("task kept by a peer", re.compile(r"\bTask \d+ stays with \d+")),
    ("dispatch not reserved", re.compile(r"\bTask dispatch \d+ not reserved to \d+")),
    ("unfenced response", re.compile(r"\bTask \d+: response from \d+ precedes")),
    ("unadmitted peer offer", re.compile(r"\bIgnoring task offer from unadmitted peer \d+")),
    ("unrecorded offer", re.compile(r"\bTask \d+: offer from \d+ not recorded")),
    ("assignment retries exhausted", re.compile(r"\bTask \d+ assignment failed after \d+ retries")),
    ("assignment request resent", re.compile(r"\bTask \d+: request not acked by \d+")),
    ("assignment request released", re.compile(r"\bTask \d+: no confirmed response from \d+")),
    ("assignment retry", re.compile(r"\bTask \d+ retry: no free peer")),
    ("global assignment", re.compile(r"\bGlobal assignment:")),
    ("dispatch dropped", re.compile(r"\bdrops task \d+ \w+ to peer \d+")),  # task_auction_lifecycle.py
    ("dispatch shutdown error", re.compile(r"\bTaskDispatch shutdown error:")),
    ("task assignment from an owner", _ASSIGNED_RE),  # task_capability.py
    ("offer from an owner", re.compile(r"\bTask \d+ offered by owner \d+")),
    ("offer rejected", re.compile(r"\bTask \d+ from owner \d+ rejected:")),
    ("offer withdrawn", re.compile(r"\bTask \d+: owner \d+ no longer offers it")),
    ("offer dropped on approach", re.compile(r"\bOwn final approach started; rejecting task \d+")),
    ("offer re-advertised", re.compile(r"\bTask \d+ re-advertised by owner \d+")),
    ("advert while busy", re.compile(r"\bBusy \(holding a task or flying a final approach\); not bidding")),
    ("advert without location", re.compile(r"\bLocation not set\. Cannot send available message")),
    ("first task offer", re.compile(r"\bTask offered: task_id=\d+")),  # task_rebroadcast.py
    ("advert with no task", re.compile(r"\bNo tasks can handle\b")),
    ("assignment not applied", re.compile(r"\bTask \d+ from owner \d+ not applied within")),  # task_assign_reply.py
    ("peer approach", re.compile(re.escape(PEER_APPROACH_MARKER))),  # peer_approach_start.py
    ("peer task priority", re.compile(r"\bflying peer task\b")),  # peer_task_priority.py
)


def _peer_assignment_errors(navigation_text: str) -> list[str]:
    """A peer flies its task only once its owner applied it (acked logs)."""
    assigned = _ASSIGNED_RE.search(navigation_text)
    if assigned is None:
        return ["peer never logged 'Task T assigned by owner O'"]
    approach = navigation_text.find(PEER_APPROACH_MARKER)
    if approach < 0:
        return [f"peer never logged {PEER_APPROACH_MARKER!r}"]
    if approach < assigned.start():
        return [f"peer logged {PEER_APPROACH_MARKER!r} before 'Task T assigned by owner O'"]
    return []


def _owner_orbit_errors(navigation_text: str) -> list[str]:
    if "Self-detect orbit:" not in navigation_text:
        return ["owner did not keep a POI-centred self-detect orbit"]
    return []


def _owner_dispatch_errors(navigation_text: str) -> list[str]:
    errors: list[str] = []
    for task_id in (2, 3):
        if f"Rebroadcast task {task_id}" not in navigation_text and re.search(rf"\bTask {task_id} accepted by \d+\b", navigation_text) is None:
            errors.append(f"owner did not dispatch task {task_id} to peers")
    return errors


def _peer_role_errors(navigation_text: str, assignment_acked: bool) -> list[str]:
    errors: list[str] = []
    if "No POI is set" not in navigation_text:
        errors.append("peer did not start with targ_wps=0")
    if "GUIDED_LOITER" not in navigation_text:
        errors.append("peer did not execute GUIDED_LOITER assignment approach")
    if assignment_acked:
        errors.extend(_peer_assignment_errors(navigation_text))
    return errors


def peer_assignment_marker_errors(navigation_text: str) -> list[str]:
    """Any dispatch, auction or peer marker: an owner-only run has no peers."""
    return [
        f"owner-only slice logged {label} marker ({match.group(0)!r})"
        for label, pattern in PEER_ASSIGNMENT_MARKERS
        for match in (pattern.search(navigation_text),)
        if match is not None
    ]


def _certified_role_errors(
    navigation_text: str, role: str, assignment_acked: bool,
) -> list[str]:
    if role == "owner":
        return _owner_dispatch_errors(navigation_text) + _owner_orbit_errors(navigation_text)
    return _peer_role_errors(navigation_text, assignment_acked)


def _owner_only_slice_role_errors(
    navigation_text: str, role: str, assignment_acked: bool,
) -> list[str]:
    del assignment_acked  # no peer ever applies an assignment in the slice
    if role != "owner":
        return [f"the owner-only slice has no {role} role"]
    # The wire-level checks (zero task-dispatch, advert and peer-relay MAVLink
    # messages per link, the workflow tick order, the confirmation timing) are
    # the profile's journal gates: eval_gcs_demo_slice_gates.
    return _owner_orbit_errors(navigation_text) + peer_assignment_marker_errors(navigation_text)


#: Execution profile -> its role checks; the rest of the workflow gates are
#: shared. Lockstep keeps the certified roles (owner dispatch and orbit, peers).
ROLE_GATES: dict[str, Callable[[str, str, bool], list[str]]] = {
    CERTIFIED_PROFILE: _certified_role_errors,
    CERTIFIED_LOCKSTEP_PROFILE: _certified_role_errors,
    OWNER_ONLY_SLICE_PROFILE: _owner_only_slice_role_errors,
}


__all__ = [
    "PEER_APPROACH_MARKER",
    "PEER_ASSIGNMENT_MARKERS",
    "ROLE_GATES",
    "peer_assignment_marker_errors",
]
