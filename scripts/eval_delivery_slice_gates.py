"""Gate for the e2b delivery slice: one UAV, the fleet dock world, no peer.

Pass when, in one NavPy navigation log:
- the first dock the UAV selects is the kept dock (world id), with its
  self-detect orbit, and the UAV asks for its approval (``CONFIRMING``) in
  that orbit;
- the offered dock is first offered inside that approval orbit (from the
  kept dock's selection to the first answer, abort, new orbit or new
  selection, so before or after ``CONFIRMING``), and the send succeeded;
- nothing answers: no ground approval and no assignment traffic in the log.

World id of a detection = the dock file's entry at its obj_id: the sim
detector sets ``DetectionIdentity(poi.uid, poi.uid)`` and the fleet world's
uid is the dock's index in the file. Offers carry no obj_id on the wire, so
the ``Task offered`` log line (``task_rebroadcast.py``) is the link.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional, Sequence

_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
FLEET_DOCKS = re.compile(r"Fleet docks: (?P<docks>.+)$")
SELECTION = re.compile(r"\bPOI: P(?P<task>\d+) \(tracking obj_id=(?P<obj>\d+)\)")
ORBIT = re.compile(r"\bSelf-detect orbit: poi=")
OFFER = re.compile(
    rf"\bTask offered: task_id=(?P<task>\d+) obj_id=(?P<obj>\d+) "
    rf"at \((?P<lat>{_NUMBER}), (?P<lon>{_NUMBER})\) sent=(?P<sent>True|False|None)"
)
RESUME_AUTO = "REJECTED: resuming AUTO mission"
#: Lines that end an approval window: an answer, an abort, or a new orbit or selection.
WINDOW_END = re.compile(
    r"confirmed by ground station\.|CONFIRM max dwell exceeded|CONFIRM re-acquire timeout|"
    rf"{re.escape(RESUME_AUTO)}|\bPOI: P\d+ \(tracking|GUIDED_LOITER cmd=|Self-detect orbit:"
)
#: Any of these means the offer or the approval was answered.
ANSWERED = re.compile(r"confirmed by ground station\.|Assign request:|assignment response|Task assign")


@dataclass(frozen=True)
class SliceResult:
    passed: bool
    errors: list[str]
    evidence: dict[str, str] = field(default_factory=dict)
    diagnostics: list[str] = field(default_factory=list)


def _dock_list(text: str) -> list[str]:
    entries = [part.strip() for part in text.split(";")]
    names = []
    for index, entry in enumerate(entries):
        label, _, name = entry.partition("=")
        if label != f"P{index + 1}" or not name:
            return []
        names.append(name)
    return names


def _first(lines: Sequence[str], pattern: re.Pattern[str], start: int = 0) -> Optional[int]:
    return next((i for i in range(start, len(lines)) if pattern.search(lines[i])), None)


def _world_id(docks: Sequence[str], obj: str) -> Optional[str]:
    index = int(obj)
    return docks[index] if index < len(docks) else None


def evaluate_slice(
    lines: Sequence[str], docks: Sequence[str], kept: str, offered: str,
) -> SliceResult:
    errors: list[str] = []
    evidence: dict[str, str] = {}
    diagnostics: list[str] = []
    fleet = _first(lines, FLEET_DOCKS)
    if fleet is None:
        errors.append("no 'Fleet docks:' line: NavPy did not load the fleet dock world")
    else:
        evidence["fleet_docks"] = lines[fleet]
        logged = _dock_list(FLEET_DOCKS.search(lines[fleet])["docks"])
        if logged != list(docks):
            errors.append(f"fleet docks {logged} differ from the dock file {list(docks)}")

    selection = _first(lines, SELECTION)
    if selection is None:
        return SliceResult(False, errors + ["the UAV selected no dock"], evidence)
    match = SELECTION.search(lines[selection])
    task, selected = match["task"], _world_id(docks, match["obj"])
    evidence["selection"] = lines[selection]
    if selected != kept:
        errors.append(f"first selected dock is {selected!r} (obj_id={match['obj']}), not {kept!r}")

    # The approval orbit: the selection prepares its orbit, then logs itself
    # (navigation_task_action.py), so the orbit line is the last window-end
    # line before the selection. The window runs from the selection to the
    # first answer, abort, new orbit or new selection.
    before = [i for i in range(selection) if WINDOW_END.search(lines[i])]
    orbit = before[-1] if before and ORBIT.search(lines[before[-1]]) else None
    if orbit is None:
        errors.append(f"no self-detect orbit directly before the {kept} selection")
    else:
        evidence["orbit"] = lines[orbit]
    end = _first(lines, WINDOW_END, selection + 1)
    window_end = len(lines) if end is None else end
    evidence["window_end"] = "open at log end" if end is None else lines[end]
    window = None if orbit is None else (selection, window_end)

    confirm_pattern = re.compile(rf"\bCONFIRMING: P{task}\b")
    confirming = _first(lines, confirm_pattern, selection + 1)
    if confirming is None or confirming >= window_end:
        errors.append(f"no approval request (CONFIRMING: P{task}) for {kept} in its orbit")
    else:
        evidence["approval_request"] = lines[confirming]

    offer_index = None
    for i, line in enumerate(lines):
        offer = OFFER.search(line)
        if offer is None:
            continue
        if offer_index is None and _world_id(docks, offer["obj"]) == offered:
            offer_index = i
        else:
            diagnostics.append(f"other offer: {line}")
    if offer_index is None:
        errors.append(f"{offered} was never offered")
    else:
        evidence["offer"] = lines[offer_index]
        if OFFER.search(lines[offer_index])["sent"] != "True":
            errors.append(f"the {offered} offer was not sent: {lines[offer_index]}")
        if window is not None and not window[0] < offer_index < window[1]:
            when = "before" if offer_index < window[0] else "after"
            errors.append(f"{offered} was first offered {when} the {kept} approval orbit")

    answered = [line for line in lines if ANSWERED.search(line)]
    if answered:
        errors.append(f"the run was answered (approval or assignment): {answered[0]}")
    return SliceResult(not errors, errors, evidence, diagnostics)


__all__ = ["SliceResult", "evaluate_slice"]
