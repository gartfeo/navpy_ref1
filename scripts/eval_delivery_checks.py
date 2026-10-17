"""The delivery run's log checks, by role (e6).

Pure functions over parsed evidence; ``eval_delivery_gates.py`` loads it.
Dock ids are world ids: a detection's obj_id indexes the run's
``fleet_docks.json``. See docs/design/delivery_eval_scenario.md, "e6 as built".
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence

from navpy.core.flow.nav_constants import DOCK_ASSOCIATION_RADIUS_M
from navpy.core.kernel.location import Location
from navpy.sim.world.fleet_dock_world import DockSpec
from scripts.eval_delivery_roles import HOME_DOCK_MARKER, Applied
from scripts.eval_gcs_demo_assignment_lines import AssignmentLines
from scripts.eval_gcs_demo_role_gates import PEER_APPROACH_MARKER

#: Offers and assignment requests carry the dock's own position.
MAX_DOCK_COORD_ERROR_M = 1.0
_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_OBJ_RE = re.compile(r"\bobj_id=(-?\d+)")
_OFFER_RE = re.compile(
    rf"Task offered: task_id=(?P<task>\d+) obj_id=(?P<obj>-?\d+) "
    rf"at \((?P<lat>{_NUMBER}), (?P<lon>{_NUMBER})\)"
)
_ASSIGNED_RE = re.compile(r"\bTask (?P<task>\d+) assigned by owner (?P<owner>\d+)\b")
_REPOSITION_RE = re.compile(
    rf"GUIDED_LOITER cmd=DO_REPOSITION center=(?P<lat>{_NUMBER}),(?P<lon>{_NUMBER}),"
)
_HUB_RE = re.compile(rf"{re.escape(HOME_DOCK_MARKER)} (?P<lat>{_NUMBER}), (?P<lon>{_NUMBER}),")
_AUTO_RESUME = "resuming AUTO mission"
_ORBIT = "Self-detect orbit:"


def distance_m(lat: float, lon: float, dock: DockSpec) -> float:
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return math.inf
    return Location(lat, lon, 0.0).distance_to(Location(dock.lat, dock.lon, 0.0))


def world_id_errors(texts: Mapping[int, str], ids: Mapping[int, Sequence[int]], count: int) -> list[str]:
    """Every obj_id in every log, and every episode dock id, is a world dock."""
    errors = []
    for sys_id, text in texts.items():
        found = [int(match) for match in _OBJ_RE.findall(text)] + list(ids.get(sys_id, ()))
        unknown = sorted({obj for obj in found if not 0 <= obj < count})
        if unknown:
            errors.append(f"UAV {sys_id} names dock ids {unknown} outside the {count} world docks")
    return errors


def offer_errors(
    texts: Mapping[int, str], offerer: int, docks: Sequence[DockSpec],
    offered: int, homes: Sequence[int], applied: Applied | None,
) -> list[str]:
    """Exactly one logical offer fleet-wide: the offerer's, of the offered dock
    at its position, for the APPLIED task; no offer of, or near, a home dock."""
    errors: list[str] = []
    logical: dict[tuple[int, int], set[tuple[int, float, float]]] = {}
    for sys_id, text in texts.items():
        for match in _OFFER_RE.finditer(text):
            obj, lat, lon = int(match["obj"]), float(match["lat"]), float(match["lon"])
            near = [docks[home].name for home in homes
                    if obj == home or distance_m(lat, lon, docks[home]) <= DOCK_ASSOCIATION_RADIUS_M]
            if near:
                errors.append(f"UAV {sys_id} offered a home dock ({near[0]}): {match[0]!r}")
            logical.setdefault((sys_id, int(match["task"])), set()).add((obj, lat, lon))
    if len(logical) != 1:
        return errors + [f"expected exactly one logical offer, found {sorted(logical)}"]
    (sender, task), payloads = next(iter(logical.items()))
    if sender != offerer:
        errors.append(f"the offer came from UAV {sender}, not the offerer {offerer}")
    if applied is not None and task != applied[2]:
        errors.append(f"the offered task {task} is not the APPLIED task {applied[2]}")
    for obj, lat, lon in sorted(payloads):
        if obj != offered or distance_m(lat, lon, docks[offered]) > MAX_DOCK_COORD_ERROR_M:
            errors.append(
                f"offer of task {task} names obj {obj} at ({lat}, {lon}), "
                f"not {docks[offered].name}"
            )
    return errors


def assignment_errors(
    lines: AssignmentLines, applied: Applied, home: int | None, dock: DockSpec,
) -> list[str]:
    """The APPLIED names the winner's accepted bid; the request carries the
    offered dock; nothing involves the home UAV."""
    owner, helper, task = applied
    errors: list[str] = []
    acks = [ack for ack in lines.acks if (ack.owner, ack.helper, ack.task) == applied]
    responses = [
        line for line in lines.responses
        if (line.sender, line.receiver, line.task) == (helper, owner, task)
    ]
    if any(not line.accepted for line in responses):
        errors.append(f"winner {helper} also rejected task {task}")
    accepted = {line.uid for line in responses if line.accepted}
    if not accepted:
        errors.append(f"winner {helper} sent no accepted bid for task {task}")
    if any(ack.ref not in accepted for ack in acks):
        errors.append(f"an APPLIED for task {task} names no accepted bid of {helper}")
    requests = [
        line for line in lines.requests
        if (line.sender, line.receiver, line.task) == (owner, helper, task)
    ]
    if not requests:
        errors.append(f"no assignment request {owner}->{helper} for task {task}")
    if any(distance_m(line.lat, line.lon, dock) > MAX_DOCK_COORD_ERROR_M for line in requests):
        errors.append(f"assignment request for task {task} is not at {dock.name}")
    if home is not None:
        involved = [line for line in lines.requests if home in (line.sender, line.receiver)]
        involved += [line for line in lines.responses if home in (line.sender, line.receiver)]
        involved += [ack for ack in lines.acks if home in (ack.owner, ack.helper)]
        if involved:
            errors.append(f"home UAV {home} took part in {len(involved)} assignment lines")
    return errors


def _at(text: str, marker: str, start: int = 0) -> int:
    return text.find(marker, start)


def _selection(obj: int) -> str:
    return f"(tracking obj_id={obj})"


def offerer_errors(text: str, dock: int) -> list[str]:
    """Orbit, then the d1 detection lock, then its selection and review."""
    errors = []
    if _ASSIGNED_RE.search(text) or PEER_APPROACH_MARKER in text:
        errors.append("offerer flew a peer task")
    lock = _at(text, f"start_tracking obj_id={dock} ")
    select = _at(text, _selection(dock))
    confirming = _at(text, "CONFIRMING: P")
    orbit = _at(text, _ORBIT)
    if not 0 <= orbit < lock <= select < confirming:
        errors.append(f"offerer order is not orbit -> lock -> select -> CONFIRMING for obj {dock}")
    return errors


def winner_errors(text: str, applied: Applied, docks: Sequence[DockSpec], dock: int) -> list[str]:
    """Assignment, then DO_REPOSITION to the dock, then the peer approach,
    then its selection and review; no return to the search."""
    owner, _helper, task = applied
    assigned = next(
        (match for match in _ASSIGNED_RE.finditer(text)
         if (int(match["task"]), int(match["owner"])) == (task, owner)),
        None,
    )
    if assigned is None:
        return [f"winner never logged 'Task {task} assigned by owner {owner}'"]
    errors = []
    reposition = _REPOSITION_RE.search(text, assigned.end())
    if reposition is None or distance_m(
        float(reposition["lat"]), float(reposition["lon"]), docks[dock],
    ) > DOCK_ASSOCIATION_RADIUS_M:
        errors.append(f"winner has no DO_REPOSITION to {docks[dock].name} after its assignment")
    approach = _at(text, PEER_APPROACH_MARKER, assigned.end())
    select = _at(text, _selection(dock), max(approach, 0))
    confirming = _at(text, "CONFIRMING: P", max(select, 0))
    if reposition is not None and not reposition.start() < approach < select < confirming:
        errors.append(
            "winner order is not assigned -> DO_REPOSITION -> peer navigation "
            f"-> select obj {dock} -> CONFIRMING"
        )
    if _AUTO_RESUME in text[assigned.end():]:
        errors.append("winner resumed its AUTO search after the assignment")
    return errors


def home_errors(text: str, docks: Sequence[DockSpec], dock: int) -> list[str]:
    """Hub navigation to its own home dock, then that dock's lock and selection."""
    errors = []
    if _ASSIGNED_RE.search(text) or PEER_APPROACH_MARKER in text or "Task offered:" in text:
        errors.append("home UAV offered or flew a peer task")
    hub = _HUB_RE.search(text)
    if hub is None:
        return errors + [f"home UAV never logged {HOME_DOCK_MARKER!r}"]
    if distance_m(float(hub["lat"]), float(hub["lon"]), docks[dock]) > DOCK_ASSOCIATION_RADIUS_M:
        errors.append(f"home UAV's hub navigation is not at {docks[dock].name}")
    lock = _at(text, f"start_tracking obj_id={dock} ", hub.end())
    select = _at(text, _selection(dock), hub.end())
    if not 0 <= lock <= select:
        errors.append(f"home UAV did not lock and select {docks[dock].name} after its hub navigation")
    return errors


__all__ = [
    "MAX_DOCK_COORD_ERROR_M",
    "assignment_errors",
    "distance_m",
    "home_errors",
    "offer_errors",
    "offerer_errors",
    "winner_errors",
    "world_id_errors",
]
