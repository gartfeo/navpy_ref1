"""Strict global task-assignment evidence from the independent GCS log."""

from __future__ import annotations

import math
from dataclasses import dataclass

from scripts.eval_gcs_demo_assignment_lines import (
    AssignmentLines,
    LogFormat,
    RequestLine,
    parse_assignment_lines,
)
from scripts.eval_gcs_demo_scenario import DemoMissionPlan


MAX_ASSIGNMENT_COORD_ERROR_M = 1.0


@dataclass(frozen=True)
class _PlannedRequest:
    receiver: int
    task: int
    plan_task: int


def derive_global_assignment_map(
    backend_log: str,
    plan: DemoMissionPlan,
) -> dict[int, int]:
    """Return vehicle -> resolved-plan task for a complete accepted auction.

    Auction task IDs belong to the detector's first-seen namespace and are
    therefore correlated with responses only.  The transmitted coordinate is
    the authoritative join to the independently resolved mission plan.

    Legacy logs need exactly one request and one accepted response per peer.
    Acked logs (one line per copy, docs/design/swarm-task-assignment-ack.md)
    need exactly one owner task per peer whose APPLIED names an accepted
    response of that peer; copies, released rounds and rejects carry none.
    """
    lines = parse_assignment_lines(backend_log)
    owner_id = plan.owner.sys_id
    peer_ids = {peer.sys_id for peer in plan.peers}
    requests = _owner_requests(lines, plan, owner_id, peer_ids)
    if lines.log_format is LogFormat.ACKED:
        by_peer = _applied_tasks(lines, requests, owner_id, peer_ids)
        _require_distinct(by_peer)
    else:
        by_peer = _requested_tasks(requests, peer_ids)
        _require_distinct(by_peer)
        _require_accepted_responses(lines, by_peer, owner_id)
    peer_plan_tasks = {plan_task for _task, plan_task in by_peer.values()}
    remaining = {poi.task_id for poi in plan.pois} - peer_plan_tasks
    if len(remaining) != 1:
        raise ValueError("accepted peer assignments do not leave one owner task")
    owner_task = remaining.pop()
    return {
        vehicle.sys_id: (
            owner_task
            if vehicle.sys_id == owner_id
            else by_peer[vehicle.sys_id][1]
        )
        for vehicle in plan.vehicles
    }


def _owner_requests(
    lines: AssignmentLines,
    plan: DemoMissionPlan,
    owner_id: int,
    peer_ids: set[int],
) -> list[_PlannedRequest]:
    requests = []
    for request in lines.requests:
        if request.sender == request.receiver:
            continue
        if request.sender != owner_id or request.receiver not in peer_ids:
            raise ValueError(
                "non-owner task assignment request "
                f"{request.sender}->{request.receiver}"
            )
        requests.append(_PlannedRequest(
            request.receiver, request.task, _plan_task(request, plan),
        ))
    return requests


def _plan_task(request: RequestLine, plan: DemoMissionPlan) -> int:
    matched_pois = [
        poi
        for poi in plan.pois
        if _distance_m(request.lat, request.lon, poi.lat, poi.lon)
        <= MAX_ASSIGNMENT_COORD_ERROR_M
    ]
    if not matched_pois:
        raise ValueError(
            f"assignment auction task P{request.task} is not within "
            f"{MAX_ASSIGNMENT_COORD_ERROR_M:g}m of a plan coordinate"
        )
    if len(matched_pois) != 1:
        raise ValueError(
            f"assignment auction task P{request.task} ambiguously matches "
            "multiple plan coordinates"
        )
    return matched_pois[0].task_id


def _requested_tasks(
    requests: list[_PlannedRequest],
    peer_ids: set[int],
) -> dict[int, tuple[int, int]]:
    """Legacy: exactly one owner request per peer, and no other."""
    by_peer: dict[int, tuple[int, int]] = {}
    for peer_id in peer_ids:
        peer_requests = [item for item in requests if item.receiver == peer_id]
        if len(peer_requests) != 1:
            raise ValueError(
                "expected exactly one owner assignment request for peer "
                f"{peer_id}, found {len(peer_requests)}"
            )
        by_peer[peer_id] = peer_requests[0].task, peer_requests[0].plan_task
    if len(requests) != len(peer_ids):
        raise ValueError("unexpected extra owner assignment request")
    return by_peer


def _applied_tasks(
    lines: AssignmentLines,
    requests: list[_PlannedRequest],
    owner_id: int,
    peer_ids: set[int],
) -> dict[int, tuple[int, int]]:
    """Acked: each peer's one task, from the owner's APPLIED lines."""
    accepted = {
        (line.sender, line.receiver, line.task, line.uid)
        for line in lines.responses
        if line.accepted
    }
    applied: dict[int, set[int]] = {peer_id: set() for peer_id in peer_ids}
    for ack in lines.acks:
        if ack.owner != owner_id or ack.helper not in peer_ids:
            raise ValueError(
                f"non-owner task assignment ack {ack.owner}->{ack.helper}"
            )
        if (ack.helper, ack.owner, ack.task, ack.ref) not in accepted:
            raise ValueError(
                f"APPLIED ref {ack.ref[0]}:{ack.ref[1]} names no accepted "
                f"assignment response of peer {ack.helper} for P{ack.task}"
            )
        applied[ack.helper].add(ack.task)
    by_peer: dict[int, tuple[int, int]] = {}
    for peer_id, tasks in sorted(applied.items()):
        if len(tasks) != 1:
            raise ValueError(
                "expected exactly one owner task APPLIED to peer "
                f"{peer_id}, found {len(tasks)}"
            )
        task = tasks.pop()
        plan_tasks = {
            item.plan_task
            for item in requests
            if item.receiver == peer_id and item.task == task
        }
        if len(plan_tasks) != 1:
            raise ValueError(
                f"APPLIED task P{task} of peer {peer_id} has "
                f"{len(plan_tasks)} plan coordinates in its requests"
            )
        by_peer[peer_id] = task, plan_tasks.pop()
    return by_peer


def _require_distinct(by_peer: dict[int, tuple[int, int]]) -> None:
    peer_auction_tasks = [task for task, _plan_task in by_peer.values()]
    if len(set(peer_auction_tasks)) != len(peer_auction_tasks):
        raise ValueError("owner assigned the same auction task to multiple peers")
    peer_plan_tasks = [plan_task for _task, plan_task in by_peer.values()]
    if len(set(peer_plan_tasks)) != len(peer_plan_tasks):
        raise ValueError(
            "owner assigned the same resolved-plan POI to multiple peers"
        )


def _require_accepted_responses(
    lines: AssignmentLines,
    by_peer: dict[int, tuple[int, int]],
    owner_id: int,
) -> None:
    """Legacy: exactly one accepted response per assignment, and no other."""
    responses = [
        (line.sender, line.receiver, line.task, line.accepted)
        for line in lines.responses
    ]
    expected_responses = {
        (peer_id, owner_id, task_id)
        for peer_id, (task_id, _plan_task) in by_peer.items()
    }
    observed_response_keys = {
        (sender, receiver, task_id)
        for sender, receiver, task_id, _accepted in responses
    }
    extra = observed_response_keys - expected_responses
    if extra:
        raise ValueError(f"unexpected task assignment responses: {sorted(extra)}")
    for key in expected_responses:
        matches = [item for item in responses if item[:3] == key]
        if len(matches) != 1:
            raise ValueError(
                f"expected exactly one assignment response {key}, found {len(matches)}"
            )
        if not matches[0][3]:
            raise ValueError(f"peer {key[0]} rejected global task P{key[2]}")


def _distance_m(
    lat_a: float,
    lon_a: float,
    lat_b: float,
    lon_b: float,
) -> float:
    if not (
        _valid_coordinate(lat_a, lon_a)
        and _valid_coordinate(lat_b, lon_b)
    ):
        return math.inf
    radius_m = 6_371_000.0
    phi_a, phi_b = math.radians(lat_a), math.radians(lat_b)
    delta_phi = phi_b - phi_a
    delta_lon = math.radians(lon_b - lon_a)
    haversine = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi_a) * math.cos(phi_b) * math.sin(delta_lon / 2.0) ** 2
    )
    return 2.0 * radius_m * math.asin(min(1.0, math.sqrt(haversine)))


def _valid_coordinate(lat: float, lon: float) -> bool:
    return (
        math.isfinite(lat)
        and math.isfinite(lon)
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
    )


__all__ = ["MAX_ASSIGNMENT_COORD_ERROR_M", "derive_global_assignment_map"]
