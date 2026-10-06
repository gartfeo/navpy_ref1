"""Strict global task-assignment evidence from the independent GCS log."""

from __future__ import annotations

import math
import re

from scripts.eval_gcs_demo_scenario import DemoMissionPlan


MAX_ASSIGNMENT_COORD_ERROR_M = 1.0
_NUMBER = r"-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_REQUEST = re.compile(
    rf"Task assign request: sender=(?P<sender>\d+) "
    rf"receiver=(?P<receiver>\d+) task_id=(?P<task>\d+) "
    rf"at \((?P<lat>{_NUMBER}),\s*(?P<lon>{_NUMBER})\)"
)
_RESPONSE = re.compile(
    r"Task assign response: sender=(?P<sender>\d+) "
    r"receiver=(?P<receiver>\d+) task_id=(?P<task>\d+) "
    r"accepted=(?P<accepted>True|False)"
)


def derive_global_assignment_map(
    backend_log: str,
    plan: DemoMissionPlan,
) -> dict[int, int]:
    """Return vehicle -> resolved-plan task for a complete accepted auction.

    Auction task IDs belong to the detector's first-seen namespace and are
    therefore correlated with responses only.  The transmitted coordinate is
    the authoritative join to the independently resolved mission plan.
    """
    if type(backend_log) is not str:
        raise TypeError("GCS backend assignment evidence must be text")
    owner_id = plan.owner.sys_id
    peer_ids = {peer.sys_id for peer in plan.peers}
    target_by_id = {target.task_id: target for target in plan.targets}
    requests: list[tuple[int, int, int, float, float]] = []
    responses: list[tuple[int, int, int, bool]] = []

    for line in backend_log.splitlines():
        if "Task assign request:" in line:
            match = _REQUEST.search(line)
            if match is None:
                raise ValueError(f"malformed task assignment request marker: {line}")
            sender = int(match.group("sender"))
            receiver = int(match.group("receiver"))
            if sender == receiver:
                continue
            if sender != owner_id or receiver not in peer_ids:
                raise ValueError(
                    "non-owner task assignment request "
                    f"{sender}->{receiver}"
                )
            task_id = int(match.group("task"))
            lat = float(match.group("lat"))
            lon = float(match.group("lon"))
            matched_targets = [
                target
                for target in plan.targets
                if _distance_m(lat, lon, target.lat, target.lon)
                <= MAX_ASSIGNMENT_COORD_ERROR_M
            ]
            if not matched_targets:
                raise ValueError(
                    f"assignment auction task T{task_id} is not within "
                    f"{MAX_ASSIGNMENT_COORD_ERROR_M:g}m of a plan coordinate"
                )
            if len(matched_targets) != 1:
                raise ValueError(
                    f"assignment auction task T{task_id} ambiguously matches "
                    "multiple plan coordinates"
                )
            requests.append((
                receiver,
                task_id,
                matched_targets[0].task_id,
                lat,
                lon,
            ))
        elif "Task assign response:" in line:
            match = _RESPONSE.search(line)
            if match is None:
                raise ValueError(f"malformed task assignment response marker: {line}")
            responses.append((
                int(match.group("sender")),
                int(match.group("receiver")),
                int(match.group("task")),
                match.group("accepted") == "True",
            ))

    by_peer: dict[int, tuple[int, int, float, float]] = {}
    for peer_id in peer_ids:
        peer_requests = [item for item in requests if item[0] == peer_id]
        if len(peer_requests) != 1:
            raise ValueError(
                "expected exactly one owner assignment request for peer "
                f"{peer_id}, found {len(peer_requests)}"
            )
        _, auction_task_id, plan_task_id, lat, lon = peer_requests[0]
        by_peer[peer_id] = auction_task_id, plan_task_id, lat, lon

    if len(requests) != len(peer_ids):
        raise ValueError("unexpected extra owner assignment request")
    peer_auction_tasks = [item[0] for item in by_peer.values()]
    if len(set(peer_auction_tasks)) != len(peer_auction_tasks):
        raise ValueError("owner assigned the same auction task to multiple peers")
    peer_plan_tasks = [item[1] for item in by_peer.values()]
    if len(set(peer_plan_tasks)) != len(peer_plan_tasks):
        raise ValueError(
            "owner assigned the same resolved-plan target to multiple peers"
        )

    expected_responses = {
        (peer_id, owner_id, task_id)
        for peer_id, (task_id, _plan_task, _lat, _lon) in by_peer.items()
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
            raise ValueError(f"peer {key[0]} rejected global task T{key[2]}")

    remaining = set(target_by_id) - set(peer_plan_tasks)
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
