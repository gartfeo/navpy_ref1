"""Build and upload a polygon inclusion geofence to a VehicleMav.

Mirrors ``waypoint_builder`` but for the FENCE table: each vertex is a
``MISSION_ITEM_INT`` with command ``MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION``
and ``mission_type = MAV_MISSION_TYPE_FENCE``, uploaded through the standard
mission handshake. ArduPilot writes ``FENCE_TOTAL`` automatically on upload.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    MAV_FRAME_GLOBAL,
    MAV_MISSION_TYPE_FENCE,
    MAVLink_mission_item_int_message,
)

_log = logging.getLogger(__name__)

# Minimum vertices for a valid polygon fence.
MIN_FENCE_VERTICES = 3


def _polygon_items(vertices: list[dict], command: int, target_system: int, seq0: int) -> list:
    """Build one polygon's vertex items starting at seq ``seq0``.

    Every vertex carries ``param1 = this polygon's vertex count`` so ArduPilot
    groups the run into a single polygon (it walks the flat item list and starts
    a new polygon after each param1-length run — verified against
    AC_PolyFence_loader::validate_fence).
    """
    n = len(vertices)
    return [
        MAVLink_mission_item_int_message(
            target_system=target_system,
            target_component=0,
            seq=seq0 + i,
            frame=MAV_FRAME_GLOBAL,
            command=command,
            current=0,
            autocontinue=1,
            param1=float(n),
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x=int(round(v["lat"] * 1e7)),
            y=int(round(v["lon"] * 1e7)),
            z=0.0,
            mission_type=MAV_MISSION_TYPE_FENCE,
        )
        for i, v in enumerate(vertices)
    ]


def build_fence_items(
    vertices: list[dict],
    target_system: int,
    exclusions: list[list[dict]] | None = None,
) -> list:
    """Build fence items: one inclusion polygon, then each exclusion polygon.

    ``vertices`` is the inclusion ring ``[{"lat": .., "lon": ..}, ...]``.
    ``exclusions`` is a list of keep-out rings. Polygons are concatenated as
    consecutive param1-delimited runs; exclusion vertices use the EXCLUSION
    command so ArduPilot keeps the vehicle OUT of them.
    """
    items = _polygon_items(vertices, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, target_system, 0)
    for ring in (exclusions or []):
        if len(ring) < MIN_FENCE_VERTICES:
            continue  # a degenerate keep-out can't be enforced — skip it
        items += _polygon_items(
            ring, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, target_system, len(items),
        )
    return items


def parse_fence_items(items) -> dict:
    """Parse a flat fence item list back into inclusion + exclusion rings.

    Inverse of :func:`build_fence_items`. Walks the flat list, grouping
    consecutive param1-delimited runs: every item in a polygon run carries
    ``param1 = that polygon's vertex count`` and shares the same polygon-vertex
    command. Non-polygon items (return point, circle inclusion/exclusion, or any
    unknown command another GCS may add) are skipped one at a time.

    Item coordinates are read as ``item.x / 1e7`` (lat) and ``item.y / 1e7``
    (lon), matching the MISSION_ITEM_INT scaling ``build_fence_items`` writes.

    Robust to malformed runs: if a run's declared count exceeds the number of
    consecutive same-command items actually present, the parser keeps the valid
    vertices it collected and resumes at the divergence point. It never raises.

    Returns ``{"vertices": [...first inclusion ring, [] if none...],
    "exclusions": [[...], ...]}``.
    """
    poly_cmds = {
        MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
        MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    }
    inclusion_rings: list[list[dict]] = []
    exclusion_rings: list[list[dict]] = []

    n = len(items)
    i = 0
    while i < n:
        cmd = getattr(items[i], "command", None)
        if cmd not in poly_cmds:
            i += 1  # return point / circle / unknown — skip one item, keep going
            continue

        try:
            count = int(round(float(getattr(items[i], "param1", 0) or 0)))
        except (TypeError, ValueError):
            count = 0
        if count <= 0:
            i += 1  # untrustworthy run header — skip this single item
            continue

        # Collect up to `count` consecutive vertices that share this command.
        ring: list[dict] = []
        j = i
        while j < n and len(ring) < count and getattr(items[j], "command", None) == cmd:
            v = items[j]
            ring.append({
                "lat": (getattr(v, "x", 0) or 0) / 1e7,
                "lon": (getattr(v, "y", 0) or 0) / 1e7,
            })
            j += 1

        if ring:  # keep what's valid (partial runs tolerated)
            if cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                inclusion_rings.append(ring)
            else:
                exclusion_rings.append(ring)

        i = j if j > i else i + 1  # resume where the run actually ended

    return {
        "vertices": inclusion_rings[0] if inclusion_rings else [],
        "exclusions": exclusion_rings,
    }


@dataclass
class FenceUploadResult:
    success: bool
    vertex_count: int
    error: str | None = None


def upload_fence_with_retry(
    vehicle,
    vertices: list[dict],
    exclusions: list[list[dict]] | None = None,
    max_retries: int = 3,
) -> FenceUploadResult:
    """Clear + upload the inclusion fence (+ exclusion keep-outs), retrying
    until the vehicle ACKs.

    Success is the mission-protocol ``MISSION_ACK`` (``upload_fence`` returns
    True only on ``MAV_MISSION_ACCEPTED``) — ArduPilot's authoritative signal
    that it stored every fence item. We deliberately do NOT read FENCE_TOTAL
    back to verify: ``get_parameter`` serves a cached value that a fence upload
    never invalidates, so a stale/racy read could report a real, ACK'd fence as
    failed (leaving the vehicle armed with a fence the operator was told failed).

    ``vertex_count`` is the total items uploaded (inclusion + exclusions), which
    is what ArduPilot writes to FENCE_TOTAL.
    """
    if len(vertices) < MIN_FENCE_VERTICES:
        return FenceUploadResult(
            success=False, vertex_count=len(vertices),
            error=f"need >= {MIN_FENCE_VERTICES} vertices, got {len(vertices)}",
        )

    items = build_fence_items(vertices, vehicle.target_system, exclusions)

    for attempt in range(1, max_retries + 1):
        # clear + upload is one transaction under the mission lock so a
        # concurrent mission op can't interleave on the shared protocol queue.
        with vehicle.mission_lock:
            vehicle.clear_fence()
            ok = vehicle.upload_fence(items)

        if ok:
            return FenceUploadResult(success=True, vertex_count=len(items))

        _log.warning("Fence upload attempt %d/%d got no ACK, retrying...", attempt, max_retries)

    return FenceUploadResult(
        success=False, vertex_count=len(vertices),
        error=f"Fence upload failed after {max_retries} attempts",
    )
