"""Pydantic request/response schemas for the GCS API."""
from __future__ import annotations

from typing import Literal, Optional
from pydantic import BaseModel, ConfigDict


class LatLon(BaseModel):
    lat: float
    lon: float


class MissionFallbackLocation(LatLon):
    model_config = ConfigDict(extra="forbid")

    type: Optional[str] = None


class VehicleStatus(BaseModel):
    sys_id: int
    name: str
    battery: Optional[int] = None
    mode: Optional[str] = None
    armed: Optional[bool] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    heading: Optional[float] = None
    ground_speed: Optional[float] = None
    link_ok: bool = False
    mission_progress: Optional[int] = None  # current WP
    mission_total: Optional[int] = None


class FencePlan(BaseModel):
    """AO-wide polygon geofence shared by all assigned UAVs.

    Uploaded to each vehicle as MAV_MISSION_TYPE_FENCE points after its mission.
    ``action`` and ``type`` are ArduPlane param values (FENCE_ACTION,
    FENCE_TYPE): action 1 = RTL, type 4 = polygon (inclusion + exclusion).
    ``vertices`` is the single inclusion ring; ``exclusions`` are keep-out rings
    the UAVs must stay out of.
    """
    vertices: list[LatLon] = []
    exclusions: list[list[LatLon]] = []
    enabled: bool = False
    action: int = 1  # FENCE_ACTION: 1 = RTL
    type: int = 4    # FENCE_TYPE bitmask: 4 = polygon (inclusion/exclusion)


class UploadRequest(BaseModel):
    assignments: list[VehicleAssignment]
    fence: Optional[FencePlan] = None


class VehicleAssignment(BaseModel):
    model_config = ConfigDict(extra="forbid")

    sys_id: int
    zone_index: int
    waypoints: list[LatLon]
    altitude_m: float
    corridor_count: int = 0  # number of leading corridor waypoints (corridor pts only)
    corridor_altitude_m: Optional[float] = None  # altitude for corridor approach (default: same as altitude_m)
    search_pattern: str = "distributed"  # search pattern: distributed, corridor
    polygon: list[LatLon] = []  # original planning polygon vertices (for round-trip)
    corridor_backbone: list[LatLon] = []  # corridor backbone waypoints (for round-trip)
    launch_point: Optional[LatLon] = None  # separate launch/home position
    fallback_delivery_location: Optional[MissionFallbackLocation] = None  # configured delivery location


# Fix forward reference
UploadRequest.model_rebuild()


class FenceOutcome(BaseModel):
    """Result of the fence work requested for ONE vehicle.

    Separate from the mission result on purpose: a mission can upload fine
    while a fence parameter write is rejected, and the operator must see that
    instead of an unqualified success. ``action`` records what was attempted:
    ``enable``, ``disable``, or ``rejected`` (the request was refused before
    any vehicle write). ``failed_params`` names the writes the vehicle did not
    acknowledge — the route reports them, it does not claim a rollback.
    """
    applied: bool
    action: Literal["enable", "disable", "rejected"]
    enabled: bool = False
    total: Optional[int] = None
    error: Optional[str] = None
    failed_params: list[str] = []


class UploadProgress(BaseModel):
    sys_id: int
    progress: float  # 0.0 - 1.0
    done: bool
    error: Optional[str] = None
    # None = no fence work was attempted for this vehicle (no fence in the
    # request, or its mission upload failed and it was left untouched).
    fence: Optional[FenceOutcome] = None


class CommandRequest(BaseModel):
    command: str  # "estop", "arm", "set_mode"
    sys_ids: Optional[list[int]] = None  # None = all
    params: Optional[dict] = None


class LaunchRequest(BaseModel):
    sys_ids: list[int]
    force: bool = False
    # Operator acknowledgement that pitots are covered / air is calm, allowing
    # auto preflight cal to re-zero airspeed on pitot-equipped vehicles at START.
    pitot_covered: bool = False


class TriggerRequest(BaseModel):
    sys_id: int


class TaskConfirmResponseRequest(BaseModel):
    sys_id: int
    task_id: int
    is_confirmed: bool
    # Operator intent for UI/audit: approve | deny | cancel | timeout_approve |
    # timeout_deny. The vehicle only consumes is_confirmed; action enriches the
    # WS broadcast and audit log. When None the route derives it from
    # is_confirmed. Validated against is_confirmed in the route. (A per-UAV
    # abort is the destructive E-STOP command, not a confirm action.)
    action: Optional[str] = None
    # Identity of the confirm round this answers, as published with the popup
    # (task_confirm_request.round_uid) and echoed back by the card. It binds
    # the decision to the round the operator saw, so a round that arrives
    # while this response is still being sent is not answered by it. Optional
    # for older clients; a response without it records no decision, meaning
    # a repeat of the answered round re-opens the popup instead of being
    # answered automatically.
    round_uid: Optional[str] = None


class TaskForceConfirmRequest(BaseModel):
    """CONF-03 "Ask me anyway" one-shot gate-bypass override (D-18/D-19/D-20).

    Deliberately minimal: no wire-visible "why" or thumbnail data — the
    operator already sees the blocked reason on the Vehicle status card
    before pressing-and-holding this override.
    """
    sys_id: int
    task_id: int
