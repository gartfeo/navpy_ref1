"""Gate profiles a demo scenario names, and the scenario shape each requires."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from scripts.eval_gcs_demo_constants import SIM_SPEEDUP

CERTIFIED_PROFILE = "certified"
OWNER_ONLY_SLICE_PROFILE = "owner_only_slice"
#: The certified scenario run in lockstep: a resolved execution profile, never
#: a scenario's own ``gate_profile`` (``lockstep_fleet.md``, Execution profile).
CERTIFIED_LOCKSTEP_PROFILE = "certified_lockstep"
WALL, LOCKSTEP = "wall", "lockstep"
# Owner track rows only: NavPy also counts the appended hub row the GCS parser drops.
CERTIFIED_POI_NAV_WAYPOINT_ORDINALS = (2, 3, 6)


def _require_certified_shape(roles: Sequence[str], ordinals: tuple[int, ...]) -> None:
    if sorted(roles) != ["owner", "peer", "peer"]:
        raise ValueError(
            "the certified scenario requires exactly three slots: one owner and two peers"
        )
    if ordinals != CERTIFIED_POI_NAV_WAYPOINT_ORDINALS:
        raise ValueError(
            "the certified scenario POI NAV waypoint ordinals must be "
            f"{list(CERTIFIED_POI_NAV_WAYPOINT_ORDINALS)}"
        )


def _require_owner_only_slice_shape(roles: Sequence[str], ordinals: tuple[int, ...]) -> None:
    if list(roles) != ["owner"]:
        raise ValueError("the owner-only slice scenario requires exactly one slot, the owner")
    if len(ordinals) != 1:
        raise ValueError(
            "the owner-only slice scenario requires exactly one POI NAV waypoint ordinal"
        )


#: Gate profile -> the scenario shape (slot roles, POI ordinals) its gates are written for.
_PROFILE_SHAPES: dict[str, Callable[[Sequence[str], tuple[int, ...]], None]] = {
    CERTIFIED_PROFILE: _require_certified_shape,
    OWNER_ONLY_SLICE_PROFILE: _require_owner_only_slice_shape,
}
GATE_PROFILES = tuple(_PROFILE_SHAPES)
#: Execution profile -> the sim-time factor its cadence gates expect: the
#: certified run's wall launch speedup, and lockstep's 1.
PROFILE_SIM_SPEEDUP: dict[str, float] = {
    CERTIFIED_PROFILE: SIM_SPEEDUP,
    CERTIFIED_LOCKSTEP_PROFILE: 1.0,
    OWNER_ONLY_SLICE_PROFILE: 1.0,
}
#: (gate profile, execution mode) pairs that resolve to another profile; every
#: other pair resolves to its gate profile.
_RESOLVED_PROFILES: dict[tuple[str, str], str] = {
    (CERTIFIED_PROFILE, LOCKSTEP): CERTIFIED_LOCKSTEP_PROFILE,
}


def execution_profile(gate_profile: str, mode: str) -> str:
    """The profile every profile-keyed gate table reads for a run."""
    if gate_profile not in _PROFILE_SHAPES or mode not in (WALL, LOCKSTEP):
        raise ValueError(f"no execution profile for ({gate_profile!r}, {mode!r})")
    return _RESOLVED_PROFILES.get((gate_profile, mode), gate_profile)


def require_profile_shape(
    gate_profile: object,
    roles: Sequence[str],
    ordinals: tuple[int, ...],
) -> str:
    """Return the known *gate_profile* once the scenario has its shape."""
    if type(gate_profile) is not str or gate_profile not in _PROFILE_SHAPES:
        raise ValueError(
            f"scenario gate_profile must be one of {list(GATE_PROFILES)}, got {gate_profile!r}"
        )
    _PROFILE_SHAPES[gate_profile](roles, ordinals)
    return gate_profile


__all__ = [
    "CERTIFIED_LOCKSTEP_PROFILE",
    "CERTIFIED_POI_NAV_WAYPOINT_ORDINALS",
    "CERTIFIED_PROFILE",
    "GATE_PROFILES",
    "LOCKSTEP",
    "OWNER_ONLY_SLICE_PROFILE",
    "PROFILE_SIM_SPEEDUP",
    "WALL",
    "execution_profile",
    "require_profile_shape",
]
