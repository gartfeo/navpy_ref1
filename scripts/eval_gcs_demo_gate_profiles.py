"""Gate profiles a demo scenario names, and the scenario shape each requires."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from scripts.eval_gcs_demo_constants import SIM_SPEEDUP

CERTIFIED_PROFILE = "certified"
OWNER_ONLY_SLICE_PROFILE = "owner_only_slice"
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
#: Gate profile -> the sim-time factor its cadence gates expect: the certified
#: run's wall launch speedup, and lockstep's 1 for the slice.
PROFILE_SIM_SPEEDUP: dict[str, float] = {
    CERTIFIED_PROFILE: SIM_SPEEDUP,
    OWNER_ONLY_SLICE_PROFILE: 1.0,
}


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
    "CERTIFIED_POI_NAV_WAYPOINT_ORDINALS",
    "CERTIFIED_PROFILE",
    "GATE_PROFILES",
    "OWNER_ONLY_SLICE_PROFILE",
    "PROFILE_SIM_SPEEDUP",
    "require_profile_shape",
]
