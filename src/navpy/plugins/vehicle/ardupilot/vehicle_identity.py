"""Immutable MAVLink vehicle identity."""
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleIdentity:
    target_system: int
    source_system: int
    source_component: int
    mav_type: int
