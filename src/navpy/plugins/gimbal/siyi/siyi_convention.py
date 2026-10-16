"""The SIYI gimbal's angle convention, shared by navpy's real client, its sim and the HITL emulator.

Axis order: the gimbal is yaw-outer, so the camera's rotation relative to the aircraft is
Rz(yaw) Ry(pitch) Rx(roll), navpy rotation sequence "ZYX" (gimbal-controller geometry.py: an outer yaw and an
inner pitch). navpy's GimbalData of a SIYI gimbal always carries this sequence.

Signs: navpy's yaw is right-positive (about the body's down axis). The SIYI wire's yaw is left-positive in 0x0E
commands and 0x0D reports (rig step 0 2026-10-14; gimbal-controller pointing yaw_sign -1), so wire yaw = -navpy yaw.
Pitch is up-positive on both. A positive 0x07 yaw speed turns the camera right, as navpy's positive yaw rate, so
rates pass unchanged. Roll keeps the wire's sign (unmeasured: the real unit reported 0.00 throughout).
"""
from __future__ import annotations

SIYI_GIMBAL_SEQ = "ZYX"


def yaw_to_wire(yaw_deg: float) -> float:
    """navpy's right-positive yaw -> the SIYI wire's left-positive yaw (0x0E)."""
    return -yaw_deg


def yaw_from_wire(yaw_deg: float) -> float:
    """The SIYI wire's left-positive yaw (0x0D) -> navpy's right-positive yaw."""
    return -yaw_deg


__all__ = ["SIYI_GIMBAL_SEQ", "yaw_from_wire", "yaw_to_wire"]
