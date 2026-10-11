"""Navigation command result data."""

from __future__ import annotations

from typing import Optional


class CalcData:
    """Calculation results emitted by a navigation command."""

    def __init__(
            self,
            yaw: float,
            pitch: float,
            cmd_roll: Optional[float],
            cmd_pitch: Optional[float],
            cmd_thr: Optional[float],
    ):
        self.yaw = yaw
        self.pitch = pitch
        self.cmd_roll = cmd_roll
        self.cmd_pitch = cmd_pitch
        self.cmd_thr = cmd_thr

    def __str__(self):
        return (
            f"yaw: {self.yaw}, pitch: {self.pitch}, cmd_roll: {self.cmd_roll}, "
            f"cmd_pitch: {self.cmd_pitch}, cmd_thr: {self.cmd_thr}"
        )
