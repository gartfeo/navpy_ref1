"""Primitive frame payload admitted to the final-approach command slot."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame


@dataclass(frozen=True)
class FinalApproachQueuedFrame:
    frame: FinalApproachVisionFrame
    diagnostic_token: int


__all__ = ["FinalApproachQueuedFrame"]
