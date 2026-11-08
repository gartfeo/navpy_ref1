"""Primitive frame payload admitted to the guidance command slot."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.ports.frame import GuidanceVisionFrame


@dataclass(frozen=True)
class GuidanceQueuedFrame:
    frame: GuidanceVisionFrame
    diagnostic_token: int


__all__ = ["GuidanceQueuedFrame"]
