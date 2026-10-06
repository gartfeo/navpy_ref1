"""Primitive frame payload admitted to the terminal command slot."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame


@dataclass(frozen=True)
class TerminalQueuedFrame:
    frame: TerminalVisionFrame
    diagnostic_token: int


__all__ = ["TerminalQueuedFrame"]
