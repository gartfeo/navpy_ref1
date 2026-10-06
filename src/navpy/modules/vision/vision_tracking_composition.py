"""Compatibility translation from Vision tracker data to Navigation policy."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING

from navpy.modules.vision.vision_tracking_profile import build_tracking_profile

if TYPE_CHECKING:
    from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup


def build_tracking_config(
    device: Mapping[str, object],
    *,
    sim: bool,
) -> "GimbalTrackingSetup | None":
    """Translate Vision-owned tracker data into Navigation-owned policy."""
    from navpy.modules.navigation.gimbal_navigation_state import (
        GimbalLossPolicy,
        GimbalTrackingSetup,
    )

    parsed = build_tracking_profile(device, sim=sim)
    if parsed is None:
        return None
    default_loss = GimbalLossPolicy()
    return GimbalTrackingSetup(
        rate=parsed.rate,
        loss=GimbalLossPolicy(
            hold_sec=(
                default_loss.hold_sec
                if parsed.loss_hold_sec is None
                else parsed.loss_hold_sec
            ),
            repoint_sec=(
                default_loss.repoint_sec
                if parsed.loss_repoint_sec is None
                else parsed.loss_repoint_sec
            ),
            preserve_zoom_during_loss=(
                default_loss.preserve_zoom_during_loss
                if parsed.preserve_zoom_during_loss is None
                else parsed.preserve_zoom_during_loss
            ),
        ),
    )


__all__ = ["build_tracking_config"]
