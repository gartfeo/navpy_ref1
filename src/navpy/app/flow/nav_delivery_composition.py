"""Composition of the delivery product's calls into core."""

from __future__ import annotations

from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavStateOwnership,
    PoiMissionOwnership,
)
from navpy.core.flow.navigation_decision import NavStateDecision
from navpy.products.delivery.flow import DeliveryFlow
from navpy.products.delivery.poi_status_decision import ConfirmationStatusDecision


def compose_delivery_flow(
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    observation: DetectionReviewOwnership,
    approach: NavStateDecision,
    review: ConfirmationStatusDecision,
) -> DeliveryFlow:
    return DeliveryFlow(
        state.phase,
        approach,
        review,
        observation.status,
        lambda: poi.confirmation_manager.clear_active_poi(),
    )


__all__ = ["compose_delivery_flow"]
