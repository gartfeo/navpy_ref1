"""Composition of the re-ask that keeps an unanswered peer task."""

from __future__ import annotations

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavStateOwnership,
)
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.skills.acquire.peer_geo import PeerGeoTracker
from navpy.core.swarm.peer_task_reask import PeerTaskReask, PeerTaskReaskPorts


def compose_peer_reask(
    state: NavStateOwnership,
    observation: DetectionReviewOwnership,
    geo_tracker: PeerGeoTracker,
    approach_kind: ApproachKind,
    logger: ILogger,
) -> PeerTaskReask:
    return PeerTaskReask(
        PeerTaskReaskPorts(
            selected_poi=observation.network.selected_poi,
            can_reask=observation.retry.can_reask,
            clear_detections=state.detections.clear,
            # Start, then prime, as the peer approach's setup does.
            point_camera_at=lambda dock: (
                geo_tracker.start(dock) and geo_tracker.prime()
            ),
            stop_camera_pointing=geo_tracker.stop,
        ),
        state.navigation_task,
        state.geo_hold,
        approach_kind == ApproachKind.ORBIT,
        logger,
    )


__all__ = ["compose_peer_reask"]
