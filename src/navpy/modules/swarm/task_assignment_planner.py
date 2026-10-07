"""Pure minimum-ETA allocation for swarm task auctions.

Reported ETA/availability does not establish energy reserve or safe separation;
those require separate operational validation.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from navpy.modules.common.linear_assignment import hungarian
from navpy.modules.swarm.task_auction_models import (
    TaskOffer,
    UNAVAILABLE_ASSIGNMENT_COST,
)


class MinimumEtaAssignmentPlanner:
    """Assign distinct free peers to tasks with minimum total reported ETA."""

    def plan(
        self,
        offers: Sequence[TaskOffer],
        busy_peers: set[int],
    ) -> list[tuple[int, int]]:
        candidates = [offer for offer in offers if offer.eta_by_peer]
        peers = sorted({
            peer_id
            for offer in candidates
            for peer_id in offer.eta_by_peer
            if peer_id not in busy_peers
        })
        if not candidates or not peers:
            return []

        size = max(len(candidates), len(peers))
        costs = [[UNAVAILABLE_ASSIGNMENT_COST] * size for _ in range(size)]
        peer_index = {peer_id: index for index, peer_id in enumerate(peers)}
        for row, offer in enumerate(candidates):
            for peer_id, eta in offer.eta_by_peer.items():
                column = peer_index.get(peer_id)
                if column is not None:
                    costs[row][column] = (
                        float(eta)
                        if eta is not None
                        else UNAVAILABLE_ASSIGNMENT_COST
                    )

        columns = self._hungarian_min(costs)
        return [
            (candidates[row].task_id, peers[column])
            for row, column in enumerate(columns)
            if row < len(candidates)
            and column is not None
            and column < len(peers)
            and costs[row][column] < UNAVAILABLE_ASSIGNMENT_COST
        ]

    @staticmethod
    def _hungarian_min(costs: list[list[float]]) -> list[Optional[int]]:
        pairs = hungarian(np.array(costs, dtype=np.float32))
        columns: list[Optional[int]] = [None] * len(costs)
        for row, column in pairs:
            columns[row] = column
        return columns
