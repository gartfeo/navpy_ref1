"""Closest-approach records from GCS-started sim NavPys (sim only).

docs/design/gcs_sim_closest_approach.md. The NavPy output reader offers every
line; an exact record whose vehicle is the instance's own ``sys_id`` is kept
here, per ``sys_id``, independent of the process instance, so a stop or a
restart before the next status broadcast does not lose it. Display only.
"""

from __future__ import annotations

import logging
from collections import deque
from typing import Any

from navpy.core.kernel import runtime_time as rt
from navpy.sim.world.closest_approach_record import parse, to_mapping

log = logging.getLogger(__name__)

# Records kept per vehicle: a demo has a few approaches per UAV, and the
# browser consumes each one once (it keeps its own marks).
SIM_CLOSEST_APPROACH_KEEP = 20


class SimClosestApproachStore:
    def __init__(self, boot_id: str | None = None) -> None:
        # Namespaces keys across backend restarts (generations restart at 1).
        self._boot_id = boot_id or rt.token_urlsafe(6)
        self._lock = rt.Lock()
        self._records: dict[int, deque[dict[str, Any]]] = {}

    def offer(self, sys_id: int, generation: int, line: str) -> bool:
        """Keep ``line`` if it is this vehicle's record; never raises."""
        record = parse(line)
        if record is None:
            return False
        if record.vehicle != sys_id:
            log.warning(
                "closest-approach record for vehicle %d dropped: "
                "it came from sys_id %d's NavPy",
                record.vehicle,
                sys_id,
            )
            return False
        entry = to_mapping(record)
        entry["key"] = f"{self._boot_id}:{generation}:{record.episode}"
        entry["sys_id"] = sys_id
        with self._lock:
            records = self._records.setdefault(
                sys_id, deque(maxlen=SIM_CLOSEST_APPROACH_KEEP),
            )
            records.append(entry)
        return True

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                dict(entry)
                for sys_id in sorted(self._records)
                for entry in self._records[sys_id]
            ]


__all__ = ["SIM_CLOSEST_APPROACH_KEEP", "SimClosestApproachStore"]
