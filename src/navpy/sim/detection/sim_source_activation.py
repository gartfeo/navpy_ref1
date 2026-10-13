"""Lifecycle activation for simulator pose streams."""

from __future__ import annotations

from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.sim.detection.sim_runtime_ports import ResetAction


class SimSourceActivation:
    """Request scheduler-derived streams once, when detection starts."""

    def __init__(
        self,
        *,
        request_pose: ResetAction,
        request_truth: Optional[ResetAction],
    ) -> None:
        self._request_pose = request_pose
        self._request_truth = request_truth
        self._lock = rt.Lock()
        self._activated = False

    @property
    def source_driven(self) -> bool:
        return self._request_truth is not None

    def activate(self) -> None:
        with self._lock:
            if self._activated:
                return
            self._request_pose()
            if self._request_truth is not None:
                self._request_truth()
            self._activated = True


__all__ = ["SimSourceActivation"]
