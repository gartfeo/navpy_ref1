"""Bounded, strictly source-ordered history owned by one selector.

No sort or implicit eviction on overflow. Monotonic complete delivery is an
adapter precondition; the first-stamp check detects late feed startup, not
silent packet loss. Pre-start history retains only a dependency predecessor.
"""
from __future__ import annotations

from navpy.modules.vision.sim.deterministic_selection_types import (
    SelectionStream, SourceSample,
)


class SelectionHistory:
    def __init__(self, stream: SelectionStream, start_us: int, capacity: int) -> None:
        self._stream = stream
        self._start_us = start_us
        self._capacity = capacity
        self._latest: SourceSample | None = None
        self._samples: list[SourceSample] = []

    @property
    def watermark(self) -> int | None:
        return None if self._latest is None else self._latest.time_us

    def push(self, sample: SourceSample) -> str | None:
        """Accept a validated sample, or return a fault without changing state."""
        latest = self._latest
        if latest is None and sample.time_us >= self._start_us:
            return "feed_started_late"
        if latest is not None:
            if sample.time_us < latest.time_us:
                return "source_regressed"
            if sample.time_us == latest.time_us:
                return ("source_repeated" if sample.payload == latest.payload
                        else "conflicting_duplicate")
        if sample.time_us < self._start_us:
            self._samples = [] if self._stream is SelectionStream.ATTITUDE else [sample]
        else:
            if len(self._samples) >= self._capacity:
                return "capacity_exceeded"
            self._samples.append(sample)
        self._latest = sample
        return None

    def within(self, lower_us: int, upper_us: int) -> tuple[SourceSample, ...]:
        return tuple(sample for sample in self._samples
                     if lower_us <= sample.time_us < upper_us)

    def at_or_before(self, time_us: int) -> SourceSample | None:
        return next((sample for sample in reversed(self._samples)
                     if sample.time_us <= time_us), None)

    def at_or_after(self, time_us: int) -> SourceSample | None:
        return next((sample for sample in self._samples
                     if sample.time_us >= time_us), None)

    def retain_from(self, boundary_us: int) -> None:
        first = next((index for index, sample in enumerate(self._samples)
                      if sample.time_us >= boundary_us), len(self._samples))
        if self._stream is not SelectionStream.ATTITUDE:
            first = max(0, first - 1)
        del self._samples[:first]
