"""Neutral immutable contracts shared by final-approach source consumers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.modules.vision.models.detection_publication import DetectionPublication


@dataclass(frozen=True)
class DetectionInboxLease:
    """Identity-owned prefix of source publications from one generation."""

    generation: int
    events: tuple[DetectionPublication, ...]


@dataclass(frozen=True)
class ActiveSourceEvents:
    """Active-source publications collected at one scheduler boundary."""

    events: tuple[DetectionPublication, ...]

    @property
    def latest(self) -> Optional[DetectionPublication]:
        return self.events[-1] if self.events else None

    @property
    def discontinuity_source_names(self) -> tuple[str, ...]:
        names = (
            event.source_name
            for event in self.events
            if event.source_discontinuity
            and isinstance(event.source_name, str)
            and bool(event.source_name)
        )
        return tuple(dict.fromkeys(names))

    @property
    def has_unnamed_discontinuity(self) -> bool:
        return any(
            event.source_discontinuity
            and (
                not isinstance(event.source_name, str)
                or not event.source_name
            )
            for event in self.events
        )


@dataclass(frozen=True)
class NavSourceBatch:
    source_driven: bool
    latest_event: Optional[DetectionPublication]
    discontinuity_source_names: tuple[str, ...]
    has_unnamed_discontinuity: bool
    event_lease: DetectionInboxLease | None = None


__all__ = ["ActiveSourceEvents", "DetectionInboxLease", "NavSourceBatch"]
