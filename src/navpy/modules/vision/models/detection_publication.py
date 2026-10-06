"""Immutable envelope for one source-driven detection publication."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable, Optional

from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class DetectionPublication:
    """One admitted source frame and its detection result.

    The envelope owns publication metadata and ordering. Targets are retained
    by reference because task-id allocation still belongs to the coordinator;
    pure final approach converts the chosen target to its own immutable,
    truth-redacted observation before forming a command.
    """

    detected_targets: tuple[DetectedObject, ...]
    source_timestamp_s: Optional[float]
    source_receipt_timestamp_s: Optional[float]
    source_name: Optional[str]
    source_discontinuity: bool

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "detected_targets",
            tuple(self.detected_targets),
        )
        object.__setattr__(
            self,
            "source_discontinuity",
            bool(self.source_discontinuity),
        )

    @property
    def primary_target(self) -> Optional[DetectedObject]:
        """The first target is the publication's single canonical primary."""
        return self.detected_targets[0] if self.detected_targets else None

    def with_targets(
            self,
            targets: Iterable[DetectedObject],
    ) -> "DetectionPublication":
        """Return this publication with a new canonical target ordering."""
        return replace(
            self,
            detected_targets=tuple(targets),
        )


__all__ = ["DetectionPublication"]
