"""Immediate rich-POI projection and primitive queue admission."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Protocol

from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    FinalApproachCommandTiming,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    FinalApproachDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
)
from navpy.modules.navigation.nav.vision_nav.queued_frame import FinalApproachQueuedFrame
from navpy.modules.navigation.nav.vision_nav.source_epoch import (
    FrameAdmission,
    SourceEpochLedger,
)
from navpy.modules.navigation.nav.vision_nav.source_time_ports import (
    ObservationOutcome,
    FinalApproachObservationSourceTimeObserver,
)
from navpy.modules.vision.models.detect_data import DetectedObject


class FinalApproachIngressCommandReset(Protocol):
    def invalidate_commands(self) -> None: ...


@dataclass(frozen=True)
class FinalApproachIngressPorts:
    lock: threading.RLock
    slot: NavigationCommandSlot
    projector: FinalApproachFrameProjector
    epochs: SourceEpochLedger
    mailbox: FinalApproachDiagnosticMailbox
    source_time: FinalApproachObservationSourceTimeObserver
    command_reset: FinalApproachIngressCommandReset


class FinalApproachIngress:
    """Never allow ``DetectedObject`` to cross into command work."""

    def __init__(self, ports: FinalApproachIngressPorts) -> None:
        self._ports = ports

    def nav(self, poi: DetectedObject) -> bool:
        ports = self._ports
        source_timestamp_s = _source_timestamp(poi.pixel.source_timestamp_s)
        provider = poi.timing.detection_now_s
        source_now_s = provider if callable(provider) else None
        receipt_timestamp_s = _source_timestamp(
            poi.timing.source_receipt_timestamp_s
        )
        receipt_provider = poi.timing.source_receipt_now_s
        timing = FinalApproachCommandTiming(
            source_timestamp_s=(
                math.nan if source_timestamp_s is None else source_timestamp_s
            ),
            source_now_s=source_now_s,
            receipt_timestamp_s=receipt_timestamp_s,
            receipt_now_s=(
                receipt_provider if callable(receipt_provider) else None
            ),
        )
        with ports.lock:
            accepted, outcome = self._nav_locked(
                poi,
                source_timestamp_s,
                timing,
            )
            if not accepted:
                ports.command_reset.invalidate_commands()
        ports.source_time.record_observation(
            source_timestamp_s=source_timestamp_s,
            source_now_s=source_now_s,
            outcome=outcome,
        )
        return accepted

    def _nav_locked(
        self,
        poi: DetectedObject,
        source_timestamp_s: float | None,
        timing: FinalApproachCommandTiming,
    ) -> tuple[bool, ObservationOutcome]:
        ports = self._ports
        visual = poi.visual_detection()
        source_name = ports.projector.source_name(visual)
        if source_name is None:
            return False, "invalid_source"
        frame = ports.projector.project(
            visual,
            ports.epochs.generation(source_name),
            getattr(poi.timing, "source_air_speed_mps", None),
        )
        if frame is None:
            outcome = "invalid_source" if source_timestamp_s is None else "invalid_frame"
            return False, outcome
        admission = ports.epochs.admit(frame)
        if admission is FrameAdmission.REGRESSION:
            return False, "regression"
        if admission is FrameAdmission.DUPLICATE:
            return True, "duplicate"
        token = ports.mailbox.put(poi, timing)
        queued = FinalApproachQueuedFrame(frame, token)
        replaced = ports.slot.replace(queued)
        if isinstance(replaced, FinalApproachQueuedFrame):
            ports.mailbox.discard(replaced.diagnostic_token)
        ports.slot.signal_pending()
        return True, "fresh"


def _source_timestamp(value: object) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        timestamp = float(value)
    except (TypeError, ValueError):
        return None
    return timestamp if math.isfinite(timestamp) else None


__all__ = [
    "FinalApproachIngress",
    "FinalApproachIngressCommandReset",
    "FinalApproachIngressPorts",
    "FinalApproachQueuedFrame",
]
