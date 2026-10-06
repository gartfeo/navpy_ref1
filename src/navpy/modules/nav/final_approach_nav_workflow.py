"""Top-level orchestration for one final-approach NAV tick."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, ContextManager, Optional, Protocol

from navpy.modules.nav.final_approach_source_contracts import NavSourceBatch
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.models.detect_data import DetectedObject


class NavPeerNotifier(Protocol):
    def submit(self) -> None: ...


class FinalApproachNavSource(Protocol):
    def find_active_poi_detection(self) -> Optional[DetectedObject]: ...

    def collect(
        self,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> NavSourceBatch: ...

    def frame_ready(
        self,
        batch: NavSourceBatch,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> bool: ...

    def consume(
        self,
        batch: NavSourceBatch,
        active_poi: DetectedObject,
    ) -> bool: ...


class FinalApproachNavCommands(Protocol):
    def dispatch(
        self,
        batch: NavSourceBatch,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> bool: ...

    def mark_no_detection(self, active_poi: DetectedObject) -> None: ...

    def mark_source_liveness_expired(
        self,
        poi: Optional[DetectedObject],
    ) -> None: ...


class FinalApproachNavRecord(Protocol):
    def commit(
        self,
        batch: NavSourceBatch,
        fresh_poi: Optional[DetectedObject],
        active_poi: DetectedObject,
    ) -> bool: ...


class FinalApproachEventPump(Protocol):
    @property
    def is_open(self) -> bool: ...

    def prepare(self) -> bool: ...

    def activate(self) -> bool: ...

    def close(self) -> None: ...

    def fail_if_source_receipt_expired(self) -> bool: ...


@dataclass(frozen=True)
class FinalApproachNavPorts:
    vehicle_mode: Callable[[], FlightMode]
    request_guided: Callable[[], None]
    mark_guided_session: Callable[[], None]
    active_poi: Callable[[], Optional[DetectedObject]]
    vision_nav_active: Callable[[], bool]
    command_liveness_failed: Callable[[], bool]


class FinalApproachNavWorkflow:
    """Coordinate one NAV tick using focused final-approach collaborators."""

    def __init__(
        self,
        ports: FinalApproachNavPorts,
        source: FinalApproachNavSource,
        commands: FinalApproachNavCommands,
        record: FinalApproachNavRecord,
        peers: NavPeerNotifier,
        event_pump: FinalApproachEventPump,
        bootstrap_fence: Callable[[], ContextManager[None]],
    ) -> None:
        self._ports = ports
        self._source = source
        self._commands = commands
        self._record = record
        self._peers = peers
        self._event_pump = event_pump
        self._bootstrap_fence = bootstrap_fence

    def act_nav(self) -> None:
        if not self.ensure_guided_session():
            if self._event_pump.is_open:
                self._event_pump.close()
            return
        if self._ports.command_liveness_failed() is True:
            self._commands.mark_source_liveness_expired(
                self._ports.active_poi()
            )
            self._event_pump.close()
            return
        if self._event_pump.is_open:
            if self._event_pump.fail_if_source_receipt_expired() is True:
                return
            self._peers.submit()
            return
        active_poi = self._ports.active_poi()
        if active_poi is None:
            return

        fresh_poi = self._source.find_active_poi_detection()
        self._peers.submit()
        pump_prepared = False
        activate_pump = False
        try:
            with self._bootstrap_fence():
                batch = self._source.collect(fresh_poi, active_poi)
                if not self._source.frame_ready(
                    batch,
                    fresh_poi,
                    active_poi,
                ):
                    if not (
                        batch.source_driven and batch.latest_event is None
                    ):
                        self._commands.mark_no_detection(active_poi)
                    return
                source_pump = (
                    batch.source_driven
                    and self._ports.vision_nav_active()
                )
                if source_pump:
                    if not self._event_pump.prepare():
                        return
                    pump_prepared = True
                if batch.source_driven:
                    if not self._source.consume(batch, active_poi):
                        return
                if not self._record.commit(
                    batch,
                    fresh_poi,
                    active_poi,
                ):
                    return
                if not self._commands.dispatch(
                    batch,
                    fresh_poi,
                    active_poi,
                ):
                    return
                activate_pump = source_pump
        finally:
            if pump_prepared and not activate_pump:
                self._event_pump.close()
        if activate_pump and not self._event_pump.activate():
            self._event_pump.close()

    def close_source_admission(self) -> None:
        self._event_pump.close()

    def ensure_guided_session(self) -> bool:
        if self._ports.vehicle_mode() != FlightMode.GUIDED:
            self._ports.request_guided()
            return False
        self._ports.mark_guided_session()
        return True


__all__ = [
    "NavPeerNotifier",
    "FinalApproachEventPump",
    "FinalApproachNavCommands",
    "FinalApproachNavPorts",
    "FinalApproachNavRecord",
    "FinalApproachNavSource",
    "FinalApproachNavWorkflow",
]
