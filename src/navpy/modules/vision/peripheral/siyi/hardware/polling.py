"""Bounded SIYI attitude and zoom polling."""

from __future__ import annotations

import threading

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.peripheral.siyi.hardware.ports import MonotonicClock
from navpy.modules.vision.peripheral.siyi.hardware.session import SiyiSdkSession
from navpy.modules.vision.peripheral.siyi.hardware.state import SiyiReadbackStore


SIYI_POLL_PERIOD_S = 1.0 / 20.0


class SiyiTelemetryPoller:
    def __init__(
        self,
        session: SiyiSdkSession,
        store: SiyiReadbackStore,
        monotonic: MonotonicClock,
        logger: ILogger,
    ) -> None:
        self._session = session
        self._store = store
        self._monotonic = monotonic
        self._logger = logger

    def run(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            started_s = self._monotonic()
            self.poll_once()
            elapsed_s = self._monotonic() - started_s
            stop_event.wait(max(0.0, SIYI_POLL_PERIOD_S - elapsed_s))

    def poll_once(self) -> None:
        with self._session.borrow() as sdk:
            if sdk is None:
                return
            attitude_sample = None
            try:
                attitude_sample = sdk.getAttitudeSample()
            except OSError as exc:
                self._logger.warning(f"SIYI attitude poll error: {exc}")

            zoom_sample = None
            zoom_received = False
            try:
                zoom_sample = sdk.getCurrentZoomLevelSample()
            except OSError as exc:
                self._logger.warning(f"SIYI zoom poll error: {exc}")
            else:
                zoom_received = True
            self._store.commit_poll(
                attitude_sample,
                zoom_sample,
                self._monotonic(),
            )
            if zoom_received:
                sdk.requestCurrentZoomLevel()


__all__ = ["SIYI_POLL_PERIOD_S", "SiyiTelemetryPoller"]
