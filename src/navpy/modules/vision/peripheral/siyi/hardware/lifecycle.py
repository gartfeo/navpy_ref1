"""Idempotent synchronized lifecycle for a hardware SIYI session."""

from __future__ import annotations

import threading

from navpy.exception_groups import BaseExceptionGroup
from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.peripheral.siyi.hardware.poll_worker import (
    SiyiPollWorker,
)
from navpy.modules.vision.peripheral.siyi.hardware.polling import (
    SiyiTelemetryPoller,
)
from navpy.modules.vision.peripheral.siyi.hardware.ports import (
    SiyiSdkFactory,
)
from navpy.modules.vision.peripheral.siyi.hardware.sdk_owner import (
    SiyiEndpoint,
    SiyiSdkOwner,
)
from navpy.modules.vision.peripheral.siyi.hardware.session import SiyiSdkSession
from navpy.modules.vision.peripheral.siyi.hardware.state import SiyiReadbackStore


ATTITUDE_READY_TIMEOUT_S = 1.0
POLL_JOIN_TIMEOUT_S = 2.0


class SiyiHardwareLifecycle:
    def __init__(
        self,
        endpoint: SiyiEndpoint,
        logger: ILogger,
        sdk_factory: SiyiSdkFactory,
        session: SiyiSdkSession,
        store: SiyiReadbackStore,
        poller: SiyiTelemetryPoller,
    ) -> None:
        self._logger = logger
        self._sdk_owner = SiyiSdkOwner(
            endpoint,
            logger,
            sdk_factory,
            session,
        )
        self._store = store
        self._poll_worker = SiyiPollWorker(poller, logger)
        self._lifecycle_lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._running:
                return
            if (
                self._poll_worker.has_thread
                or self._sdk_owner.has_cleanup
            ):
                raise RuntimeError(
                    "Cannot start SIYI hardware while shutdown is incomplete"
                )
            self._store.reset_session()
            if not self._sdk_owner.open():
                return
            try:
                self._poll_worker.start()
            except BaseException as start_error:
                self._running = False
                if self._poll_worker.has_thread:
                    raise
                try:
                    self._sdk_owner.close()
                except BaseException as cleanup_error:
                    raise BaseExceptionGroup(
                        "SIYI poll startup and cleanup failed",
                        [start_error, cleanup_error],
                    ) from None
                raise
            self._running = True

            self._store.attitude_ready.wait(ATTITUDE_READY_TIMEOUT_S)
            roll, pitch_sign = self._store.set_mount_orientation()
            self._logger.info(
                "SIYI mount orientation: "
                f"roll={roll:.1f} pitch_sign={pitch_sign:+.0f}"
            )

    def stop(self) -> bool:
        with self._lifecycle_lock:
            if (
                not self._running
                and not self._poll_worker.has_thread
                and not self._sdk_owner.has_cleanup
            ):
                return True
            self._running = False
            if not self._poll_worker.stop(POLL_JOIN_TIMEOUT_S):
                return False
            self._sdk_owner.close()
            return True

    def is_connected(self) -> bool:
        return self._sdk_owner.is_connected()

    def raise_if_failed(self) -> None:
        self._poll_worker.raise_if_failed()


__all__ = [
    "ATTITUDE_READY_TIMEOUT_S",
    "POLL_JOIN_TIMEOUT_S",
    "SiyiEndpoint",
    "SiyiHardwareLifecycle",
]
