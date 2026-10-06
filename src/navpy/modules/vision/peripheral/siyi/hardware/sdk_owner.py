"""Transactional ownership of one connected SIYI SDK payload session."""

from __future__ import annotations

from dataclasses import dataclass
from typing import NoReturn

from navpy.exception_groups import BaseExceptionGroup
from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.peripheral.siyi.hardware.bounded_sdk import (
    BoundedSiyiSdk,
)
from navpy.modules.vision.peripheral.siyi.hardware.ports import (
    SiyiSdkFactory,
    SiyiSdkPort,
)
from navpy.modules.vision.peripheral.siyi.hardware.session import SiyiSdkSession
from navpy.modules.vision.peripheral.siyi.hardware.shutdown import (
    require_acknowledgement,
    shutdown_payload,
)


def _raise_start_cleanup_failure(
    message: str,
    start_error: BaseException,
    cleanup_error: BaseException,
) -> NoReturn:
    raise BaseExceptionGroup(message, [start_error, cleanup_error]) from None


@dataclass(frozen=True)
class SiyiEndpoint:
    ip: str
    port: int


class SiyiSdkOwner:
    """Own pending and installed SDK state through retryable payload cleanup."""

    def __init__(
        self,
        endpoint: SiyiEndpoint,
        logger: ILogger,
        sdk_factory: SiyiSdkFactory,
        session: SiyiSdkSession,
    ) -> None:
        self._endpoint = endpoint
        self._logger = logger
        self._sdk_factory = sdk_factory
        self._session = session
        self._pending_sdk: SiyiSdkPort | None = None
        self._pending_requires_stop = False

    @property
    def has_cleanup(self) -> bool:
        return self._pending_sdk is not None or self._session.has_owner

    def is_connected(self) -> bool:
        return self._session.is_connected()

    def open(self) -> bool:
        if self.has_cleanup:
            raise RuntimeError("SIYI SDK ownership is already installed")
        try:
            sdk = BoundedSiyiSdk(
                self._sdk_factory(
                    server_ip=self._endpoint.ip,
                    port=self._endpoint.port,
                )
            )
        except OSError as error:
            self._log_open_error("create SIYI gimbal SDK for", error)
            return False
        self._pending_sdk = sdk
        try:
            connected = sdk.connect()
        except OSError as error:
            self._cleanup_failed_start(
                "SIYI connection and cleanup failed",
                error,
            )
            self._log_open_error("connect to SIYI gimbal at", error)
            return False
        if not connected:
            self._retire_pending_sdk()
            self._log_open_error("connect to SIYI gimbal at")
            return False

        self._pending_requires_stop = True
        try:
            self._initialize_payload(sdk)
        except BaseException as error:
            self._cleanup_failed_start("SIYI setup and cleanup failed", error)
            raise
        try:
            self._session.install(sdk)
        except BaseException as error:
            self._cleanup_failed_start(
                "SIYI session install and cleanup failed",
                error,
            )
            raise
        self._pending_sdk = None
        self._pending_requires_stop = False
        return True

    def close(self) -> None:
        if self._pending_sdk is not None:
            self._retire_pending_sdk()
        with self._session.borrow() as sdk:
            if sdk is None:
                return
            shutdown_payload(sdk)
            self._session.detach()

    def _initialize_payload(self, sdk: SiyiSdkPort) -> None:
        require_acknowledgement("center", sdk.requestCenterGimbal())
        require_acknowledgement("lock-mode", sdk.requestLockMode())
        require_acknowledgement("zoom-reset", sdk.requestAbsoluteZoom(1))
        require_acknowledgement("autofocus", sdk.requestAutoFocus())
        self._logger.info("SIYI gimbal connected, Lock Mode enabled")

    def _cleanup_failed_start(
        self,
        message: str,
        start_error: BaseException,
    ) -> None:
        try:
            self._retire_pending_sdk()
        except BaseException as cleanup_error:
            _raise_start_cleanup_failure(message, start_error, cleanup_error)

    def _retire_pending_sdk(self) -> None:
        sdk = self._pending_sdk
        if sdk is None:
            return
        if self._pending_requires_stop:
            shutdown_payload(sdk)
        else:
            disconnected = sdk.disconnect()
            if disconnected is False:
                raise TimeoutError(
                    "SIYI SDK threads did not stop during disconnect"
                )
        self._pending_sdk = None
        self._pending_requires_stop = False

    def _log_open_error(
        self,
        action: str,
        error: OSError | None = None,
    ) -> None:
        suffix = "" if error is None else f": {error}"
        self._logger.error(
            f"Failed to {action} "
            f"{self._endpoint.ip}:{self._endpoint.port}{suffix}"
        )


__all__ = ["SiyiEndpoint", "SiyiSdkOwner"]
