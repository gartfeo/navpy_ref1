"""Synchronized ownership of the active SIYI SDK session."""

from __future__ import annotations

import threading
from collections.abc import Iterator
from contextlib import contextmanager

from navpy.modules.vision.peripheral.siyi.hardware.ports import SiyiSdkPort


class SiyiSdkSession:
    """Serializes SDK calls with install/detach during lifecycle changes."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._sdk: SiyiSdkPort | None = None

    @contextmanager
    def borrow(self) -> Iterator[SiyiSdkPort | None]:
        with self._lock:
            yield self._sdk

    def install(self, sdk: SiyiSdkPort) -> None:
        with self._lock:
            if self._sdk is not None:
                raise RuntimeError("SIYI SDK session is already installed")
            self._sdk = sdk

    def detach(self) -> SiyiSdkPort | None:
        with self._lock:
            sdk = self._sdk
            self._sdk = None
            return sdk

    @property
    def has_owner(self) -> bool:
        with self._lock:
            return self._sdk is not None

    def is_connected(self) -> bool:
        with self.borrow() as sdk:
            return sdk is not None and bool(sdk.isConnected())


__all__ = ["SiyiSdkSession"]
