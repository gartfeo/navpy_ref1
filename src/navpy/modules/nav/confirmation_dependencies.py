"""Late-bound dependencies and failure policy for target confirmation."""

from __future__ import annotations

import threading
from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_ports import ConfirmationNetworkPort
from navpy.modules.nav.confirmation_manager_state import ConfirmationStatus


class ConfirmationNetworkSlot:
    """Thread-safe late-bound network dependency."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._network: Optional[ConfirmationNetworkPort] = None

    def set(self, network: Optional[ConfirmationNetworkPort]) -> None:
        with self._lock:
            self._network = network

    def get(self) -> Optional[ConfirmationNetworkPort]:
        with self._lock:
            return self._network


class FreshnessGate:
    """Fail-closed target-freshness dependency."""

    def __init__(self, logger: ILogger) -> None:
        self._logger = logger
        self._check: Callable[[Optional[int]], bool] = lambda _target_id: False

    def set_check(self, check: Callable[[Optional[int]], bool]) -> None:
        self._check = check

    def is_fresh(self, target_id: Optional[int]) -> bool:
        if target_id is None:
            return False
        try:
            return bool(self._check(target_id))
        except OSError as exc:
            self._logger.error(
                f"Freshness check raised for T{target_id}, "
                f"treating as not fresh: {exc}",
                exc,
            )
            return False


class ConfirmationFailurePolicy:
    def __init__(
        self,
        confirm_on_fail: Callable[[], bool],
        freshness: FreshnessGate,
    ) -> None:
        self._confirm_on_fail = confirm_on_fail
        self._freshness = freshness

    def status(self, target_id: Optional[int]) -> ConfirmationStatus:
        if self._confirm_on_fail() and self._freshness.is_fresh(target_id):
            return ConfirmationStatus.CONFIRMED
        return ConfirmationStatus.TIMEOUT_REJECTED


__all__ = [
    "ConfirmationFailurePolicy",
    "ConfirmationNetworkSlot",
    "FreshnessGate",
]
