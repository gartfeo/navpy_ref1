"""Run this process as background work beside the CPU shares
(``bounded_cpu_share.md``).

Background is BELOW_NORMAL, which children inherit, on the CPUs outside the
valid shares. At the earliest avoided share's ``expires_at`` a one-shot
timer recomputes the mask from the shares valid then. Priority is the
guarantee: a share's threads outrank all background work, so a priority
that cannot be set is an error and ``apply`` says so. Affinity is best
effort: a child keeps the mask it started with, and a share claimed later is
not seen until the next recompute (nothing polls the record).
"""

from __future__ import annotations

import logging
import threading
from typing import Callable

from gcs.backend.cpu_share import ShareRecord
from gcs.backend.cpu_share_native import BELOW_NORMAL_PRIORITY_CLASS, mask_of, native_scheduling

_LOG = logging.getLogger(__name__)


class BackgroundScheduling:
    """This process's background state and its one pending recompute."""

    def __init__(
        self,
        record: ShareRecord | None = None,
        native: object | None = None,
        timer: Callable[..., threading.Timer] = threading.Timer,
    ) -> None:
        self._record = record
        self._native = native
        self._timer_factory = timer
        self._timer: threading.Timer | None = None
        self._lock = threading.Lock()

    def apply(self) -> bool:
        """Set background priority and mask now; True when background
        priority is in force (the mask is best effort)."""
        native = self._native = self._native or native_scheduling()
        record = self._record = self._record or ShareRecord()
        system = native.system_mask()  # type: ignore[attr-defined]
        if not system:
            return False
        avoided, next_expiry = record.avoided()
        # Every CPU in a share: background keeps priority only.
        mask = system & ~mask_of(avoided) or system
        try:
            native.set_own(BELOW_NORMAL_PRIORITY_CLASS, mask)  # type: ignore[attr-defined]
        except OSError as error:
            if not _below_normal(native):
                _LOG.error("Could not run as background (%s): this process keeps its priority",
                           error)
                return False
            _LOG.warning("Background CPUs not set (%s): background priority only", error)
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
            self._timer = None
            if next_expiry is not None:
                self._timer = self._timer_factory(max(0.0, next_expiry - record.clock()), self.apply)
                self._timer.daemon = True
                self._timer.start()
        return True

    def cancel(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None


def _below_normal(native: object) -> bool:
    try:
        return native.own()[0] == BELOW_NORMAL_PRIORITY_CLASS  # type: ignore[attr-defined]
    except OSError:
        return False


_BACKGROUND = BackgroundScheduling()


def apply_background() -> bool:
    """Run this process, and the children it starts from now on, as
    background; True when background priority is in force."""
    return _BACKGROUND.apply()


__all__ = ["BackgroundScheduling", "apply_background"]
