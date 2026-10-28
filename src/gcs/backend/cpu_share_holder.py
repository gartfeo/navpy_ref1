"""The GCS backend's scheduling: CPU share holder or background
(``bounded_cpu_share.md``).

``gcs_launch.py --cpu-share-s N`` asks the backend for a share through
``GCS_CPU_SHARE_S``. Granted, the backend runs ABOVE_NORMAL on the share's
CPUs before it starts any NavPy, level with its NavPys, which inherit the
CPUs and raise themselves to ABOVE_NORMAL (at NORMAL their threads starved
the backend's reader, which fell seconds behind). Refused or not asked, the
backend runs as background. At
``expires_at`` a one-shot timer demotes the backend and every process in
every NavPy job to background, and only then releases the entry: until the
release is written the CPUs stay held, so no new holder gets them early.
The run itself keeps going. Wall only: the lifespan calls it, never the
lockstep backend.
"""

from __future__ import annotations

import logging
import os
import threading
from typing import Callable

from gcs.backend.cpu_share import Share, ShareRecord
from gcs.backend.cpu_share_background import apply_background
from gcs.backend.cpu_share_native import (
    ABOVE_NORMAL_PRIORITY_CLASS,
    BELOW_NORMAL_PRIORITY_CLASS,
    mask_of,
    native_scheduling,
)
from gcs.backend.navpy_process_instance import JOBS_LOCK, demote_open_jobs

log = logging.getLogger(__name__)

CPU_SHARE_ENV = "GCS_CPU_SHARE_S"


class StackScheduling:
    """This backend's share, if it holds one, and its expiry timer."""

    def __init__(
        self,
        record: ShareRecord | None = None,
        native: object | None = None,
        *,
        timer: Callable[..., threading.Timer] = threading.Timer,
        background: Callable[[], bool] = apply_background,
        demote_jobs: Callable[[int, int], int] = demote_open_jobs,
    ) -> None:
        self._record = record
        self._native = native
        self._timer_factory = timer
        self._background = background
        self._demote_jobs = demote_jobs
        self._share: Share | None = None
        self._timer: threading.Timer | None = None

    @property
    def holding(self) -> bool:
        return self._share is not None

    def start_from_env(self) -> str:
        raw = os.environ.get(CPU_SHARE_ENV)
        try:
            seconds = float(raw) if raw else None
        except ValueError:
            log.warning("Ignoring invalid %s=%r", CPU_SHARE_ENV, raw)
            seconds = None
        label = f"gcs slot {os.environ.get('GCS_CHAT_INDEX', '?')} ({os.getcwd()})"
        return self.start(seconds, label)

    def start(self, seconds: float | None, label: str) -> str:
        """Claim a share for *seconds* when asked, else run as background;
        returns ``holder``, ``background`` or ``normal`` (background priority
        could not be set; this stack's NavPys still start BELOW_NORMAL)."""
        native = self._native = self._native or native_scheduling()
        record = self._record = self._record or ShareRecord()
        if seconds is not None and seconds > 0:
            outcome = record.claim(label, seconds, native.efficiency_classes())  # type: ignore[attr-defined]
            if isinstance(outcome, Share):
                try:
                    native.set_own(ABOVE_NORMAL_PRIORITY_CLASS, mask_of(outcome.cpus))  # type: ignore[attr-defined]
                except OSError as error:
                    if not self._background():
                        log.error("CPU share %s not applied (%s) and this stack could not run "
                                  "as background; its CPUs stay held until this backend exits",
                                  outcome.id, error)
                        return "normal"
                    log.warning("CPU share %s not applied (%s): this stack runs as background",
                                outcome.id, error)
                    record.release(outcome)
                    return "background"
                with JOBS_LOCK:
                    self._share = outcome
                    self._timer = self._timer_factory(
                        max(0.0, outcome.expires_at - record.clock()), self.give_up, ("expired",),
                    )
                    self._timer.daemon = True
                    self._timer.start()
                log.info("CPU share %s granted: CPUs %s for %.0f s", outcome.id,
                         list(outcome.cpus), seconds)
                return "holder"
            log.warning("CPU share refused (%s): this stack runs as background", outcome)
        return "background" if self._background() else "normal"

    def give_up(self, reason: str) -> bool:
        """Demote this stack to background, then release the share.

        True when nothing is held any more. On a failed demotion the share
        stays held (its CPUs stay out of other holders' reach) until this
        backend exits.
        """
        with JOBS_LOCK:
            share = self._share
            if share is None:
                return True
            if self._timer is not None:
                self._timer.cancel()
            native = self._native
            system = native.system_mask()  # type: ignore[union-attr]
            try:
                native.set_own(BELOW_NORMAL_PRIORITY_CLASS, system)  # type: ignore[union-attr]
                jobs = self._demote_jobs(BELOW_NORMAL_PRIORITY_CLASS, system)
            except OSError as error:
                log.error("CPU share %s: demotion failed (%s); its CPUs stay held until "
                          "this backend exits", share.id, error)
                return False
            self._share = None
        released = self._record.release(share)  # type: ignore[union-attr]
        if released:
            log.info("CPU share %s ended (%s): backend and %d NavPy job(s) run as background",
                     share.id, reason, jobs)
        else:
            log.warning("CPU share %s (%s): demoted, but the record stayed busy; its CPUs "
                        "stay held until this backend exits", share.id, reason)
        return released


stack_scheduling = StackScheduling()


__all__ = ["CPU_SHARE_ENV", "StackScheduling", "stack_scheduling"]
