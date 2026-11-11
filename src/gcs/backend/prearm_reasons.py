"""ArduPilot's current PreArm failure reasons, aged on the autopilot clock.

ArduPilot re-runs its pre-arm checks at 1 Hz and, every
``PREARM_DISPLAY_PERIOD`` seconds of *autopilot* time (AP_Arming.cpp), sends
one ``PreArm: <reason>`` STATUSTEXT for every check failing in that run. A
reason that ArduPilot no longer repeats has stopped failing. Remembering every
reason since connect let one transient reason (e.g. "Accels inconsistent"
during a restart) block launch for as long as the vehicle sat in a
non-armable mode, because SYS_STATUS stays ``failed`` there and never clears
the memory.

Reasons are therefore stamped with the autopilot's ``time_boot_ms`` and count
as current only while younger than :data:`PREARM_REASON_STALE_MS`. Ageing on
the autopilot clock makes the window independent of SITL speedup: at speedup
10 ArduPilot re-reports every 3 s of wall time, and the window shrinks with
it.
"""
from __future__ import annotations

from typing import Optional

from navpy.plugins.vehicle.ardupilot.inbound_router import boot_time_newer_or_equal

# ArduPilot AP_Arming PREARM_DISPLAY_PERIOD (30 s) in autopilot milliseconds.
ARDUPILOT_PREARM_DISPLAY_PERIOD_MS = 30_000
# A reason stays current for 2.5 report periods after it was last seen: one
# lost STATUSTEXT on a lossy link must not drop a still-failing reason (that
# would open the launch gate), while a reason missing from two consecutive
# reports has stopped failing.
PREARM_REASON_STALE_MS = ARDUPILOT_PREARM_DISPLAY_PERIOD_MS * 5 // 2

MODE_NOT_ARMABLE = "mode not armable"


class PrearmReasons:
    """PreArm reasons keyed by text, each stamped with autopilot boot time.

    The autopilot clock is fed from the vehicle's own telemetry
    (:meth:`observe_clock`). A reason received before any clock sample is
    stamped at the first sample, so it can age but never ages early. A
    reboot (clock jumps back) forgets all reasons: ArduPilot reports afresh,
    and with no reason known the launch gate stays closed on ``failed``.
    """

    def __init__(self) -> None:
        self._last_seen_ms: dict[str, Optional[int]] = {}
        self._clock_ms: Optional[int] = None

    def observe_clock(self, boot_time_ms: int) -> None:
        previous = self._clock_ms
        if previous is not None:
            if not boot_time_newer_or_equal(boot_time_ms, previous):
                return  # reordered older sample: keep the clock monotone
            if boot_time_ms < previous:
                self._last_seen_ms.clear()  # autopilot rebooted
        self._clock_ms = boot_time_ms
        for reason, stamp in self._last_seen_ms.items():
            if stamp is None:
                self._last_seen_ms[reason] = boot_time_ms

    def observe(self, reason: str) -> None:
        """Record one ``PreArm:`` reason (text after the prefix)."""
        self._last_seen_ms[reason.strip().lower()] = self._clock_ms

    def clear(self) -> None:
        self._last_seen_ms.clear()

    def current(self) -> frozenset[str]:
        """Reasons ArduPilot reported within the last stale window."""
        now = self._clock_ms
        return frozenset(
            reason
            for reason, stamp in self._last_seen_ms.items()
            if stamp is None or now is None or now - stamp <= PREARM_REASON_STALE_MS
        )

    def mode_not_armable_only(self) -> bool:
        """True when there are current reasons and all are 'mode not armable'."""
        reasons = self.current()
        return bool(reasons) and all(MODE_NOT_ARMABLE in r for r in reasons)
