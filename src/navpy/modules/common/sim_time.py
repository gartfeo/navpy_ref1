"""Compatibility import for the frequency-only scheduler adapter.

New code must import :class:`SchedulerCadence` from ``scheduler_cadence``.
The historical name remains importable for callers outside this repository.
"""

from navpy.modules.common.scheduler_cadence import SchedulerCadence

SimTimebase = SchedulerCadence

__all__ = ["SimTimebase"]
