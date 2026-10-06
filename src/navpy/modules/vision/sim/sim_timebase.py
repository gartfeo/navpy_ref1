"""Backward-compatible import for the shared scheduler cadence."""

from navpy.modules.common.scheduler_cadence import SchedulerCadence

SimTimebase = SchedulerCadence

__all__ = ["SimTimebase"]
