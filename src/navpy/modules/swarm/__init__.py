"""Swarm coordination and task distribution.

This module provides:
- Task assignment and distribution
- Multi-vehicle coordination
- Swarm task communication protocols

The swarm import path and message names remain stable for compatibility.
Task allocation is not a guarantee of safe separation, and
approach accuracy does not establish a mission outcome (e.g. docking, cargo receipt, coverage).

Usage:
    from navpy.modules.swarm import TaskActor, TaskDispatch
"""
# Public task-coordination exports
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_dispatch import TaskDispatch

__all__ = [
    "TaskActor",
    "TaskDispatch",
]

