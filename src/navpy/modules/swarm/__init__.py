"""Fleet coordination and delivery-task distribution (compatible swarm package).

This module provides:
- Task assignment and distribution
- Multi-vehicle coordination
- Fleet task communication protocols

The swarm import path and message names remain stable for compatibility.
Task allocation is not a guarantee of safe separation or successful handover.

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

