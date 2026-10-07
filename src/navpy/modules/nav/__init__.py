"""Navigation module - High-level navigation orchestration.

This module owns ALL navigation-related functionality:
- NavController: Main navigation state machine
- ConfirmationManager: POI review and tracking
- NavState: Navigation state enumeration

Usage:
    from navpy.modules.nav import NavController, NavState
"""
from navpy.modules.nav.nav_controller import NavController, NavState
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus

__all__ = [
    "NavController",
    "NavState",
    "ConfirmationManager",
    "ConfirmationStatus",
]

