"""Compatibility exports for focused terminal NAV owners."""

from navpy.modules.nav.terminal_command_dispatch import (
    TerminalCommandDispatch,
    TerminalCommandPorts,
)
from navpy.modules.nav.terminal_record_commit import (
    TerminalRecordCommit,
    TerminalRecordDeferral,
    TerminalRecordPorts,
)
from navpy.modules.nav.terminal_nav_workflow import (
    NavPeerNotifier,
    TerminalNavPorts,
    TerminalNavWorkflow,
)
from navpy.modules.nav.terminal_source_admission import (
    DetectionEventInbox,
    TerminalSourceAdmission,
    TerminalSourcePorts,
)
from navpy.modules.nav.terminal_source_contracts import NavSourceBatch

__all__ = [
    "DetectionEventInbox",
    "NavPeerNotifier",
    "NavSourceBatch",
    "TerminalCommandDispatch",
    "TerminalCommandPorts",
    "TerminalNavPorts",
    "TerminalNavWorkflow",
    "TerminalRecordCommit",
    "TerminalRecordDeferral",
    "TerminalRecordPorts",
    "TerminalSourceAdmission",
    "TerminalSourcePorts",
]
