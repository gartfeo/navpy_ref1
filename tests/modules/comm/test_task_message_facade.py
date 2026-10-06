from __future__ import annotations

from pathlib import Path

from navpy.modules.comm.messages import available_task_msg as facade
from navpy.modules.comm.messages import task_assignment_msg as assignment
from navpy.modules.comm.messages import task_availability_msg as availability
from navpy.modules.comm.messages import task_confirmation_msg as confirmation
from navpy.modules.comm.messages import task_message_data as data


REPO_ROOT = Path(__file__).resolve().parents[3]
TASK_MESSAGE_LINE_LIMITS = {
    "src/navpy/modules/comm/messages/available_task_msg.py": 60,
    "src/navpy/modules/comm/messages/task_assignment_msg.py": 220,
    "src/navpy/modules/comm/messages/task_availability_msg.py": 220,
    "src/navpy/modules/comm/messages/task_confirmation_msg.py": 220,
    "src/navpy/modules/comm/messages/task_message_data.py": 100,
    "src/navpy/modules/comm/messages/task_message_meta.py": 60,
}


def test_available_task_facade_preserves_exact_class_identities() -> None:
    assert facade.TaskMsgData is data.TaskMsgData
    assert facade.TaskHandleMsgData is data.TaskHandleMsgData
    assert facade.TaskAssignMsgData is data.TaskAssignMsgData
    assert facade.TaskConfirmRequestMsg is confirmation.TaskConfirmRequestMsg
    assert facade.TaskConfirmResponseMsg is confirmation.TaskConfirmResponseMsg
    assert facade.AvailableTaskRequestMsg is availability.AvailableTaskRequestMsg
    assert facade.AvailableTaskResponseMsg is availability.AvailableTaskResponseMsg
    assert facade.TaskAssignRequestMsg is assignment.TaskAssignRequestMsg
    assert facade.TaskAssignResponseMsg is assignment.TaskAssignResponseMsg


def test_task_message_module_size_inventory_stays_bounded() -> None:
    for relative, limit in TASK_MESSAGE_LINE_LIMITS.items():
        lines = len((REPO_ROOT / relative).read_text(encoding="utf-8").splitlines())
        assert lines <= limit, f"{relative}: {lines} lines exceeds {limit}"
