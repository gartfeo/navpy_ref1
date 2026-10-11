from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
TASK_MESSAGE_LINE_LIMITS = {
    "src/navpy/core/link/task_assignment_msg.py": 220,
    "src/navpy/core/link/task_availability_msg.py": 220,
    "src/navpy/core/link/task_confirmation_msg.py": 220,
    "src/navpy/core/link/task_message_data.py": 100,
    "src/navpy/core/link/task_message_meta.py": 60,
}


def test_task_message_module_size_inventory_stays_bounded() -> None:
    for relative, limit in TASK_MESSAGE_LINE_LIMITS.items():
        lines = len((REPO_ROOT / relative).read_text(encoding="utf-8").splitlines())
        assert lines <= limit, f"{relative}: {lines} lines exceeds {limit}"
