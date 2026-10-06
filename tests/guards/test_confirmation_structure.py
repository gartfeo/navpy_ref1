from __future__ import annotations

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIRMATION_LINE_LIMITS = {
    "src/gcs/backend/task_confirm_listener.py": 200,
    "src/gcs/backend/task_confirm_ports.py": 60,
    "src/gcs/backend/task_confirm_images.py": 160,
    "src/gcs/backend/task_confirm_rounds.py": 180,
    "src/gcs/backend/task_confirm_transport.py": 150,
    "src/gcs/backend/task_confirm_uid.py": 80,
    "src/navpy/modules/nav/confirmation_ports.py": 140,
    "src/navpy/modules/nav/confirmation_round_state.py": 260,
    "src/navpy/modules/nav/confirmation_response_state.py": 180,
    "src/navpy/modules/nav/confirmation_send_gate.py": 120,
    "src/navpy/modules/swarm/task_ports.py": 80,
}


def _class_shape(relative: str, class_name: str) -> tuple[int, int]:
    source = (REPO_ROOT / relative).read_text(encoding="utf-8")
    tree = ast.parse(source)
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    methods = sum(
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        for node in owner.body
    )
    return owner.end_lineno - owner.lineno + 1, methods


def test_confirmation_owner_size_inventory_stays_bounded() -> None:
    for relative, limit in CONFIRMATION_LINE_LIMITS.items():
        lines = len((REPO_ROOT / relative).read_text(encoding="utf-8").splitlines())
        assert lines <= limit, f"{relative}: {lines} lines exceeds {limit}"


def test_public_confirmation_facades_stay_within_behavior_budgets() -> None:
    listener_lines, listener_methods = _class_shape(
        "src/gcs/backend/task_confirm_listener.py",
        "TaskConfirmListener",
    )
    round_lines, round_methods = _class_shape(
        "src/navpy/modules/nav/confirmation_round_state.py",
        "ConfirmationRoundState",
    )
    assert listener_lines <= 200 and listener_methods <= 15
    assert round_lines <= 200 and round_methods <= 15
