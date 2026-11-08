"""Every GUIDED NavPy asks for is told to recovery first (navpy_recovery.md, step 3 ownership; found in step 9 SITL).

In SITL (2026-11-09) an injected NAV cycle fault mid-approach only fenced commands: NAV entry, the self-detected
approach and the peer approach asked for GUIDED through ``approach.commands.request_guided()`` directly, so recovery
never learned the GUIDED was NavPy's, never owned it, and so never held in LOITER or went to RTL; the final approach
asks only when the mode is not GUIDED already. ``request_guided_owned`` tells recovery, then asks.
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

FLOW = Path(__file__).resolve().parents[3] / "src" / "navpy" / "app" / "flow"


def _raw_requests(path: Path) -> list[str]:
    """``<x>.commands.request_guided()`` calls outside ``request_guided_owned``."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    hits = []
    for function in [None, *[n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]]:
        if function is not None and function.name == "request_guided_owned":
            continue
        for node in ast.walk(function or tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "request_guided" and isinstance(node.func.value, ast.Attribute)
                    and node.func.value.attr == "commands"
                    and not _inside_owned(tree, node)):
                hits.append(f"{path.name}:{node.lineno}")
    return sorted(set(hits))


def _inside_owned(tree: ast.Module, target: ast.AST) -> bool:
    for function in ast.walk(tree):
        if isinstance(function, ast.FunctionDef) and function.name == "request_guided_owned":
            if any(node is target for node in ast.walk(function)):
                return True
    return False


def test_no_composition_asks_for_guided_without_telling_recovery():
    hits = [hit for path in sorted(FLOW.glob("*.py")) for hit in _raw_requests(path)]
    assert hits == [], hits


def test_request_guided_owned_tells_recovery_before_asking():
    from navpy.app.flow.nav_decision_composition import request_guided_owned
    from navpy.core.flow.nav_ownership import ModeObservation
    from navpy.core.flow.nav_recovery_ports import RecoveryHold
    from navpy.core.ports.flight_mode import FlightMode

    order = []
    hold = RecoveryHold(lambda: order.append("observed") or ModeObservation(7, FlightMode.AUTO, True, True))
    approach = SimpleNamespace(commands=Mock(request_guided=Mock(side_effect=lambda: order.append("asked"))))
    request_guided_owned(SimpleNamespace(recovery_hold=hold), approach)
    assert order == ["observed", "asked"]
    assert hold.take_guided_request() == ModeObservation(7, FlightMode.AUTO, True, True)
