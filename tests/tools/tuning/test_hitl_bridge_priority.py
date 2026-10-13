"""The bridge process asks Windows for the HIGH priority class so Unreal on the same PC cannot starve its replies."""
from __future__ import annotations

from tools.tuning.hitl_bridge.process_priority import HIGH_PRIORITY_CLASS, raise_bridge_priority


def test_windows_raises_the_process_to_the_high_class():
    asked = []
    line = raise_bridge_priority("win32", lambda cls: asked.append(cls) or True)
    assert asked == [HIGH_PRIORITY_CLASS] == [0x80]
    assert line == "process priority HIGH"


def test_a_refused_request_is_reported_not_raised():
    assert raise_bridge_priority("win32", lambda cls: False) == "process priority unchanged (SetPriorityClass refused)"


def test_other_platforms_leave_the_priority_alone():
    def must_not_run(cls: int) -> bool:
        raise AssertionError("SetPriorityClass called off Windows")
    assert raise_bridge_priority("linux", must_not_run) == "process priority unchanged (not Windows)"
