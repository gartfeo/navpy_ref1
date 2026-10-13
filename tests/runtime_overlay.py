"""A module-local ``rt`` stand-in for tests.

Patched onto one product module (``monkeypatch.setattr(module, "rt", ...)``),
it serves the given overrides and the real ``runtime_time`` port for
everything else, so no other module in the test process sees the fakes.
"""

from __future__ import annotations

from typing import Any

from navpy.core.kernel import runtime_time as rt


class RuntimeOverlay:
    def __init__(self, **overrides: Any) -> None:
        self.__dict__.update(overrides)

    def __getattr__(self, name: str) -> Any:
        return getattr(rt, name)


__all__ = ["RuntimeOverlay"]
