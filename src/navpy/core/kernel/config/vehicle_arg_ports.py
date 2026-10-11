"""Narrow vehicle reads used while resolving runtime arguments."""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class ParameterDefaultReader(Protocol):
    def get_param_or_default(self, name: str, default: object) -> object: ...


class MissionParameterReader(ParameterDefaultReader, Protocol):
    @property
    def mission_items_count(self) -> int: ...

    def get_mission_item(self, sequence: int) -> object | None: ...


class NavigationParameterReader(ParameterDefaultReader, Protocol):
    @property
    def min_pitch(self) -> float: ...


__all__ = [
    "NavigationParameterReader",
    "MissionParameterReader",
    "ParameterDefaultReader",
]
