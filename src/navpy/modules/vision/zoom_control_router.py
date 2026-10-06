"""Route target-local zoom queries and fleet-wide zoom policy commands."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Sequence

if TYPE_CHECKING:
    from navpy.modules.vision.target_identity import TargetIdentity
    from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class ZoomMember(Protocol):
    @property
    def source_name(self) -> str: ...

    @property
    def is_detection_armed(self) -> bool: ...

    @property
    def is_zoom_stable(self) -> bool: ...

    def get_zoom_result(
        self,
        obj_id: int | None = None,
    ) -> "ZoomTrackResult | None": ...

    def set_zoom_size_demand(self, enabled: bool) -> None: ...

    def freeze_terminal_zoom_at_min(self) -> bool: ...


class ZoomIdentityResolver(Protocol):
    def identity_for_task(self, task_id: int) -> "TargetIdentity | None": ...


class ZoomControlRouter:
    def __init__(
        self,
        members: Sequence[ZoomMember],
        registry: ZoomIdentityResolver,
    ) -> None:
        self._members = tuple(members)
        self._registry = registry

    @property
    def is_zoom_stable(self) -> bool:
        return all(member.is_zoom_stable for member in self._members)

    def get_zoom_result(
        self,
        obj_id: int | None = None,
    ) -> "ZoomTrackResult | None":
        if obj_id is not None:
            identity = self._registry.identity_for_task(obj_id)
            if identity is not None:
                member = self._member_for_source(identity.source_name)
                return (
                    None
                    if member is None
                    else member.get_zoom_result(identity.local_obj_id)
                )
            if len(self._members) == 1:
                return self._members[0].get_zoom_result(obj_id)
        armed = [member for member in self._members if member.is_detection_armed]
        return armed[0].get_zoom_result(None) if len(armed) == 1 else None

    def set_zoom_size_demand(self, enabled: bool) -> None:
        for member in self._members:
            member.set_zoom_size_demand(enabled)

    def freeze_terminal_zoom_at_min(self) -> bool:
        results = [
            member.freeze_terminal_zoom_at_min()
            for member in self._members
        ]
        return all(results)

    def _member_for_source(self, source_name: str) -> ZoomMember | None:
        matches = [
            member for member in self._members
            if member.source_name == source_name
        ]
        if len(matches) > 1:
            raise RuntimeError(
                f"multiple detectors report source={source_name}; routing is ambiguous"
            )
        return matches[0] if matches else None


__all__ = ["ZoomControlRouter", "ZoomIdentityResolver"]
