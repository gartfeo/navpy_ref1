"""Route POI-local tracking commands across detector sources."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from typing import TYPE_CHECKING, Callable, Protocol, Sequence

if TYPE_CHECKING:
    from navpy.modules.vision.poi_identity import PoiIdentity


class TrackingMember(Protocol):
    @property
    def source_name(self) -> str: ...

    def start_tracking(self, obj_id: int) -> None: ...

    def stop_tracking(self, to_neutral: bool = True) -> None: ...


class TrackingIdentityResolver(Protocol):
    def identity_for_task(self, task_id: int) -> "PoiIdentity | None": ...


class TrackingLogger(Protocol):
    def info(self, msg: object) -> None: ...

    def warning(self, msg: object) -> None: ...


class TrackingCommandRouter:
    """Resolve task identity and apply tracking commands transactionally."""

    def __init__(
        self,
        members: Sequence[TrackingMember],
        registry: TrackingIdentityResolver,
        logger: TrackingLogger,
    ) -> None:
        self._members = tuple(members)
        self._registry = registry
        self._logger = logger

    def start_tracking(self, obj_id: int) -> None:
        identity = self._registry.identity_for_task(obj_id)
        if identity is None:
            self._safe_log(
                self._logger.warning,
                f"DetectionCoordinator: start_tracking({obj_id}) is not resolvable "
                "to a source detector; broadcasting (legacy fallback)",
            )
            self._start_transaction(
                tuple((member, obj_id) for member in self._members)
            )
            return
        member = self._member_for_source(identity.source_name)
        if member is None:
            raise RuntimeError(
                f"DetectionCoordinator: start_tracking task_id={obj_id} "
                f"resolved to source={identity.source_name} but no detector "
                "reports that source; arm aborted"
            )
        self._start_transaction(((member, identity.local_obj_id),))
        self._safe_log(
            self._logger.info,
            f"DetectionCoordinator: start_tracking task_id={obj_id} "
            f"-> {identity.source_name} obj_id={identity.local_obj_id}",
        )

    def _start_transaction(
        self,
        commands: Sequence[tuple[TrackingMember, int]],
    ) -> None:
        attempted: list[TrackingMember] = []
        try:
            for member, local_obj_id in commands:
                attempted.append(member)
                member.start_tracking(local_obj_id)
        except Exception as start_error:
            rollback_errors: list[Exception] = []
            for member in reversed(attempted):
                try:
                    member.stop_tracking(to_neutral=True)
                except Exception as rollback_error:
                    rollback_errors.append(rollback_error)
            if rollback_errors:
                raise ExceptionGroup(
                    "detector tracking start and rollback failed",
                    [start_error, *rollback_errors],
                ) from None
            raise

    def stop_tracking(self, to_neutral: bool = True) -> None:
        errors: list[Exception] = []
        for member in self._members:
            try:
                member.stop_tracking(to_neutral=to_neutral)
            except Exception as error:
                errors.append(error)
        self._safe_log(self._logger.info, "DetectionCoordinator: stop_tracking")
        if errors:
            raise ExceptionGroup("detector tracking stop failed", errors)

    def _member_for_source(self, source_name: str) -> TrackingMember | None:
        matches = [
            member for member in self._members
            if member.source_name == source_name
        ]
        if len(matches) > 1:
            raise RuntimeError(
                f"multiple detectors report source={source_name}; routing is ambiguous"
            )
        return matches[0] if matches else None

    @staticmethod
    def _safe_log(
        log: Callable[[object], None],
        message: str,
    ) -> None:
        try:
            log(message)
        except Exception:
            pass


__all__ = [
    "TrackingCommandRouter",
    "TrackingIdentityResolver",
    "TrackingLogger",
]
