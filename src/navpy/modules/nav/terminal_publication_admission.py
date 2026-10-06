"""Shared source-publication continuity admission for terminal NAV."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.target_identity import get_target_task_id


@dataclass(frozen=True)
class TerminalPublicationAdmissionPorts:
    clear_discontinuity: Callable[[tuple[str, ...]], None]
    mark_failed: Callable[[], None]
    logger: ILogger


class TerminalPublicationAdmission:
    """Apply identical continuity policy to batch and live events."""

    def __init__(self, ports: TerminalPublicationAdmissionPorts) -> None:
        self._ports = ports

    def admit_batch(
        self,
        publication: DetectionPublication | None,
        active_target: DetectedObject,
        *,
        discontinuity_source_names: tuple[str, ...],
        has_unnamed_discontinuity: bool,
        commit: Callable[[], bool],
    ) -> bool:
        return self._admit(
            publication,
            active_target,
            discontinuity_source_names=discontinuity_source_names,
            has_unnamed_discontinuity=has_unnamed_discontinuity,
            commit=commit,
        )

    def admit_event(
        self,
        publication: DetectionPublication,
        active_target: DetectedObject | None,
    ) -> bool:
        discontinuous = bool(publication.source_discontinuity)
        source_name = publication.source_name
        named = discontinuous and isinstance(source_name, str) and bool(
            source_name
        )
        return self._admit(
            publication,
            active_target,
            discontinuity_source_names=(source_name,) if named else (),
            has_unnamed_discontinuity=discontinuous and not named,
            commit=lambda: True,
        )

    def _admit(
        self,
        publication: DetectionPublication | None,
        active_target: DetectedObject | None,
        *,
        discontinuity_source_names: tuple[str, ...],
        has_unnamed_discontinuity: bool,
        commit: Callable[[], bool],
    ) -> bool:
        if discontinuity_source_names:
            self._ports.clear_discontinuity(discontinuity_source_names)
        if not commit():
            return False
        if has_unnamed_discontinuity:
            self._fail_unnamed_discontinuity(active_target)
            return False
        if publication is None:
            return False
        return True

    def _fail_unnamed_discontinuity(
        self,
        active_target: DetectedObject | None,
    ) -> None:
        self._ports.mark_failed()
        identity = (
            "active_target=missing"
            if active_target is None
            else (
                f"task={get_target_task_id(active_target)} "
                f"obj={active_target.identity.obj_id}"
            )
        )
        try:
            self._ports.logger.info(
                f"NAV_FAIL: discontinuity_source_missing {identity}",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
        except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
            return


__all__ = [
    "TerminalPublicationAdmission",
    "TerminalPublicationAdmissionPorts",
]
