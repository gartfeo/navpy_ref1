"""Detector start/stop transactions."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

from typing import Protocol, Sequence

from navpy.modules.vision.detector_ports import DetectorFleetMember


class LifecycleLogger(Protocol):
    def info(self, message: str) -> None: ...


class DetectorLifecycleFleet:
    def __init__(
        self,
        members: Sequence[DetectorFleetMember],
        logger: LifecycleLogger,
    ) -> None:
        self._members = tuple(members)
        self._logger = logger
        self._quiescent: dict[int, bool] = {
            id(member): False for member in self._members
        }
        self._complete: dict[int, bool] = {
            id(member): False for member in self._members
        }

    @property
    def members(self) -> tuple[DetectorFleetMember, ...]:
        return self._members

    def start(self) -> None:
        attempted: list[DetectorFleetMember] = []
        try:
            for member in self._members:
                self._quiescent[id(member)] = False
                self._complete[id(member)] = False
                attempted.append(member)
                member.start()
        except BaseException as start_error:
            rollback_errors: list[BaseException] = []
            for member in reversed(attempted):
                try:
                    stopped = member.stop()
                    if stopped is not True:
                        if stopped is not False:
                            rollback_errors.append(TypeError(
                                "detector fleet member stop() must return bool"
                            ))
                        else:
                            rollback_errors.append(TimeoutError(
                                "detector fleet member did not stop during rollback"
                            ))
                        self._quiescent[id(member)] = _reports_quiescent(member)
                    else:
                        self._quiescent[id(member)] = True
                        self._complete[id(member)] = True
                except BaseException as rollback_error:
                    rollback_errors.append(rollback_error)
                    self._quiescent[id(member)] = _reports_quiescent(member)
            if rollback_errors:
                failures = [start_error, *rollback_errors]
                if all(isinstance(error, Exception) for error in failures):
                    raise ExceptionGroup(
                        "detector fleet start and rollback failed",
                        failures,
                    ) from None
                raise BaseExceptionGroup(
                    "detector fleet start and rollback failed",
                    failures,
                ) from None
            raise
        self._safe_info(
            f"DetectionCoordinator: Started {len(self._members)} detectors"
        )

    @property
    def is_quiescent(self) -> bool:
        return all(self._quiescent.values())

    def stop(self) -> bool:
        errors: list[BaseException] = []
        for member in self._members:
            if self._complete[id(member)]:
                continue
            try:
                stopped = member.stop()
            except BaseException as error:
                errors.append(error)
                self._quiescent[id(member)] = _reports_quiescent(member)
                continue
            if stopped is not True:
                self._quiescent[id(member)] = _reports_quiescent(member)
                if stopped is not False:
                    errors.append(TypeError(
                        "detector fleet member stop() must return bool"
                    ))
                continue
            self._quiescent[id(member)] = True
            self._complete[id(member)] = True
        if len(errors) == 1:
            raise errors[0]
        if errors:
            if all(isinstance(error, Exception) for error in errors):
                raise ExceptionGroup("multiple detector stops failed", errors)
            raise BaseExceptionGroup("multiple detector stops failed", errors)
        self._safe_info("DetectionCoordinator: Stopped all detectors")
        return self.is_quiescent

    def raise_if_failed(self) -> None:
        for member in self._members:
            member.raise_if_failed()

    def _safe_info(self, message: str) -> None:
        try:
            self._logger.info(message)
        except BaseException:
            pass


def _reports_quiescent(member: DetectorFleetMember) -> bool:
    try:
        return member.is_quiescent is True
    except BaseException:
        return False


__all__ = ["DetectorLifecycleFleet"]
