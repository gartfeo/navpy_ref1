"""Retry only vision components that have not reached quiescence."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence


StopAction = Callable[[], bool]
ErrorReporter = Callable[[str], None]
QuiescenceProbe = Callable[[], bool]


class VisionStopTimeout(RuntimeError):
    """A component reported that it remains live without raising an error."""


@dataclass(frozen=True)
class VisionStopStep:
    label: str
    action: StopAction
    is_quiescent: QuiescenceProbe | None = None


@dataclass(frozen=True)
class VisionStopOutcome:
    complete: bool
    quiescent: bool
    errors: tuple[BaseException, ...]


def _run_steps(
    steps: Sequence[VisionStopStep],
    report_error: ErrorReporter,
) -> tuple[tuple[VisionStopStep, ...], tuple[BaseException, ...]]:
    pending: list[VisionStopStep] = []
    errors: list[BaseException] = []
    for step in steps:
        try:
            result = step.action()
        except BaseException as stop_error:
            pending.append(step)
            errors.append(stop_error)
            report_error(f"Failed to stop {step.label}: {stop_error}")
            continue
        if result is not True:
            pending.append(step)
            error = _incomplete_stop_error(step, result)
            errors.append(error)
            report_error(str(error))
    return tuple(pending), tuple(errors)


def _step_is_quiescent(step: VisionStopStep) -> bool:
    if step.is_quiescent is None:
        return False
    try:
        return bool(step.is_quiescent())
    except BaseException:
        return False


def _run_prerequisites(
    steps: Sequence[VisionStopStep],
    report_error: ErrorReporter,
) -> tuple[
    tuple[VisionStopStep, ...],
    tuple[VisionStopStep, ...],
    tuple[BaseException, ...],
]:
    blocking: list[VisionStopStep] = []
    cleanup: list[VisionStopStep] = []
    errors: list[BaseException] = []
    for step in steps:
        try:
            result = step.action()
        except BaseException as stop_error:
            target = cleanup if _step_is_quiescent(step) else blocking
            target.append(step)
            errors.append(stop_error)
            report_error(f"Failed to stop {step.label}: {stop_error}")
            continue
        if result is not True:
            blocking.append(step)
            error = _incomplete_stop_error(step, result)
            errors.append(error)
            report_error(str(error))
            continue
        if not _step_is_quiescent(step):
            blocking.append(step)
            timeout = VisionStopTimeout(
                f"Failed to stop {step.label}: quiescence was not confirmed"
            )
            errors.append(timeout)
            report_error(str(timeout))
    return tuple(blocking), tuple(cleanup), tuple(errors)


def _incomplete_stop_error(
    step: VisionStopStep,
    result: object,
) -> BaseException:
    if result is False:
        return VisionStopTimeout(
            f"Failed to stop {step.label}: stop timed out"
        )
    return TypeError(
        f"Failed to stop {step.label}: stop() must return bool"
    )


class VisionStopTransaction:
    """Retain failed stop steps while retiring successful ownership."""

    def __init__(
        self,
        prerequisites: Sequence[VisionStopStep],
        dependents: Sequence[VisionStopStep],
        report_error: ErrorReporter,
    ) -> None:
        self._pending_prerequisites = tuple(prerequisites)
        self._pending_dependents = tuple(dependents)
        self._pending_cleanup: tuple[VisionStopStep, ...] = ()
        self._report_error = report_error

    @property
    def is_complete(self) -> bool:
        return (
            not self._pending_prerequisites
            and not self._pending_dependents
            and not self._pending_cleanup
        )

    def run(self) -> VisionStopOutcome:
        cleanup, cleanup_errors = _run_steps(
            self._pending_cleanup,
            self._report_error,
        )
        self._pending_cleanup = cleanup
        prerequisites, deferred_cleanup, prerequisite_errors = _run_prerequisites(
            self._pending_prerequisites,
            self._report_error,
        )
        self._pending_prerequisites = prerequisites
        self._pending_cleanup += deferred_cleanup
        if prerequisites:
            return VisionStopOutcome(
                False,
                False,
                cleanup_errors + prerequisite_errors,
            )
        dependents, dependent_errors = _run_steps(
            self._pending_dependents,
            self._report_error,
        )
        self._pending_dependents = dependents
        quiescent = not dependents
        return VisionStopOutcome(
            quiescent and not self._pending_cleanup,
            quiescent,
            cleanup_errors + prerequisite_errors + dependent_errors,
        )


__all__ = [
    "VisionStopOutcome",
    "VisionStopStep",
    "VisionStopTimeout",
    "VisionStopTransaction",
]
