"""Ordered, retryable ownership for one NavPy process runtime."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import threading
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, auto
from typing import NoReturn


CleanupAction = Callable[[], object]
RUNTIME_CLEANUP_ATTEMPTS = 2


class _CleanupRole(Enum):
    DEPENDENT = auto()
    QUIESCENCE = auto()
    INDEPENDENT = auto()


def _raise_group(message: str, errors: list[BaseException]) -> NoReturn:
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup(message, errors)
    raise BaseExceptionGroup(message, errors)


def _contains_error(root: BaseException | None, candidate: BaseException) -> bool:
    if root is None:
        return False
    if root is candidate:
        return True
    return any(
        _contains_error(child, candidate)
        for child in getattr(root, "exceptions", ())
    )


def _close_with_bounded_retry(
    resources: "RuntimeResourceStack",
) -> BaseException | None:
    """Retry retained ordinary cleanup failures once, without polling or sleep."""
    last_error: BaseException | None = None
    for _attempt in range(RUNTIME_CLEANUP_ATTEMPTS):
        try:
            resources.close()
            return None
        except BaseException as error:
            last_error = error
            if not isinstance(error, Exception):
                break
    return last_error


@dataclass(frozen=True)
class RuntimeCleanupStep:
    label: str
    action: CleanupAction
    role: _CleanupRole = _CleanupRole.DEPENDENT


class RuntimeResourceStack:
    """Release resources in reverse order and retain incomplete ownership.

    A quiescence step fences resources that may still be used by its running
    owner.  If that step fails, dependent cleanup remains owned for a later
    retry; explicitly independent cleanup can still run safely.
    """

    def __init__(self) -> None:
        self._condition = threading.Condition(threading.RLock())
        self._steps: list[RuntimeCleanupStep] = []
        self._phase = "open"
        self._close_owner: int | None = None

    def own(self, label: str, action: CleanupAction) -> None:
        self._own(RuntimeCleanupStep(label, action))

    def own_quiescence(self, label: str, action: CleanupAction) -> None:
        """Own a stop/join action that must succeed before dependents close."""
        self._own(RuntimeCleanupStep(label, action, _CleanupRole.QUIESCENCE))

    def own_independent(self, label: str, action: CleanupAction) -> None:
        """Own cleanup that is safe even when quiescence is incomplete."""
        self._own(RuntimeCleanupStep(label, action, _CleanupRole.INDEPENDENT))

    def _own(self, step: RuntimeCleanupStep) -> None:
        with self._condition:
            if self._phase != "open":
                raise RuntimeError(
                    f"cannot add a resource while runtime is {self._phase}"
                )
            self._steps.append(step)

    def close(self) -> None:
        owner = threading.get_ident()
        with self._condition:
            while self._phase == "closing":
                if self._close_owner == owner:
                    return
                self._condition.wait()
            if self._phase == "closed":
                return
            self._phase = "closing"
            self._close_owner = owner
            steps = tuple(reversed(self._steps))

        retained: list[RuntimeCleanupStep] = []
        errors: list[BaseException] = []
        quiescence_blocked = False
        for step in steps:
            if quiescence_blocked and step.role is not _CleanupRole.INDEPENDENT:
                retained.append(step)
                continue
            try:
                result = step.action()
                if result is False:
                    raise RuntimeError(
                        f"{step.label} reported incomplete cleanup"
                    )
            except BaseException as error:
                retained.append(step)
                errors.append(error)
                if step.role is _CleanupRole.QUIESCENCE:
                    quiescence_blocked = True

        with self._condition:
            self._steps = list(reversed(retained))
            self._phase = "open" if retained else "closed"
            self._close_owner = None
            self._condition.notify_all()
        if errors:
            _raise_group("runtime cleanup failed", errors)


class VehicleCloseGate:
    """Prevent transport closure while a vision publisher may still be live."""

    def __init__(self, vehicle_close: CleanupAction) -> None:
        self._vehicle_close = vehicle_close
        self._vision_safe = True

    def stop_vision(
        self,
        action: Callable[[], bool | None],
        is_quiescent: Callable[[], bool] | None = None,
    ) -> None:
        try:
            result = action()
        except BaseException:
            try:
                self._vision_safe = bool(
                    is_quiescent is not None and is_quiescent()
                )
            except BaseException:
                self._vision_safe = False
            raise
        if result is False:
            self._vision_safe = False
            raise RuntimeError("vision stop reported incomplete cleanup")
        self._vision_safe = True

    def close_vehicle(self) -> object:
        if not self._vision_safe:
            raise RuntimeError(
                "vehicle close is unsafe while vision shutdown is incomplete"
            )
        return self._vehicle_close()


class RuntimeSession:
    """Run the UI pump and guarantee complete ordered cleanup."""

    def __init__(
        self,
        *,
        resources: RuntimeResourceStack,
        start_vision: Callable[[], None],
        start_controller: Callable[[], None],
        health_check: Callable[[], None],
        ui_step: Callable[[], bool],
        sleep: Callable[[float], None],
        ready_signal: Callable[[], None] | None = None,
    ) -> None:
        self._resources = resources
        self._start_vision = start_vision
        self._start_controller = start_controller
        self._health_check = health_check
        self._ui_step = ui_step
        self._sleep = sleep
        self._ready_signal = ready_signal

    def run(self) -> int:
        primary: BaseException | None = None
        try:
            self._start_vision()
            self._start_controller()
            ready_pending = True
            while True:
                self._health_check()
                if ready_pending:
                    if self._ready_signal is not None:
                        self._ready_signal()
                    ready_pending = False
                keep_running = self._ui_step()
                self._health_check()
                if not keep_running:
                    break
                self._sleep(0.005)
        except KeyboardInterrupt:
            pass
        except BaseException as error:
            primary = error

        cleanup = _close_with_bounded_retry(self._resources)

        # Controller cleanup joins the worker.  Always harvest its final health,
        # even when an earlier UI/start failure already exists.
        late_health: BaseException | None = None
        try:
            self._health_check()
        except BaseException as error:
            if not _contains_error(primary, error) and not _contains_error(
                cleanup,
                error,
            ):
                late_health = error

        failures = [
            error for error in (primary, late_health, cleanup)
            if error is not None
        ]
        if len(failures) == 1:
            raise failures[0]
        if failures:
            _raise_group("runtime execution and cleanup failed", failures)
        return 0


__all__ = [
    "RuntimeCleanupStep",
    "RuntimeResourceStack",
    "RuntimeSession",
    "VehicleCloseGate",
]
