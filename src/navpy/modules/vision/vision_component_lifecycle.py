"""Ordered lifecycle topology for composed vision components."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

from navpy.modules.vision.vision_stop_transaction import (
    ErrorReporter,
    VisionStopOutcome,
    VisionStopStep,
    VisionStopTransaction,
)


class VisionMountLifecyclePort(Protocol):
    @property
    def name(self) -> str: ...

    def start(self) -> None: ...

    def stop(self) -> bool: ...

    def refresh(self) -> None: ...

    def raise_if_failed(self) -> None: ...


class VisionQuiescencePort(Protocol):
    @property
    def is_quiescent(self) -> bool: ...


class VisionCoordinatorLifecyclePort(VisionQuiescencePort, Protocol):
    def start(self) -> None: ...

    def stop(self) -> bool: ...

    def refresh(self) -> None: ...

    def raise_if_failed(self) -> None: ...


class VisionPublisherLifecyclePort(VisionQuiescencePort, Protocol):
    def start(self) -> None: ...

    def stop(self) -> bool: ...

    def raise_if_failed(self) -> None: ...


@dataclass(frozen=True)
class VisionComponentStartFailure:
    error: BaseException
    rollback: VisionStopTransaction
    outcome: VisionStopOutcome


def _is_quiescent(component: VisionQuiescencePort) -> bool:
    return component.is_quiescent is True


class VisionComponentLifecycle:
    """Own component order, rollback topology, refresh, and health checks."""

    def __init__(
        self,
        mounts: Sequence[VisionMountLifecyclePort],
        publisher: VisionPublisherLifecyclePort,
        coordinator: VisionCoordinatorLifecyclePort,
    ) -> None:
        self._mounts = tuple(mounts)
        self._publisher = publisher
        self._coordinator = coordinator

    def start(
        self,
        report_error: ErrorReporter,
    ) -> VisionComponentStartFailure | None:
        attempted_readers: list[VisionStopStep] = []
        attempted_mounts: list[VisionStopStep] = []
        try:
            for mount in self._mounts:
                attempted_mounts.append(
                    VisionStopStep(f"mount '{mount.name}'", mount.stop)
                )
                mount.start()
            attempted_readers.append(VisionStopStep(
                "gimbal telemetry publisher",
                self._publisher.stop,
                lambda: _is_quiescent(self._publisher),
            ))
            self._publisher.start()
            attempted_readers.append(VisionStopStep(
                "coordinator",
                self._coordinator.stop,
                lambda: _is_quiescent(self._coordinator),
            ))
            self._coordinator.start()
        except BaseException as error:
            rollback = VisionStopTransaction(
                tuple(reversed(attempted_readers)),
                tuple(reversed(attempted_mounts)),
                report_error,
            )
            return VisionComponentStartFailure(error, rollback, rollback.run())
        return None

    def create_stop_transaction(
        self,
        report_error: ErrorReporter,
    ) -> VisionStopTransaction:
        readers = [
            VisionStopStep(
                "coordinator",
                self._coordinator.stop,
                lambda: _is_quiescent(self._coordinator),
            ),
            VisionStopStep(
                "gimbal telemetry publisher",
                self._publisher.stop,
                lambda: _is_quiescent(self._publisher),
            ),
        ]
        mounts = [
            VisionStopStep(f"mount '{mount.name}'", mount.stop)
            for mount in self._mounts
        ]
        return VisionStopTransaction(readers, mounts, report_error)

    def refresh(self) -> None:
        for mount in self._mounts:
            mount.refresh()
        self._coordinator.refresh()

    def raise_if_failed(self) -> None:
        self._publisher.raise_if_failed()
        self._coordinator.raise_if_failed()
        for mount in self._mounts:
            mount.raise_if_failed()


__all__ = [
    "VisionComponentLifecycle",
    "VisionComponentStartFailure",
    "VisionCoordinatorLifecyclePort",
    "VisionMountLifecyclePort",
    "VisionPublisherLifecyclePort",
    "VisionQuiescencePort",
]
