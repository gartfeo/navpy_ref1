"""Several MAVLink publishers behind the vision lifecycle's one publisher port."""

from __future__ import annotations

from collections.abc import Sequence

from navpy.core.kernel.exception_groups import BaseExceptionGroup, ExceptionGroup
from navpy.core.perception.vision_component_lifecycle import (
    VisionPublisherLifecyclePort,
)


class VisionPublisherGroup:
    """Start in order; stop every member in reverse, even after a failure.

    A failed start raises at once: the vision lifecycle's rollback then calls
    ``stop``, which stops every member, started or not.
    """

    def __init__(self, publishers: Sequence[VisionPublisherLifecyclePort]) -> None:
        self._publishers = tuple(publishers)

    def start(self) -> None:
        for publisher in self._publishers:
            publisher.start()

    def stop(self) -> bool:
        quiescent = True
        errors: list[BaseException] = []
        for publisher in reversed(self._publishers):
            try:
                quiescent = publisher.stop() is True and quiescent
            except BaseException as error:  # stop the rest, then report all
                quiescent = False
                errors.append(error)
        if len(errors) == 1:
            raise errors[0]
        if errors:
            if all(isinstance(error, Exception) for error in errors):
                raise ExceptionGroup("vision publishers failed to stop", errors)
            raise BaseExceptionGroup("vision publishers failed to stop", errors)
        return quiescent

    @property
    def is_quiescent(self) -> bool:
        return all(publisher.is_quiescent is True for publisher in self._publishers)

    def raise_if_failed(self) -> None:
        for publisher in self._publishers:
            publisher.raise_if_failed()


__all__ = ["VisionPublisherGroup"]
