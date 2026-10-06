"""Public hardware SIYI gimbal adapter."""

from __future__ import annotations

import time

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData
from navpy.modules.vision.peripheral.siyi.hardware.composition import (
    build_siyi_hardware,
)
from navpy.modules.vision.peripheral.siyi.hardware.facets import (
    SiyiHardwareControlFacet,
    SiyiHardwareLifecycleFacet,
    SiyiHardwareReadbackFacet,
)
from navpy.modules.vision.peripheral.siyi.hardware.ports import (
    MonotonicClock,
    SiyiSdkFactory,
)


class GimbalSiyi(
    SiyiHardwareLifecycleFacet,
    SiyiHardwareReadbackFacet,
    SiyiHardwareControlFacet,
    GimbalAbc,
):
    """One-field compatibility facade for the hardware SIYI capabilities."""

    def __init__(
        self,
        data: GimbalData,
        ip: str,
        port: int,
        logger: ILogger,
        *,
        sdk_factory: SiyiSdkFactory | None = None,
        monotonic: MonotonicClock = time.monotonic,
    ) -> None:
        self._parts = build_siyi_hardware(
            data,
            ip,
            port,
            logger,
            sdk_factory=sdk_factory,
            monotonic=monotonic,
        )


__all__ = ["GimbalSiyi"]
