"""Composition root for hardware SIYI capabilities."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.hardware.commands import (
    SiyiHardwareCommands,
)
from navpy.modules.vision.peripheral.siyi.hardware.lifecycle import (
    SiyiEndpoint,
    SiyiHardwareLifecycle,
)
from navpy.modules.vision.peripheral.siyi.hardware.polling import (
    SiyiTelemetryPoller,
)
from navpy.modules.vision.peripheral.siyi.hardware.ports import (
    MonotonicClock,
    SiyiSdkFactory,
    SiyiSdkPort,
)
from navpy.modules.vision.peripheral.siyi.hardware.readback import (
    SiyiHardwareReadback,
)
from navpy.modules.vision.peripheral.siyi.hardware.session import SiyiSdkSession
from navpy.modules.vision.peripheral.siyi.hardware.state import SiyiReadbackStore


@dataclass(frozen=True)
class SiyiHardwareParts:
    control: SiyiHardwareCommands
    readback: SiyiHardwareReadback
    lifecycle: SiyiHardwareLifecycle


def create_siyi_sdk(*, server_ip: str, port: int) -> SiyiSdkPort:
    from navpy.modules.vision.peripheral.siyi import SIYISDK

    return SIYISDK(server_ip=server_ip, port=port)


def build_siyi_hardware(
    data: GimbalData,
    ip: str,
    port: int,
    logger: ILogger,
    *,
    sdk_factory: SiyiSdkFactory | None,
    monotonic: MonotonicClock,
) -> SiyiHardwareParts:
    session = SiyiSdkSession()
    store = SiyiReadbackStore(data)
    selected_factory = create_siyi_sdk if sdk_factory is None else sdk_factory
    poller = SiyiTelemetryPoller(session, store, monotonic, logger)
    return SiyiHardwareParts(
        control=SiyiHardwareCommands(session, store, logger),
        readback=SiyiHardwareReadback(store, monotonic),
        lifecycle=SiyiHardwareLifecycle(
            SiyiEndpoint(ip, port),
            logger,
            selected_factory,
            session,
            store,
            poller,
        ),
    )


__all__ = ["SiyiHardwareParts", "build_siyi_hardware", "create_siyi_sdk"]
