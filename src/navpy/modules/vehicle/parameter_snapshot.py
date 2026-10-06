"""Lazy MAVFTP parameter snapshot fetcher."""
from __future__ import annotations

import threading
import time
from collections.abc import Callable

from navpy.modules.vehicle._mavftp.param_pck import parse_param_pck
from navpy.modules.vehicle._mavftp.vehicle_ftp import VehicleFtp
from navpy.modules.vehicle.full_param_snapshot import FullParamSnapshot
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


class ParameterSnapshotClient:
    def __init__(
        self,
        transport: MavTransport,
        identity: VehicleIdentity,
        callbacks: CallbackRegistry,
    ) -> None:
        self._transport = transport
        self._identity = identity
        self._callbacks = callbacks
        self._lock = threading.Lock()
        self._ftp = None

    def fetch(
        self,
        *,
        with_defaults: bool = True,
        timeout: float = 30.0,
        progress_callback: Callable[
            [dict[str, object] | None],
            None,
        ] | None = None,
    ) -> FullParamSnapshot:
        with self._lock:
            if self._ftp is None:
                self._ftp = VehicleFtp(
                    self._transport, self._identity, self._callbacks,
                )
            data = self._ftp.fetch_param_pck(
                with_defaults=with_defaults,
                timeout=timeout,
                progress_callback=progress_callback,
            )
        return FullParamSnapshot(
            target_system=self._identity.target_system,
            fetched_at_unix_s=time.time(),
            with_defaults=with_defaults,
            pck=parse_param_pck(data, defaults_requested=with_defaults),
        )

    def close(self) -> None:
        with self._lock:
            if self._ftp is not None:
                self._ftp.close()
                self._ftp = None
