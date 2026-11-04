"""Lazy MAVFTP parameter snapshot fetcher."""
from __future__ import annotations

import logging

from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot._mavftp.param_pck import (
    ParamPck,
    ParamPckError,
    parse_param_pck,
)
from navpy.plugins.vehicle.ardupilot._mavftp.vehicle_ftp import VehicleFtp
from navpy.plugins.vehicle.ardupilot.full_param_snapshot import FullParamSnapshot
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_subscriptions import CallbackRegistry
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity

log = logging.getLogger(__name__)

# ArduPilot builds param.pck again for each read (AP_Filesystem_Param.cpp).
# With defaults, a record holds its default only while the value differs
# from it, so a value that crosses its default during a lossy read moves
# every later record. Gap reads then join bytes from two layouts and the
# blob is torn. Fetch the whole file again, a bounded number of times,
# within the one timeout. The root fix, a snapshot per read, is todo 272.
TORN_PCK_FETCHES = 3


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
        self._lock = rt.Lock()
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
            pck = self._fetch_whole(
                with_defaults=with_defaults,
                timeout=timeout,
                progress_callback=progress_callback,
            )
        return FullParamSnapshot(
            target_system=self._identity.target_system,
            fetched_at_unix_s=rt.time(),
            with_defaults=with_defaults,
            pck=pck,
        )

    def _fetch_whole(
        self,
        *,
        with_defaults: bool,
        timeout: float,
        progress_callback: Callable[[dict[str, object] | None], None] | None,
    ) -> ParamPck:
        # The refetches share the caller's timeout; they do not each get it again.
        deadline = rt.monotonic() + timeout
        for attempt in range(1, TORN_PCK_FETCHES + 1):
            data = self._ftp.fetch_param_pck(
                with_defaults=with_defaults,
                timeout=max(0.0, deadline - rt.monotonic()),
                progress_callback=progress_callback,
            )
            try:
                return parse_param_pck(data, defaults_requested=with_defaults)
            except ParamPckError as exc:
                if attempt == TORN_PCK_FETCHES or rt.monotonic() >= deadline:
                    raise
                log.warning(
                    "param.pck for sys_id=%d is torn (%s); fetching again "
                    "(%d of %d)", self._identity.target_system, exc,
                    attempt + 1, TORN_PCK_FETCHES,
                )
        raise AssertionError("unreachable")

    def close(self) -> None:
        with self._lock:
            if self._ftp is not None:
                self._ftp.close()
                self._ftp = None
