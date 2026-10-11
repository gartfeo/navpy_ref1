"""MAVFTP adapter over explicit locked transport and subscriptions."""
from __future__ import annotations

import queue
from typing import Callable, Optional

from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_AUTOPILOT1

from ._upstream_mavftp import MAVFTP


class _BridgeMav:
    def __init__(self, transport) -> None:
        self._transport = transport

    def file_transfer_protocol_send(
        self,
        *args: int | bytes | bytearray,
        **kwargs: int | bytes | bytearray,
    ) -> None:
        return self._transport.call(
            lambda connection: connection.mav.file_transfer_protocol_send(
                *args, **kwargs,
            )
        )


class _BridgeMaster:
    def __init__(self, transport, identity, target_component: int) -> None:
        self.target_system = identity.target_system
        self.target_component = target_component
        self.source_system = (
            transport.source_system
            if transport.source_system is not None
            else identity.source_system
        )
        self.source_component = transport.source_component or 0
        self.mav = _BridgeMav(transport)
        self._inbox = queue.Queue()

    def feed(self, message) -> None:
        try:
            if message.get_srcSystem() != self.target_system:
                return
        except Exception:
            return
        self._inbox.put(message)

    def recv_match(self, *, type=None, timeout: float = 0.1, **_kwargs):
        try:
            return self._inbox.get(timeout=max(0.0, timeout))
        except queue.Empty:
            return None


class VehicleFtp:
    def __init__(self, transport, identity, callbacks) -> None:
        self.master = _BridgeMaster(
            transport, identity, MAV_COMP_ID_AUTOPILOT1,
        )
        self._subscription = callbacks.subscribe(
            "FILE_TRANSFER_PROTOCOL", self.master.feed,
        )
        self.ftp = MAVFTP(
            self.master,
            target_system=identity.target_system,
            target_component=MAV_COMP_ID_AUTOPILOT1,
        )

    def close(self) -> None:
        self._subscription.cancel()

    def fetch_param_pck(
        self,
        *,
        with_defaults: bool = True,
        timeout: float = 30.0,
        progress_callback: Optional[Callable[[dict | None], None]] = None,
    ) -> bytes:
        captured: dict = {}

        def _grab(file_handle) -> None:
            if file_handle is not None:
                captured["data"] = file_handle.read()

        path = (
            "@PARAM/param.pck?withdefaults=1"
            if with_defaults
            else "@PARAM/param.pck"
        )
        self.ftp.cmd_get(
            [path],
            callback=_grab,
            progress_callback_bytes=progress_callback,
        )
        result = self.ftp.process_ftp_reply("OpenFileRO", timeout=timeout)
        if result.error_code != 0:
            raise RuntimeError(
                f"MAVFTP get {path} failed: {result.operation_name} "
                f"error={result.error_code}"
            )
        if "data" not in captured:
            raise RuntimeError(
                f"MAVFTP get {path} completed but no data was captured."
            )
        return captured["data"]
