"""Unreal native camera frames (NPYC over TCP) as a frame-capture backend.

Source ``npyc://HOST:PORT``: navpy listens, the Unreal gimbal camera actor
connects (NativeCameraHost/NativeCameraPort, 127.0.0.1:18110 for UAV1) and
reconnects by itself after a drop, so this backend simply accepts again.

Stamps: a v2 frame whose pose is stamped and exact carries the board time
of the pose it was rendered from; with a ``BoardClock`` that board time is
mapped onto the ATTITUDE receipt wall clock and published as an EXPOSURE
stamp. Every other frame (v1, interpolated pose, no mapping yet) gets a
READ stamp taken when its last byte arrived: the stamp kind always says
which it is, and an exposure time is never synthesized.
"""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt

import socket
from collections.abc import Callable
from typing import Optional

import numpy as np

from navpy.core.ports.frame_capture_ports import (
    BoardClock,
    CaptureLogger,
    CaptureRunState,
    CaptureStamp,
    CaptureStampKind,
    FramePublisher,
)
from navpy.plugins.camera.unreal.npyc_codec import (
    NPYC_PREFIX_SIZE,
    NpycHeader,
    bytes_per_pixel,
    decode_header,
    decode_prefix,
)

NPYC_SCHEME = "npyc://"

# Diagnostics: (header, receipt wall time, published stamp) for every frame.
FrameObserver = Callable[[NpycHeader, float, CaptureStamp], None]


def is_npyc_source(source: object) -> bool:
    return isinstance(source, str) and source.lower().startswith(NPYC_SCHEME)


def parse_npyc_source(source: str) -> tuple[str, int]:
    """``npyc://HOST:PORT`` -> (host, port); ValueError otherwise."""
    if not is_npyc_source(source):
        raise ValueError(f"not an npyc:// source: {source!r}")
    host, sep, port = source[len(NPYC_SCHEME):].rpartition(":")
    if not sep or not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"npyc source {source!r}: expected npyc://HOST:PORT")
    return host, int(port)


class StreamClosed(Exception):
    """The peer closed the connection (or it was shut down by request_stop)."""


class NpycFrameCapture:
    """Listen for the Unreal camera, decode NPYC frames, stamp and publish them."""

    def __init__(
        self,
        listener: socket.socket,
        source: str,
        logger: CaptureLogger,
        board_clock: Optional[BoardClock] = None,
        observer: Optional[FrameObserver] = None,
    ) -> None:
        self._listener = listener
        self._source = source
        self._logger = logger
        self._board_clock = board_clock
        self._observer = observer
        self._lock = rt.Lock()
        self._connection: Optional[socket.socket] = None
        self._stop_requested = False
        self._closed = False
        self.frames = 0
        self.connections = 0

    @classmethod
    def open(
        cls,
        source: str,
        logger: CaptureLogger,
        board_clock: Optional[BoardClock] = None,
        observer: Optional[FrameObserver] = None,
    ) -> Optional["NpycFrameCapture"]:
        endpoint = parse_npyc_source(source)
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            listener.bind(endpoint)
            listener.listen(1)
        except OSError as error:
            listener.close()
            logger.error(f"FrameProvider: cannot listen on {source}: {error}")
            return None
        logger.info(
            f"FrameProvider: listening on {source} for the Unreal camera "
            f"({'board-clock EXPOSURE stamps' if board_clock else 'READ stamps only'})"
        )
        return cls(listener, source, logger, board_clock, observer)

    def run(self, publish: FramePublisher, is_running: CaptureRunState) -> None:
        while is_running():
            connection = self._accept()
            if connection is None:
                return
            try:
                self._serve(connection, publish, is_running)
            finally:
                self._release(connection)

    def _accept(self) -> Optional[socket.socket]:
        try:
            connection, peer = self._listener.accept()
        except OSError as error:
            if not self._stop_requested:
                self._logger.error(f"FrameProvider: {self._source} accept failed: {error}")
            return None
        with self._lock:
            if self._stop_requested:
                connection.close()
                return None
            self._connection = connection
        self.connections += 1
        self._logger.info(f"FrameProvider: Unreal camera connected from {peer[0]}:{peer[1]}")
        return connection

    def _release(self, connection: socket.socket) -> None:
        with self._lock:
            if self._connection is connection:
                self._connection = None
        try:
            connection.close()
        except OSError:
            pass

    def _serve(self, connection: socket.socket, publish: FramePublisher, is_running: CaptureRunState) -> None:
        while is_running():
            try:
                header, payload = _read_frame(connection)
            except StreamClosed:
                if not self._stop_requested:
                    self._logger.warning(f"FrameProvider: Unreal camera on {self._source} disconnected")
                return
            except ValueError as error:
                self._logger.warning(f"FrameProvider: {self._source} protocol error, dropping the connection: {error}")
                return
            except OSError as error:
                if not self._stop_requested:
                    self._logger.warning(f"FrameProvider: {self._source} read failed: {error}")
                return
            read_at_s = rt.time()
            stamp = self._stamp(header, read_at_s)
            frame = _pixels(header, payload)
            self.frames += 1
            if self._observer is not None:
                self._observer(header, read_at_s, stamp)
            publish(frame, stamp)

    def _stamp(self, header: NpycHeader, read_at_s: float) -> CaptureStamp:
        if header.exposure_on_board_clock and self._board_clock is not None:
            exposure_s = self._board_clock.wall_time_s(header.board_time_us)
            if exposure_s is not None:
                return CaptureStamp(exposure_s, CaptureStampKind.EXPOSURE)
        return CaptureStamp(read_at_s, CaptureStampKind.READ)

    def request_stop(self) -> bool:
        with self._lock:
            self._stop_requested = True
            connection = self._connection
        for sock in (connection, self._listener):
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)     # unblocks recv/accept on every platform
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        return True

    def close(self) -> None:
        if self._closed:
            return
        self.request_stop()
        self._closed = True


def _read_frame(connection: socket.socket) -> tuple[NpycHeader, bytearray]:
    prefix = _recv_exact(connection, NPYC_PREFIX_SIZE)
    _version, header_size = decode_prefix(bytes(prefix))
    raw = bytes(prefix) + bytes(_recv_exact(connection, header_size - NPYC_PREFIX_SIZE))
    header = decode_header(raw)
    return header, _recv_exact(connection, header.payload_size)


def _recv_exact(connection: socket.socket, size: int) -> bytearray:
    buffer = bytearray(size)
    view = memoryview(buffer)
    received = 0
    while received < size:
        count = connection.recv_into(view[received:], size - received)
        if count == 0:
            raise StreamClosed()
        received += count
    return buffer


def _pixels(header: NpycHeader, payload: bytearray) -> np.ndarray:
    channels = bytes_per_pixel(header.pixel_format)
    image = np.frombuffer(payload, dtype=np.uint8).reshape(header.height, header.width, channels)
    if channels == 4:
        return np.ascontiguousarray(image[:, :, :3])    # BGRA8 -> BGR
    return image


__all__ = [
    "NPYC_SCHEME",
    "FrameObserver",
    "NpycFrameCapture",
    "is_npyc_source",
    "parse_npyc_source",
]
