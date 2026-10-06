import random
import threading
import time
from typing import Optional

import serial

from navpy.args.conn.serial_network_args import SerialNetworkArgs
from navpy.modules.comm.serial.serial_listener import ISerialListener
from navpy.logger.cache_logger import ILogger

MAX_PACKET_SIZE = 340 # SiK air-frame limit
DELIM = b"\x1E" # ASCII “record separator”
LOOP_SLEEP = 0.01
SEND_JITTER_MIN = 0.05 # avoids clumping
SEND_JITTER_MAX = 0.15

_SIK_PREFIXES = (
b"OK", b"ERR", b"RSSI", b"ATI", b"RFD", b"VER", b"IM",
b"S0:", b"S1:", b"S2:", b"S3:", b"S4:", b"S5:"
)


class SerialInterfaceRFD900:
    """
    Threaded raw serial transport with delimiter framing.
    """
    # .....................................................................
    def __init__(self, args: SerialNetworkArgs, logger: ILogger):
        self._log = logger
        try:
            self._ser = serial.serial_for_url(args.conn, args.baud, timeout=0)
            self._log.info(f"Opened {args.conn} at {args.baud} baud")
        except serial.SerialException as exc:
            self._log.error(f"Failed to open {args.conn}: {exc}")
            raise

        self._listener: Optional[ISerialListener] = None
        self._running = True

        # TX state
        self._tx_queue: list[bytes] = []
        self._tx_lock = threading.Lock()

        # RX state
        self._stream = bytearray()    # delimiter accumulator
        self._ascii = bytearray()     # CRLF-terminated lines

        # Threads
        self._tx_thr = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thr = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thr.start()
        self._rx_thr.start()

    # .....................................................................
    # Public API
    # .....................................................................
    def register_listener(self, listener: ISerialListener) -> None:
        """Forward decoded payloads to *listener*."""
        self._listener = listener

    def send_data(self, payload: bytes) -> None:
        """Queue a message for transmission."""
        if not payload:
            return
        if len(payload) > MAX_PACKET_SIZE:
            self._log.error("send_data: packet too large – dropped")
            return

        # Escape delimiter (0x1E -> 0x1E 0x1E) and append terminator.
        enc = payload.replace(DELIM, DELIM * 2) + DELIM
        with self._tx_lock:
            self._tx_queue.append(enc)

    def close(self) -> None:
        """Stop background threads and close the port."""
        self._running = False
        self._tx_thr.join()
        self._rx_thr.join()
        self._ser.close()
        self._log.info("SerialInterface closed")

    # .....................................................................
    # Internal – TX
    # .....................................................................
    def _tx_loop(self) -> None:
        while self._running:
            pkt: Optional[bytes] = None
            with self._tx_lock:
                if self._tx_queue:
                    pkt = self._tx_queue.pop(0)

            if pkt:
                try:
                    time.sleep(random.uniform(SEND_JITTER_MIN, SEND_JITTER_MAX))
                    self._ser.write(pkt)
                    self._log.debug(f"TX {len(pkt)} bytes")
                except Exception as exc:
                    self._log.error(f"TX error: {exc}")
            else:
                time.sleep(LOOP_SLEEP)

    # .....................................................................
    # Internal – RX
    # .....................................................................
    def _rx_loop(self) -> None:
        while self._running:
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if not chunk:
                    time.sleep(LOOP_SLEEP)
                    continue

                self._handle_ascii(chunk)   # strip & log AT responses
                self._handle_delim(chunk)   # assemble user payloads
            except Exception as exc:
                self._log.error(f"RX error: {exc}")

    # .................................................... ASCII responses
    def _handle_ascii(self, data: bytes) -> None:
        self._ascii.extend(data)
        while True:
            idx = self._ascii.find(b"\r\n")
            if idx == -1:
                break
            line = self._ascii[:idx]
            del self._ascii[: idx + 2]
            if any(line.startswith(p) for p in _SIK_PREFIXES):
                try:
                    self._log.info(f"Radio: {line.decode('utf-8', 'ignore').strip()}")
                except Exception:
                    pass
        # Prevent unbounded growth if CRLF never appears
        if len(self._ascii) > 1024:
            self._ascii.clear()

    # .................................................... delimiter framing
    def _handle_delim(self, data: bytes) -> None:
        self._stream.extend(data)
        while True:
            pos = self._stream.find(DELIM)
            if pos == -1:
                break

            # Extract one frame (without delimiter) and unescape 0x1E duplication.
            frame = bytes(self._stream[:pos]).replace(DELIM * 2, DELIM)
            del self._stream[: pos + 1]   # +1 to drop delimiter itself

            if not frame:
                continue
            if self._listener:
                self._listener.receive_packet(frame)
            else:
                self._log.debug(f"RX {len(frame)} bytes (no listener)")