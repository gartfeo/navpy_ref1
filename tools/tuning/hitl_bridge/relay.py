"""Windows relay of the HITL link: the board's second USB COM port <-> the bridge's TCP listener.

The bridge runs in WSL next to the JSBSim program; the Pixhawk's HIL port (SERIAL8 = USB
OTG2, the second CDC interface, today COM11) is a Windows COM port. This relay copies bytes
both ways without looking at them:
  board COM port  --HIL_ACTUATOR_CONTROLS-->  relay  --TCP-->  bridge (--device tcpin:PORT)
  board COM port  <--HIL_STATE_QUATERNION---  relay  <--TCP--  bridge
Two threads, one per direction, each forwarding a chunk as soon as it arrives (TCP_NODELAY;
a serial read returns the first byte with what arrived with it).

- The COM port disappears when the board reboots: it is reopened every --reopen-interval-s
  until it is back, while the TCP connection is kept (bytes from the bridge meanwhile are
  dropped and counted).
- The TCP connection is re-established when it breaks; when the bridge stays unreachable
  for --bridge-gone-s after a connection existed, the bridge has ended and the relay exits.
  Before the first connection it waits for the bridge as long as it takes. A connection
  counts only once it carried a byte or lasted relay_endpoint.CONFIRM_AFTER_S (WSL's
  localhost forwarding accepts and at once closes connections after the bridge is gone);
  after one that closed without data the next attempt waits the retry interval.
- Ctrl+C ends it. Counters are printed every --stats-interval-s and at the end.
- --device tcp:HOST:PORT replaces the COM port by a TCP connection (tests without a board).

Windows PowerShell 5.1 (stdlib + pyserial only; the module is found from the repo root, the working folder):
  .venv\\Scripts\\python.exe -m tools.tuning.hitl_bridge.relay --device COM11 --bridge 127.0.0.1:9620
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Callable, Optional

from tools.tuning.hitl_bridge.relay_endpoint import DirectionStats, Endpoint
from tools.tuning.hitl_bridge.relay_timing import DOWN, UP, FrameTimer
from tools.tuning.hitl_bridge.transport import DEFAULT_SERIAL_BAUD, TCP_PREFIX, TcpTransport, make_transport

DEFAULT_BRIDGE = "127.0.0.1:9620"       # the bridge's tcpin port (WSL forwards Windows localhost to it)
# A rebooting board is back after a few seconds; trying twice a second finds it quickly.
DEFAULT_REOPEN_INTERVAL_S = 0.5
DEFAULT_RECONNECT_INTERVAL_S = 0.5
# The bridge restarts nothing by itself, so if its port stays closed this long it has ended.
DEFAULT_BRIDGE_GONE_S = 10.0
DEFAULT_STATS_INTERVAL_S = 10.0
# On Windows a blocked wait is not interrupted by Ctrl+C; the main thread waits in slices
# this long, so Ctrl+C is handled within it.
CTRL_C_SLICE_S = 0.25
EXIT_OK = 0
EXIT_BAD_ARGS = 2


class Relay:
    def __init__(self, device: Endpoint, bridge: Endpoint, bridge_gone_s: float,
                 clock: Callable[[], float] = time.monotonic, log: Callable[[str], None] = print,
                 timer: Optional[FrameTimer] = None, stamp: Callable[[], float] = time.perf_counter) -> None:
        self.device = device
        self.bridge = bridge
        self.bridge_gone_s = bridge_gone_s
        self._clock = clock
        self._log = log
        self.stop = threading.Event()
        self.reason = ""
        self.to_bridge = DirectionStats()
        self.to_device = DirectionStats()
        self._threads: list[threading.Thread] = []
        self.timer = timer              # --timing-csv: per-frame PC timestamps (relay_timing.py)
        self._stamp = stamp

    def _end(self, reason: str) -> None:
        if not self.stop.is_set():
            self.reason = reason
            self.stop.set()

    def _gone(self, side: Endpoint, gone_after_s: Optional[float]) -> bool:
        """True (and the relay ends) when the side has been down too long since a confirmed connection."""
        if gone_after_s is None or side.down_since is None or self._clock() - side.down_since <= gone_after_s:
            return False
        self._end(f"{side.name} unreachable for more than {gone_after_s} s")
        return True

    def _ensure_open(self, side: Endpoint, gone_after_s: Optional[float]) -> bool:
        """Owner thread: (re)open the side, retrying; False when the relay must stop."""
        if side.up and not side.failed:
            return True
        if side.up:
            side.drop("write failed")
        if side.closed_without_data:        # do not spin on connections that close at once
            self.stop.wait(side.retry_interval_s)
        reported = False
        while not self.stop.is_set():
            if self._gone(side, gone_after_s):
                return False
            error = side.try_open()
            if error is None:
                return True
            if not reported:                # once per outage
                self._log(f"relay: {side.name} not available ({error}); retrying every {side.retry_interval_s} s")
                reported = True
            self.stop.wait(side.retry_interval_s)
        return False

    def _pump(self, src: Endpoint, dst: Endpoint, stats: DirectionStats, gone_after_s: Optional[float],
              direction: str) -> None:
        while not self.stop.is_set():
            if not self._ensure_open(src, gone_after_s):
                return
            try:
                data = src.read()
            except OSError as exc:
                src.drop(exc)
                continue
            read_s = self._stamp()
            src.note_read(bool(data))
            if not data:
                continue
            if dst.write(data):
                stats.bytes += len(data)
                stats.chunks += 1
                if self.timer is not None:
                    self.timer.note(direction, data, read_s, self._stamp())
            else:
                stats.dropped_bytes += len(data)

    def start(self) -> None:
        self._threads = [
            threading.Thread(target=self._pump, args=(self.device, self.bridge, self.to_bridge, None, UP),
                             name="relay-device", daemon=True),
            threading.Thread(target=self._pump, args=(self.bridge, self.device, self.to_device, self.bridge_gone_s, DOWN),
                             name="relay-bridge", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self._end(self.reason or "stopped")
        for thread in self._threads:
            thread.join()
        self.device.shutdown()
        self.bridge.shutdown()
        if self.timer is not None:
            self.timer.close()

    def summary(self) -> str:
        """Bytes written into connections that closed before they counted are dropped, not forwarded."""
        def direction(stats: DirectionStats, dst: Endpoint) -> str:
            return (f"{stats.bytes - dst.lost_bytes} B forwarded in {stats.chunks} writes, "
                    f"{stats.dropped_bytes + dst.lost_bytes} B dropped")
        text = (f"device->bridge {direction(self.to_bridge, self.bridge)}; bridge->device "
                f"{direction(self.to_device, self.device)}; device opens {self.device.opens} losses "
                f"{self.device.losses}; bridge connects {self.bridge.opens} losses {self.bridge.losses}, "
                f"{self.bridge.tentative_closes} closed without data")
        return text if self.timer is None else f"{text}; {self.timer.summary()}"


def parse_host_port(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit():
        raise ValueError(f"expected HOST:PORT, got {text!r}")
    return host, int(port)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--device", required=True,
                   help=f"the board's HIL COM port (COM11), or {TCP_PREFIX}HOST:PORT for a test without a board")
    p.add_argument("--baud", type=int, default=DEFAULT_SERIAL_BAUD, help="serial baud (USB CDC ignores it)")
    p.add_argument("--bridge", default=DEFAULT_BRIDGE, help="HOST:PORT of the bridge's tcpin listener")
    p.add_argument("--reopen-interval-s", type=float, default=DEFAULT_REOPEN_INTERVAL_S,
                   help="retry period for a COM port that is gone (board reboot)")
    p.add_argument("--reconnect-interval-s", type=float, default=DEFAULT_RECONNECT_INTERVAL_S,
                   help="retry period for the TCP connection to the bridge")
    p.add_argument("--bridge-gone-s", type=float, default=DEFAULT_BRIDGE_GONE_S,
                   help="exit when the bridge stays unreachable this long after a connection existed")
    p.add_argument("--stats-interval-s", type=float, default=DEFAULT_STATS_INTERVAL_S)
    p.add_argument("--timing-csv", default=None,
                   help="write the PC time of every HIL frame each way (relay_timing.py) and add the round trip "
                        "and the stalls from the board to the counters")
    return p


def say(text: str) -> None:
    """One console line, flushed: the runner and the owner read it from a pipe or a file live."""
    print(text, flush=True)


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        host, port = parse_host_port(args.bridge)
        device_transport = make_transport(args.device, args.baud)
    except (ValueError, OSError) as exc:
        print(f"relay: {exc}", file=sys.stderr)
        return EXIT_BAD_ARGS
    timer = FrameTimer(open(args.timing_csv, "w", newline="")) if args.timing_csv else None
    relay = Relay(Endpoint(args.device, device_transport, args.reopen_interval_s, say),
                  Endpoint(f"bridge {args.bridge}", TcpTransport(host, port), args.reconnect_interval_s, say),
                  args.bridge_gone_s, log=say, timer=timer)
    say(f"relay: {args.device} <-> {args.bridge}; Ctrl+C to stop")
    relay.start()
    next_stats = time.monotonic() + args.stats_interval_s
    try:
        while not relay.stop.is_set():
            time.sleep(CTRL_C_SLICE_S)
            if time.monotonic() >= next_stats:
                say(f"relay: {relay.summary()}")
                next_stats += args.stats_interval_s
    except KeyboardInterrupt:
        relay.reason = "Ctrl+C"
    finally:
        relay.close()
        say(f"relay: stopped ({relay.reason}); {relay.summary()}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
