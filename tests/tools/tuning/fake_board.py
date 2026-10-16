"""A flight controller behind a fake pymavlink connection, for the board runner tests.

It keeps saved and live parameters (a group hidden until its enable parameter is set
shows up at the next boot), reboots when the link is opened again after a reboot
command, arms and disarms, and serves logs in 90-byte LOG_DATA chunks. Lists of
indices and offsets can be dropped once, as a lossy link would.
"""
from __future__ import annotations

import random
import struct

from pymavlink import mavutil

M = mavutil.mavlink
CHUNK = 90


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


class Msg:
    def __init__(self, mtype: str, **fields: object) -> None:
        self._type = mtype
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self._type

    def get_msgbuf(self) -> bytes:
        """Stands in for the packet's bytes: its type and fields, enough to tell packets apart."""
        return repr((self._type, sorted((k, v) for k, v in self.__dict__.items() if k != "_type"))).encode()


class FakeLog:
    def __init__(self, content: bytes, time_utc: int = 1700000000, listed_size: int | None = None) -> None:
        self.content = content
        self.time_utc = time_utc
        # a log still being written lists a smaller size until reading closes it (AP_Logger_File.cpp:610)
        self.listed_size = len(content) if listed_size is None else listed_size


class FakeBoard:
    def __init__(self, defaults: dict[str, float], stored: dict[str, float] | None = None,
                 hidden: dict[str, str] | None = None, armed: bool = False) -> None:
        self.defaults = {n: f32(v) for n, v in defaults.items()}
        self.stored = {n: f32(v) for n, v in (stored or {}).items()}
        self.hidden = dict(hidden or {})          # name -> enable parameter that reveals it at boot
        self.armed = armed
        self.logs: list[FakeLog] = []
        self.queue: list[Msg] = []
        self.events: list[str] = []
        self.overrides: list[tuple] = []
        self.boots = 0
        self.reboot_pending = False
        self.uptime_ms = 600_000
        self.drop_param_indices: set[int] = set()
        self.never_param_indices: set[int] = set()
        self.drop_log_offsets: set[int] = set()
        self.shuffle_log_data = False
        self.answer_log_data = True
        self.ignore_reboot = False                  # acknowledge a reboot but keep running
        self.vanish_on_reboot = False               # a USB port: gone at the reboot, no ack read
        self.port_gone = False
        self.boot_texts: list[list[str]] = []      # STATUSTEXTs of the next boots, one list per boot
        self._boot()
        self.uptime_ms = 600_000                    # up for a while before the runner connects

    # ---- the board -------------------------------------------------------------
    def _boot(self) -> None:
        self.ram = {**self.defaults, **self.stored}
        self.visible = sorted(n for n in self.ram if n not in self.hidden or self.ram.get(self.hidden[n], 0))
        self.queue.clear()
        self.uptime_ms = 0

    def connection(self, device: str, **kwargs: object) -> FakeConn:
        """The Board's connect factory: opening the link after a reboot command finds the new boot."""
        self.events.append("open")
        if self.reboot_pending:
            self.reboot_pending = False
            self.port_gone = False
            self.boots += 1
            self._boot()
            if self.boot_texts:
                self.queue.extend(Msg("STATUSTEXT", text=t, severity=6) for t in self.boot_texts.pop(0))
        return FakeConn(self)

    def heartbeat(self) -> Msg:
        return Msg("HEARTBEAT", type=1, autopilot=3, custom_mode=0,
                   base_mode=M.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)

    def param_msg(self, name: str, index: int) -> Msg:
        return Msg("PARAM_VALUE", param_id=name, param_value=self.ram[name], param_type=9,
                   param_count=len(self.visible), param_index=index)

    def ack(self, command: int, result: int = M.MAV_RESULT_ACCEPTED) -> None:
        self.queue.append(Msg("COMMAND_ACK", command=command, result=result))

    def command(self, cmd: int, p: list[float]) -> None:
        if cmd == M.MAV_CMD_PREFLIGHT_STORAGE and p[0] == 2:
            self.stored = {}                        # storage erased; RAM keeps its values until a boot
            self.events.append("erase")
            self.ack(cmd)
        elif cmd == M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and p[0] == 1:
            if self.armed:
                self.ack(cmd, M.MAV_RESULT_FAILED)
                return
            self.events.append("reboot")
            self.reboot_pending = not self.ignore_reboot
            if self.vanish_on_reboot:
                self.port_gone = True
                return
            self.ack(cmd)
        elif cmd == M.MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = bool(p[0])
            self.events.append(f"arm:{p[0]:g}:{p[1]:g}")
            self.ack(cmd)
        elif cmd == M.MAV_CMD_REQUEST_MESSAGE and p[0] == M.MAVLINK_MSG_ID_SYSTEM_TIME:
            self.queue.append(Msg("SYSTEM_TIME", time_boot_ms=self.uptime_ms, time_unix_usec=0))

    def log_data(self, entry: int, ofs: int, count: int) -> None:
        if not self.answer_log_data:
            return
        log = self.logs[entry - 1]
        log.listed_size = len(log.content)          # reading closes the file
        end = min(ofs + count, len(log.content))
        chunks = []
        while True:
            n = min(CHUNK, end - ofs) if ofs < end else 0
            if ofs in self.drop_log_offsets:
                self.drop_log_offsets.discard(ofs)
            else:
                data = list(log.content[ofs:ofs + n]) + [0] * (CHUNK - n)
                chunks.append(Msg("LOG_DATA", id=entry, ofs=ofs, count=n, data=data))
            ofs += n
            if n < CHUNK or ofs >= end:
                break
        if self.shuffle_log_data:
            random.Random(1).shuffle(chunks)
        self.queue.extend(chunks)


class FakeMav:
    def __init__(self, board: FakeBoard) -> None:
        self.b = board

    def param_request_list_send(self, ts: int, tc: int) -> None:
        for i, name in enumerate(self.b.visible):
            if i in self.b.never_param_indices:
                continue
            if i in self.b.drop_param_indices:
                self.b.drop_param_indices.discard(i)
                continue
            self.b.queue.append(self.b.param_msg(name, i))

    def param_request_read_send(self, ts: int, tc: int, name: bytes, index: int) -> None:
        if 0 <= index < len(self.b.visible) and index not in self.b.never_param_indices:
            self.b.queue.append(self.b.param_msg(self.b.visible[index], index))
        elif index < 0 and name.decode() in self.b.visible:
            self.b.queue.append(self.b.param_msg(name.decode(), 65535))

    def param_set_send(self, ts: int, tc: int, name: bytes, value: float, ptype: int) -> None:
        n = name.decode()
        self.b.events.append(f"set:{n}")
        if n in self.b.visible:
            self.b.ram[n] = self.b.stored[n] = f32(value)
            self.b.queue.append(self.b.param_msg(n, 65535))

    def command_long_send(self, ts: int, tc: int, cmd: int, confirmation: int, *p: float) -> None:
        self.b.command(cmd, list(p))

    def request_data_stream_send(self, *args: object) -> None:
        pass

    def rc_channels_override_send(self, *args: int) -> None:
        self.b.overrides.append(args)

    def log_request_list_send(self, ts: int, tc: int, start: int, end: int) -> None:
        if not self.b.logs:
            self.b.queue.append(Msg("LOG_ENTRY", id=0, num_logs=0, last_log_num=0, time_utc=0, size=0))
        for i, log in enumerate(self.b.logs, 1):
            self.b.queue.append(Msg("LOG_ENTRY", id=i, num_logs=len(self.b.logs), last_log_num=len(self.b.logs),
                                    time_utc=log.time_utc, size=log.listed_size))

    def log_request_data_send(self, ts: int, tc: int, entry: int, ofs: int, count: int) -> None:
        self.b.log_data(entry, ofs, count)

    def log_request_end_send(self, ts: int, tc: int) -> None:
        self.b.events.append("log_end")


class FakeConn:
    """Like a pymavlink connection, it calls its message_hooks with every message it returns and
    writes raw bytes (written; the board does not parse them)."""
    target_system = 1
    target_component = 1

    def __init__(self, board: FakeBoard) -> None:
        self.board = board
        self.mav = FakeMav(board)
        self.closed = False
        self.message_hooks: list = []
        self.written: list[bytes] = []

    def write(self, buf: bytes) -> int:
        assert not self.closed, "written after close"
        self.written.append(bytes(buf))
        return len(buf)

    def _posted(self, m: Msg | None) -> Msg | None:
        for hook in self.message_hooks if m is not None else ():
            hook(self, m)
        return m

    def wait_heartbeat(self, timeout: float | None = None) -> Msg:
        return self._posted(self.board.heartbeat())

    def recv_match(self, type: str | list[str] | None = None, blocking: bool = False,
                   timeout: float | None = None) -> Msg | None:
        if self.board.port_gone:
            raise OSError("ClearCommError failed: the device does not recognize the command")
        types = [type] if isinstance(type, str) else type
        for i, m in enumerate(self.board.queue):
            if types is None or m.get_type() in types:
                return self._posted(self.board.queue.pop(i))
        if types and "HEARTBEAT" in types:
            return self._posted(self.board.heartbeat())
        return None

    def close(self) -> None:
        self.closed = True
