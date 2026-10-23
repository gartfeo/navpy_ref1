"""The emulated ZR10 on the wire: SIYI frames in, reply frames out. No sockets.

Codec: navpy's siyi_message.py; motion: zr10_gimbal; unrequested attitude frames: zr10_streams. Here: the handlers
and the zoom table (design 4).

Wire conventions, as the real unit (design section 3, "Signs"):
- 0x0E command and 0x0D report yaw are left-positive: wire yaw = -plant yaw. Pitch is up-positive on both.
- A positive 0x07 yaw speed turns the camera right: the plant yaw rises and the reported yaw falls (rig step 0,
  2026-10-14). A positive pitch speed raises the pitch (TBC).
- The 0x0D angles are body-relative, yaw-outer (siyi_convention.py); the 0x0E angles go to the plant as navpy's sim
  takes them (gimbal_siyi_control.set_att: world angles in lock mode). Both references are unmeasured on the real
  unit (gimbal-controller, 2026-10-16: all calibration was in follow mode), so they stay as navpy's sim.

Ordering: a command acts from the device time it is handled at (after every overdue step). 0x0E and 0x08 cancel
the rates still waiting out a 0x07 dead time.

Fail closed: nothing is answered before the first aircraft attitude. Bad frames and unknown, rejected or refused
commands make the run invalid. A step at a mechanical stop is counted only: the camera stops there, as the real
gimbal (zr10_gimbal).
"""
from __future__ import annotations

import struct
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

from navpy.core.kernel.attitude import Attitude
from navpy.plugins.gimbal.siyi.siyi_message import COMMAND, SIYIMESSAGE
from navpy.sim.gimbal.siyi.gimbal_angles import MODE_FOLLOW, MODE_FPV, MODE_LOCK
from tools.hitl.zr10_config import EmulatorConfig
from tools.hitl.zr10_gimbal import Report, TrueState, Zr10Gimbal, reading
from tools.hitl.zr10_streams import AttitudeStreams
from tools.hitl.zr10_wire import ReplyEncoder, int16s, split_frames

Address = tuple[str, int]

# 0x01 payload of the real unit: camera 0.3.9, gimbal 0.4.3, zoom 0.2.3 (gimbal-controller docs/PROTOCOL.md:8)
FIRMWARE_HEX = "0903006e0304006b0302006d"
HARDWARE_ID = "6B02234248"    # the real unit's (PROTOCOL.md:7); navpy's SDK needs it to start with "6B" (ZR10)
MOUNT_NORMAL = 1
RECORD_NO_TF_CARD = 2
FUNCTION_MODES = {3: MODE_LOCK, 4: MODE_FOLLOW}
# navpy's plant keeps the world pitch when it enters FPV, so a pitched aircraft would see the camera jump; navpy
# never asks for FPV, so it is refused rather than rendered wrong.
FUNCTION_FPV = 5
MODE_NAMES = {MODE_LOCK: "lock", MODE_FOLLOW: "follow", MODE_FPV: "fpv"}


@dataclass
class Counters:
    frames: int = 0
    replies: int = 0
    pushes: int = 0
    bad_frames: int = 0
    unknown_commands: int = 0
    rejected_commands: int = 0
    refused_commands: int = 0     # answered with a failure (00): navpy asked for something the unit does not do
    before_attitude: int = 0
    stop_steps: int = 0           # steps that ended with the camera held at a body yaw/pitch stop
    by_command: dict[str, int] = field(default_factory=dict)


@dataclass(frozen=True)
class CommandRecord:
    t_s: float
    cmd: str
    data: str
    result: str


class Zr10Device:
    def __init__(self, config: EmulatorConfig, zoom_table: Mapping[int, float]) -> None:
        if len(zoom_table) < 2:
            raise ValueError("the zoom table needs at least two rows")
        self.config = config
        self._zoom_table = dict(zoom_table)
        self.gimbal = Zr10Gimbal(config, self._zoom_table)
        self.streams = AttitudeStreams(config.attitude_push_hz)
        self._encoder = ReplyEncoder()
        self._decoder = SIYIMESSAGE()
        self.counters = Counters()
        self.records: list[CommandRecord] = []
        self._handlers: dict[str, Callable[[bytes, Address], str | None]] = {
            COMMAND.ACQUIRE_FW_VER: self._firmware,
            COMMAND.ACQUIRE_HW_ID: self._hardware_id,
            COMMAND.AUTO_FOCUS: self._auto_focus,
            COMMAND.MANUAL_ZOOM: self._manual_zoom,
            COMMAND.GIMBAL_SPEED: self._gimbal_speed,
            COMMAND.CENTER: self._center,
            COMMAND.ACQUIRE_GIMBAL_INFO: self._gimbal_info,
            COMMAND.PHOTO_VIDEO_HDR: self._function,
            COMMAND.ACQUIRE_GIMBAL_ATT: self._attitude,
            COMMAND.SET_GIMBAL_ATTITUDE: self._set_angles,
            COMMAND.SET_DATA_STREAM: self._data_stream,
            COMMAND.ABSOLUTE_ZOOM: self._absolute_zoom,
            COMMAND.CURRENT_ZOOM_VALUE: self._current_zoom,
        }

    # -- time, aircraft and state (zr10_gimbal) ----------------------------------------------------------------
    @property
    def now_s(self) -> float:
        return self.gimbal.now_s

    @property
    def has_attitude(self) -> bool:
        return self.gimbal.has_attitude

    def set_vehicle_attitude(self, attitude: Attitude) -> None:
        self.gimbal.set_vehicle_attitude(attitude)

    def step(self) -> list[tuple[bytes, Address]]:
        """Advance to the next 20 ms step boundary; return the 0x25 stream frames now due."""
        self.gimbal.step()
        if not self.gimbal.has_attitude:
            return []
        if self.gimbal.at_stop():
            self.counters.stop_steps += 1
        return self._due_pushes()

    def advance_to(self, t_s: float) -> None:
        self.gimbal.advance_to(t_s)

    def true_state(self) -> TrueState | None:
        return self.gimbal.true_state()

    def reported(self) -> Report | None:
        return self.gimbal.reported()

    def invalid_reasons(self) -> list[str]:
        counters = self.counters
        reasons = []
        for count, what in ((counters.bad_frames, "bad SIYI frame(s)"),
                            (counters.unknown_commands, "unknown SIYI command(s)"),
                            (counters.rejected_commands, "rejected SIYI command(s)"),
                            (counters.refused_commands, "refused SIYI command(s)")):
            if count:
                reasons.append(f"{count} {what}")
        return reasons

    # -- frames -------------------------------------------------------------------------------------------
    def handle_datagram(self, datagram: bytes, source: Address) -> list[bytes]:
        """Every frame in one datagram, in order; returns the reply frames for ``source``."""
        frames, bad = split_frames(datagram, self._decoder)
        self.counters.bad_frames += bad
        replies = (self._handle_frame(cmd_id, data, source) for cmd_id, data in frames)
        return [reply for reply in replies if reply is not None]

    def _handle_frame(self, cmd_id: str, data: bytes, source: Address) -> bytes | None:
        self.counters.frames += 1
        self.counters.by_command[cmd_id] = self.counters.by_command.get(cmd_id, 0) + 1
        if not self.gimbal.has_attitude:
            self.counters.before_attitude += 1
            self.records.append(CommandRecord(self.now_s, cmd_id, data.hex(), "dropped: no aircraft attitude yet"))
            return None
        handler = self._handlers.get(cmd_id)
        if handler is None:
            self.counters.unknown_commands += 1
            self.records.append(CommandRecord(self.now_s, cmd_id, data.hex(), "ignored: unknown command"))
            return None
        try:
            reply_hex = handler(data, source)
        except (ValueError, struct.error) as exc:
            self.counters.rejected_commands += 1
            self.records.append(CommandRecord(self.now_s, cmd_id, data.hex(), f"rejected: {exc}"))
            return None
        self.records.append(CommandRecord(self.now_s, cmd_id, data.hex(),
                                          "no reply" if reply_hex is None else "replied"))
        if reply_hex is None:
            return None
        self.counters.replies += 1
        return self._frame(reply_hex, cmd_id)

    def drain_records(self) -> list[CommandRecord]:
        records, self.records = self.records, []
        return records

    def _frame(self, data_hex: str, cmd_id: str) -> bytes:
        return self._encoder.frame(data_hex, cmd_id)

    # -- command handlers: data in, reply data (hex) out, None for no reply ----------------------------------
    def _firmware(self, data: bytes, source: Address) -> str:
        return FIRMWARE_HEX

    def _hardware_id(self, data: bytes, source: Address) -> str:
        return HARDWARE_ID.encode("ascii").ljust(12, b"\x00").hex()

    def _auto_focus(self, data: bytes, source: Address) -> str:
        return "01"

    def _manual_zoom(self, data: bytes, source: Address) -> str:
        (direction,) = struct.unpack_from("<b", data)
        self.gimbal.manual_zoom((direction > 0) - (direction < 0))
        return struct.pack("<H", int(round(reading(self.gimbal.zoom.level) * 10.0))).hex()

    def _gimbal_speed(self, data: bytes, source: Address) -> str:
        yaw_speed, pitch_speed = struct.unpack_from("<2b", data)
        self.gimbal.set_rate(yaw_speed, pitch_speed)
        return "01"

    def _center(self, data: bytes, source: Address) -> str:
        if data != b"\x01":
            return self._refused()
        self.gimbal.cancel_rates()
        plant = self.gimbal.plant
        snapshot = plant.snapshot()
        vehicle = snapshot.vehicle_attitude
        if snapshot.motion_mode == MODE_LOCK:
            plant.set_target_angles(vehicle.yaw, vehicle.pitch)    # lock holds world angles: the body centre
        else:
            plant.set_target_angles(0.0, vehicle.pitch)
        return "01"

    def _gimbal_info(self, data: bytes, source: Address) -> str:
        mode = self.gimbal.plant.snapshot().motion_mode
        return bytes((0, 0, 0, RECORD_NO_TF_CARD, mode, MOUNT_NORMAL)).hex()

    def _function(self, data: bytes, source: Address) -> None:
        if data[:1] == bytes((FUNCTION_FPV,)):
            raise ValueError("FPV is not modelled")
        mode = FUNCTION_MODES.get(data[0]) if data else None
        if mode is None:
            raise ValueError(f"function {data.hex()!r} is not a motion mode")
        self.gimbal.plant.set_motion_mode(mode)
        return None

    def _attitude(self, data: bytes, source: Address) -> str:
        self.streams.requested(source, self.now_s)
        return self._attitude_hex()

    def _attitude_hex(self) -> str:
        report = self.reported()
        assert report is not None
        return int16s(report.yaw, report.pitch, report.roll, report.yaw_rate, report.pitch_rate, report.roll_rate)

    def _set_angles(self, data: bytes, source: Address) -> str:
        yaw_tenths, pitch_tenths = struct.unpack_from("<2h", data)
        self.gimbal.cancel_rates()
        self.gimbal.plant.set_target_angles(-yaw_tenths / 10.0, pitch_tenths / 10.0)
        report = self.reported()
        assert report is not None
        return int16s(report.yaw, report.pitch, report.roll)

    def _data_stream(self, data: bytes, source: Address) -> str:
        data_type, code = struct.unpack_from("<2B", data)
        self.streams.set_stream(source, data_type, code, self.now_s)
        return f"{data_type:02x}"

    def _absolute_zoom(self, data: bytes, source: Address) -> str:
        whole, tenth = struct.unpack_from("<2B", data)
        if tenth > 9:
            raise ValueError(f"zoom tenths {tenth} out of range")
        command = whole * 10 + tenth
        target = self._zoom_table.get(command)
        if target is None:
            return self._refused()     # a command outside the profile's table: the lens does not move
        self.gimbal.seek_zoom(command, target)
        return "01"

    def _current_zoom(self, data: bytes, source: Address) -> str:
        report = self.reported()
        assert report is not None
        whole, tenth = divmod(int(round(report.zoom * 10.0)), 10)
        return bytes((whole, tenth)).hex()

    # -- internals ------------------------------------------------------------------------------------------
    def _refused(self) -> str:
        self.counters.refused_commands += 1
        return "00"

    def _due_pushes(self) -> list[tuple[bytes, Address]]:
        pushes = [(self._frame(self._attitude_hex(), COMMAND.ACQUIRE_GIMBAL_ATT), source)
                  for source in self.streams.due(self.now_s)]
        self.counters.pushes += len(pushes)
        return pushes


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, str(mode))


__all__ = ["Address", "CommandRecord", "Counters", "Zr10Device", "mode_name"]
