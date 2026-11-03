"""A synthetic HITL run for step 12's checker: navpy's integration log, the publisher's frame log, Unreal's capture
slots and the ZR10 emulator's log, all from one known world.

Clocks: board time b; the receiver's capture clock r(b) = b (1 + ppm) + EPOCH_R; the emulator's device clock d,
which the receiver sees as d + EPOCH_G (one PC clock, another epoch). Unreal renders board slots of 1/30 s and the
gimbal state at T_r = r(B) - EPOCH_G - RENDER_LAG_S on the device clock. A frame is read at r(B) + Dc + jitter and
maps to r(B) + Dc. ATTITUDE (50 Hz, time_boot cut to whole ms) arrives at r(tb) + Ds_a + jitter; each 20 ms plant
step pushes one 0x0D frame reporting plant time d - lag, received at d + EPOCH_G + Ds_g + jitter. So the true
corrections are C_a = Ds_a - Dc and C_g = lag + Ds_g - Dc - RENDER_LAG_S. navpy's own interpolation functions place
each association, as the live code does.
"""
from __future__ import annotations

import bisect
import io
import json
import math
from dataclasses import dataclass, field, replace
from types import SimpleNamespace

import numpy as np

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.capture_interpolation import interpolate_attitude_at
from navpy.core.perception.camera.integration_timing import (
    STATUS_ABORTED,
    STATUS_UNSETTLED,
    IntegrationTimingConfig,
    IntegrationTimingLimits,
)
from navpy.core.perception.pipeline.integration_association import STATUS_ZOOM_UNSETTLED, interpolate_gimbal_at
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory, GimbalHistorySample

EPOCH_R = 1000.0
EPOCH_G = 5000.0
RENDER_LAG_S = 0.004
FPS = 30.0
BOARD_START_S = 100.0
SESSION = "emu-session-1"
RUN = "camera-run-1"
TICK_NS = 1e9 / 90000.0
ATTITUDE_BRACKET_S = 0.05            # the live builder's two ATTITUDE periods at the 40 Hz fallback
SEAM_TOLERANCE_S = 0.025             # half of it, as real_frame_association passes
PUBLISHER_COLUMNS = ("received_ns,frame_index,timestamp_s,npyc_version,board_time_us,pose_sequence,pose_flags,"
                     "lens_reading_tenths,lens_setting_tenths,lens_status,pts_ns,missed_slots,outcome,flow_return,"
                     "queue_level,reason")


@dataclass(frozen=True)
class World:
    seconds: float = 60.0
    run_id: str = "cal-001"
    ppm: float = 0.0
    video_delay_s: float = 0.060
    video_jitter_s: float = 0.002
    attitude_delay_s: float = 0.012
    attitude_jitter_s: float = 0.0003
    gimbal_delay_s: float = 0.003
    gimbal_jitter_s: float = 0.0003
    report_lag_s: float = 0.04
    holes: tuple[int, ...] = (17, 400, 905)        # Unreal slots never rendered (missed slots)
    first_received: int = 5                         # the first publisher frame navpy decoded
    corrections: tuple[float, float] = (0.0, 0.0)   # the frozen C_a, C_g navpy ran with
    attitude_slip: tuple[float, float] | None = None    # (board time, extra Ds_a from then on)
    abort_after_s: float | None = None              # the map aborts this long into the run
    fixed_lens: bool = False                        # live gimbal association (no zoom refusal)
    seed: int = 3

    @property
    def true_attitude_c(self) -> float:
        return self.attitude_delay_s - self.video_delay_s

    @property
    def true_gimbal_c(self) -> float:
        return self.report_lag_s + self.gimbal_delay_s - self.video_delay_s - RENDER_LAG_S

    def receiver(self, board_s: float) -> float:
        return board_s * (1.0 + self.ppm * 1e-6) + EPOCH_R


@dataclass
class Logs:
    world: World
    config: IntegrationTimingConfig
    navpy: list[str] = field(default_factory=list)
    publisher: str = ""
    unreal: list[str] = field(default_factory=list)
    emulator: list[str] = field(default_factory=list)


def config_for(world: World) -> IntegrationTimingConfig:
    return IntegrationTimingConfig(run_id=world.run_id, mode="pc-sitl", source_id="rtsp://zr10/main",
                                   link_id="udp:127.0.0.1:14550", limits=IntegrationTimingLimits(),
                                   attitude_correction_s=world.corrections[0], gimbal_correction_s=world.corrections[1])


def _jitter(rng: np.random.Generator, mean_s: float) -> float:
    return float(rng.exponential(mean_s)) if mean_s > 0.0 else 0.0


def build(world: World) -> Logs:
    rng = np.random.default_rng(world.seed)
    logs = Logs(world, config_for(world))
    end_s = BOARD_START_S + world.seconds

    # Emulator: 20 ms plant steps on the device clock; one 0x0D push each, and a 0x18 reply every 7th step.
    first_step = int(math.floor((world.receiver(BOARD_START_S - 3.0) - EPOCH_G) / 0.02))
    last_step = int(math.ceil((world.receiver(end_s + 1.0) - EPOCH_G) / 0.02))
    emulator = [json.dumps({"event": "start", "session_id": SESSION})]
    gimbal: list[tuple[float, int, float]] = []          # (receipt, 16-bit seq, plant time)
    wire_seq = 65530                                     # wraps early in the run
    for step in range(first_step, last_step + 1):
        device_t = step * 0.02
        emulator.append(json.dumps({"event": "step", "seq": step, "t": device_t + 7.0, "device_t": device_t}))
        wire_seq = (wire_seq + 1) % 65536
        report_t = device_t - world.report_lag_s
        emulator.append(json.dumps({"event": "siyi_out", "seq": wire_seq, "cmd": "0d", "dest": ["127.0.0.1", 40001],
                                    "sent": True, "run_t": device_t + 7.0, "device_t": device_t,
                                    "report_t": report_t}))
        gimbal.append((device_t + EPOCH_G + world.gimbal_delay_s + _jitter(rng, world.gimbal_jitter_s), wire_seq,
                       report_t))
        if step % 7 == 0:
            wire_seq = (wire_seq + 1) % 65536
            emulator.append(json.dumps({"event": "siyi_out", "seq": wire_seq, "cmd": "18",
                                        "dest": ["127.0.0.1", 40001], "sent": True, "run_t": device_t + 7.0,
                                        "device_t": device_t}))
    logs.emulator = emulator
    gimbal.sort()

    # ATTITUDE at 50 Hz, time_boot cut to whole ms.
    attitude = []
    tb = BOARD_START_S - 3.0 + 0.0007
    while tb < end_s + 1.0:
        delay = world.attitude_delay_s
        if world.attitude_slip is not None and tb >= world.attitude_slip[0]:
            delay += world.attitude_slip[1]
        receipt = world.receiver(tb) + delay + _jitter(rng, world.attitude_jitter_s)
        attitude.append(SimpleNamespace(time_boot_s=math.floor(tb * 1000.0) / 1000.0, capture_receipt_s=receipt,
                                        attitude=Attitude(0.0, 0.0, 0.0), body_rates_rad_s=None))
        tb += 0.02
    attitude.sort(key=lambda sample: sample.capture_receipt_s)

    # Unreal slots and the publisher's pushed frames.
    slots = int(world.seconds * FPS)
    rows, frames = [PUBLISHER_COLUMNS], []
    frame_index, first_board_us = 0, None
    for slot in range(slots):
        if slot in world.holes:
            continue
        board_us = round((BOARD_START_S + slot / FPS) * 1e6)
        board = board_us / 1e6
        if first_board_us is None:
            first_board_us = board_us
        rendered = world.receiver(board) - EPOCH_G - RENDER_LAG_S
        older = int(math.floor(rendered / 0.02))
        weight = rendered / 0.02 - older
        logs.unreal.append(
            f"[2026.11.03-10.00.00:000][  0]LogTemp: Display: NavPyCaptureSlot outcome=sent sys_id=1 run={RUN} "
            f"gen=1 origin_us={first_board_us} slot={slot} board_us={board_us} frame={frame_index} crc=00000000 "
            f"capture_qpc=0.0 pose_seq=1,2 pose_w=0.5 pose_valid=1 mapped=1 offset_s=0.0 map_n=64 gimbal_qpc=0.0 "
            f"gimbal_session={SESSION} gimbal_seq={older},{older + 1} gimbal_w={weight:.6f} gimbal_valid=1 "
            f"gimbal_ypr=0,0,0 fov=60 cam_loc=0,0,0 cam_quat=0,0,0,1 lens=settled lens_reading=1.0 "
            f"lens_set_by=1.0 lens_basis=x")
        pts_ns = (board_us - first_board_us) * 1000
        rows.append(f"0,{frame_index},0.0,3,{board_us},1,3,10,10,1,{pts_ns},0,pushed,ok,0,")
        frames.append((frame_index, board_us, pts_ns, rendered))
        frame_index += 1
    logs.publisher = "\n".join(rows) + "\n"

    # navpy: one attempt per received frame, receipts logged before each attempt's record.
    config = logs.config
    events: list[tuple[float, int, dict]] = []
    for sample in attitude:
        events.append((sample.capture_receipt_s, 0, {"event": "attitude_receipt", "run_id": world.run_id,
                                                     "time_boot_s": sample.time_boot_s,
                                                     "capture_receipt_s": sample.capture_receipt_s}))
    for receipt, seq, _ in gimbal:
        events.append((receipt, 0, {"event": "gimbal_receipt", "run_id": world.run_id, "session": 1, "seq": seq,
                                    "receipt_s": receipt}))
    history_samples = [GimbalHistorySample(1, seq, receipt, Attitude(0.0, 0.0, 0.0)) for receipt, seq, _ in gimbal]
    gimbal_receipts = [receipt for receipt, _, _ in gimbal]
    attitude_receipts = [sample.capture_receipt_s for sample in attitude]
    first_pts = frames[world.first_received][2]
    for sequence, (index, board_us, pts_ns, rendered) in enumerate(frames[world.first_received:], 1):
        board = board_us / 1e6
        read = world.receiver(board) + world.video_delay_s + _jitter(rng, world.video_jitter_s)
        now = read + 0.005
        navpy_pts = int(round((pts_ns - first_pts) / TICK_NS) * TICK_NS)
        record = {"event": "association", "run_id": world.run_id, "authority": "integration_uncertified",
                  "now_s": now, "open_id": "open-1", "frame_sequence": sequence, "pts_ns": navpy_pts,
                  "attitude_bracket_max_s": ATTITUDE_BRACKET_S, "seam_tolerance_s": SEAM_TOLERANCE_S}
        if world.abort_after_s is not None and board - BOARD_START_S > world.abort_after_s:
            record["status"] = STATUS_ABORTED
            events.append((now, 1, record))
            continue
        if board - BOARD_START_S < 0.5:
            record["status"] = STATUS_UNSETTLED
            events.append((now, 1, record))
            continue
        mapped = world.receiver(board) + world.video_delay_s
        seen = bisect.bisect_right(attitude_receipts, now)
        snapshot = attitude[max(0, seen - 64):seen]
        g_seen = bisect.bisect_right(gimbal_receipts, now)
        g_snapshot = tuple(history_samples[max(0, g_seen - 64):g_seen])
        record.update(read_s=read, mapped_s=mapped, offset_s=world.video_delay_s, envelope_s=world.video_delay_s,
                      rate=0.0, attitude_lookup_s=mapped + config.attitude_correction_s,
                      attitude_history={"count": len(snapshot), "first_receipt_s": snapshot[0].capture_receipt_s,
                                        "last_receipt_s": snapshot[-1].capture_receipt_s},
                      gimbal_history={"session": 1, "count": len(g_snapshot),
                                      "first_receipt_s": g_snapshot[0].receipt_s,
                                      "last_receipt_s": g_snapshot[-1].receipt_s})
        found, status = interpolate_attitude_at(snapshot, mapped + config.attitude_correction_s,
                                                config.limits.window_s, bracket_max_width_s=ATTITUDE_BRACKET_S,
                                                seam_tolerance_s=SEAM_TOLERANCE_S)
        if found is None:
            record["status"] = status
            events.append((now, 1, record))
            continue
        record["attitude_time_boot_s"] = found.time_boot_s
        if not world.fixed_lens:
            record["status"] = STATUS_ZOOM_UNSETTLED
            events.append((now, 1, record))
            continue
        lookup = mapped + config.gimbal_correction_s
        record["gimbal_lookup_s"] = lookup
        history = GimbalCaptureHistory(1, g_snapshot, GimbalData(att=Attitude(0.0, 0.0, 0.0)))
        gimbal_found, status = interpolate_gimbal_at(history, lookup, config.limits.gimbal_bracket_max_s)
        if gimbal_found is None:
            record["status"] = status
        else:
            record.update(status="bracketed", gimbal_session=1,
                          gimbal_seqs=[gimbal_found.earlier.seq, gimbal_found.later.seq],
                          gimbal_receipts_s=[gimbal_found.earlier.receipt_s, gimbal_found.later.receipt_s],
                          gimbal_weight=gimbal_found.fraction)
        events.append((now, 1, record))
    events.sort(key=lambda event: (event[0], event[1]))
    logs.navpy = [json.dumps(record) for _, _, record in events]
    return logs


def held_out(world: World, fit: dict, **changes) -> World:
    """The same world, another run, with the fitted constants frozen."""
    fields = {"run_id": "obs-001", "seed": world.seed + 100,
              "corrections": (fit["attitude"]["correction_s"], fit["gimbal"]["correction_s"]), **changes}
    return replace(world, **fields)


def publisher_file(logs: Logs, folder) -> object:
    path = folder / f"frames-{logs.world.run_id}.csv"
    path.write_text(logs.publisher, encoding="utf-8")
    return path


def lines(text: str) -> list[str]:
    return io.StringIO(text).read().splitlines()
