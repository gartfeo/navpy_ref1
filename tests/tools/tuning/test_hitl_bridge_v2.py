"""HIL link contract v2 conformance of the HITL JSBSim bridge (tools/tuning/hitl_bridge), without a board.

- Byte-for-byte replies against the fork's reference bridge (hil_v2_reference_vectors.json, recorded
  from Tools/autotest/pysim/hil_bridge.py with its static model at the same origin): every event of
  the contract (first, step, gap, retransmit, conflict, reinit by a jump and by time going back),
  the frames it must ignore (MAVLink 1, its own system id, other messages, a bad CRC) and the
  lockstep flag.
- Clock determinism (the owner's rule and the contract's rule zero): the same board frames give
  bit-identical replies, whatever the read chunking; the physics path reads no PC clock and draws
  no random number (patched to raise), and its modules import neither.
- The frame loop's handling of a reply lost with a reconnect, a conflicting frame, a reboot while
  armed and the lockstep flag; the serial port is flushed when it is opened.
"""
import ast
import contextlib
import csv
import io
import itertools
import json
import math
import os
import random
import time
from pathlib import Path
from unittest import mock

import pytest
from pymavlink.dialects.v20 import common as mavlink2

from tests.tools.tuning.test_hitl_bridge_link import FakeTransport, actuator_frame, make_link
from tools.tuning.hitl_bridge import transport as transport_module
from tools.tuning.hitl_bridge.catapult import Catapult
from tools.tuning.hitl_bridge.environment import Wind
from tools.tuning.hitl_bridge.frame_loop import LOG_COLUMNS, Bridge, LinkLog, run_bridge
from tools.tuning.hitl_bridge.link import BRIDGE_SYSID, LOCKSTEP_FLAG
from tools.tuning.hitl_bridge.board_clock import CONFLICT, GAP, REINIT, RETRANSMIT, STEP, BoardClockStepper
from tools.tuning.hitl_bridge.physics import PlantState, SurfaceCommand
from tools.tuning.hitl_bridge.state_fields import STANDARD_GRAVITY_M_S2

VECTORS = json.loads((Path(__file__).with_name("hil_v2_reference_vectors.json")).read_text())
DT_1200 = 1.0 / 1200.0
LOOP_US = 5000
T0_US = 1_000_000
# seed of the loop jitter in the recorded session (drawn before any guard is patched in)
REPLAY_SEED = 20261013
PHYSICS_PATH_MODULES = ("physics", "board_clock", "state_fields", "environment", "catapult", "rail", "plant_options",
                        "frame_loop", "jsbsim_protocol", "jsbsim_plant")
PC_CLOCK_MODULES = {"time", "datetime"}
RANDOM_MODULES = {"random", "secrets", "uuid"}
NEUTRAL = (1500, 1500, 1000)


# --- plants -----------------------------------------------------------------------------------
class StaticOriginPlant:
    """The reference bridge's static model: level at the origin on its heading, at rest."""
    dt_s = DT_1200

    def __init__(self, origin: dict) -> None:
        self._state = PlantState(lat_deg=origin["lat_deg"], lon_deg=origin["lon_deg"], alt_msl_m=origin["alt_m"],
                                 roll_rad=0.0, pitch_rad=0.0, yaw_rad=math.radians(origin["heading_deg"]),
                                 p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0, vn_m_s=0.0, ve_m_s=0.0, vd_m_s=0.0,
                                 ias_m_s=0.0, tas_m_s=0.0, fx_m_s2=0.0, fy_m_s2=0.0,
                                 fz_m_s2=-STANDARD_GRAVITY_M_S2)
        self.resets = 0

    def reset(self) -> None:
        self.resets += 1

    def step(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind) -> None:
        pass

    def state(self) -> PlantState:
        return self._state

    def close(self) -> None:
        pass


class IntegratingPlant:
    """A deterministic toy plant whose state depends on every input of every step (commands, the
    launch force, the wind), so a changed step order or count shows in the replies."""
    dt_s = DT_1200

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.speed = self.roll = self.pitch = self.yaw = self.north = self.east = self.up = 0.0

    def step(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind) -> None:
        wind_n, wind_e = wind.air_velocity_ne_m_s()
        dt = self.dt_s
        self.speed += (20.0 * command.throttle + 0.01 * catapult_force_n - 0.3 * self.speed) * dt
        self.roll += (math.radians(40.0) * command.aileron - self.roll) * 3.0 * dt
        self.pitch += (math.radians(-15.0) * command.elevator - self.pitch) * 3.0 * dt
        self.yaw = (self.yaw + 9.80665 * math.tan(self.roll) / max(self.speed, 1.0) * dt) % (2 * math.pi)
        self.north += (self.speed * math.cos(self.yaw) + wind_n) * dt
        self.east += (self.speed * math.sin(self.yaw) + wind_e) * dt
        self.up += self.speed * math.sin(self.pitch) * dt

    def state(self) -> PlantState:
        return PlantState(lat_deg=-35.0 + self.north * 9e-6, lon_deg=149.0 + self.east * 1.1e-5,
                          alt_msl_m=584.0 + self.up, roll_rad=self.roll, pitch_rad=self.pitch, yaw_rad=self.yaw,
                          p_rad_s=self.roll * 0.1, q_rad_s=self.pitch * 0.1, r_rad_s=0.0,
                          vn_m_s=self.speed * math.cos(self.yaw), ve_m_s=self.speed * math.sin(self.yaw),
                          vd_m_s=-self.speed * math.sin(self.pitch), ias_m_s=self.speed * 0.97,
                          tas_m_s=self.speed, fx_m_s2=0.1 * self.speed, fy_m_s2=0.0,
                          fz_m_s2=-STANDARD_GRAVITY_M_S2 * math.cos(self.roll))

    def close(self) -> None:
        pass


# --- running the frame loop over scripted reads -------------------------------------------------
def run(reads, plant, catapult=None, wind=None, guard=contextlib.nullcontext):
    """The bridge over scripted transport reads (bytes, or an OSError for a lost port) until they are
    used up. Returns the bytes written to the board, the log rows, the console lines, the stepper and
    the link. The log clock is a counter: the PC clock never enters, not even the log. The link is
    connected first (opening a port is I/O, where the PC clock is allowed: pymavlink stamps a new
    codec), then the frames are served inside guard()."""
    transport = FakeTransport(reads=reads)
    link = make_link(transport, [])
    assert link.connect(lambda: False)
    stepper = BoardClockStepper(plant, catapult or Catapult(0.0, 0.0, 0.0), wind or Wind(0.0, 0.0))
    stream = io.StringIO()
    ticks = itertools.count()
    said: list[str] = []
    with guard():
        run_bridge(link, Bridge(stepper), LinkLog(stream), lambda: next(ticks) * 0.001,
                   should_stop=lambda: not transport.reads, say=said.append)
    rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
    return bytes(transport.written), rows, said, stepper, link


def parse(data: bytes) -> list:
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    return parser.parse_buffer(data) or []


def recorded_session() -> list[bytes]:
    """A fixed board session: 200 Hz with seeded loop jitter, throttle up then a turn and a pull-up
    while armed, three lost frames, a reboot (board time back) and a short second boot."""
    rng = random.Random(REPLAY_SEED)
    frames, t_us = [], T0_US
    for k in range(1600):
        if k < 100:
            pwm = NEUTRAL
        elif k < 700:
            pwm = (1500, 1500, 2000)
        elif k < 1100:
            pwm = (1600, 1800, 2000)             # right roll and nose up through the elevon unmix
        else:
            pwm = (1450, 1650, 1600)
        frames.append(actuator_frame(t_us, armed=k >= 100, pwm=pwm))
        t_us += LOOP_US + rng.randint(-300, 300)
        if k == 500:
            t_us += 3 * LOOP_US                 # frames lost
    t_us = 2 * T0_US
    for _ in range(100):
        frames.append(actuator_frame(t_us))
        t_us += LOOP_US
    return frames


def replay(frames: list[bytes], chunk: int | None = None,
           guard=contextlib.nullcontext) -> tuple[bytes, list, BoardClockStepper]:
    data = b"".join(frames)
    reads = list(frames) if chunk is None else [data[i:i + chunk] for i in range(0, len(data), chunk)]
    written, rows, _, stepper, _ = run(reads, IntegratingPlant(), Catapult(150.0, 0.3, 0.5),
                                       Wind(speed_m_s=4.0, from_deg=250.0), guard)
    return written, rows, stepper


# --- conformance against the reference bridge ---------------------------------------------------
def test_replies_are_byte_identical_to_the_reference_bridge():
    chunks = VECTORS["chunks"]
    written, rows, said, stepper, link = run([bytes.fromhex(c["frame_hex"]) for c in chunks],
                                             StaticOriginPlant(VECTORS["origin"]))
    want = bytes.fromhex("".join(c["reply_hex"] for c in chunks))
    assert written == want                      # same fields, units, ids, sequence numbers and CRCs
    assert [r["event"] for r in rows] == [c["event"] for c in chunks if c["event"] is not None]
    counters = VECTORS["counters"]
    assert (len(rows), stepper.retransmits, stepper.conflicts, stepper.reinits, stepper.armed_reinits,
            stepper.gaps) == (counters["frames"], counters["retransmits"], counters["conflicts"],
                              counters["reinits"], counters["armed_reinits"], counters["gaps"])
    assert (link.stats.rx_mavlink1, link.stats.rx_loopback, link.stats.rx_other) == (
        counters["mavlink1"], counters["loopback"], counters["other"])
    assert link.stats.rx_bad > 0
    replies = parse(written)
    assert all((m.get_srcSystem(), m.get_srcComponent()) == (BRIDGE_SYSID, mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER)
               and m.get_msgbuf()[0] == mavlink2.PROTOCOL_MARKER_V2 for m in replies)


def test_every_answered_frame_is_answered_once_with_its_own_time_in_order():
    chunks = VECTORS["chunks"]
    written, rows, *_ = run([bytes.fromhex(c["frame_hex"]) for c in chunks], StaticOriginPlant(VECTORS["origin"]))
    assert [m.time_usec for m in parse(written)] == [int(r["board_time_us"]) for r in rows]
    assert [r["frame"] for r in rows] == [str(k) for k in range(1, len(rows) + 1)]
    assert list(rows[0]) == list(LOG_COLUMNS) and rows[-1]["flags"] == str(LOCKSTEP_FLAG)


# --- determinism --------------------------------------------------------------------------------
def test_the_same_board_frames_give_bit_identical_states():
    frames = recorded_session()
    first, rows, stepper = replay(frames)
    second, _, _ = replay(frames)
    assert first == second
    # the session exercised what it was meant to: one armed reboot, a gap, the launch, a turn
    assert (stepper.reinits, stepper.armed_reinits) == (1, 1) and stepper.gaps >= 1
    replies = parse(first)
    assert len(replies) == len(frames) == len(rows)
    assert any(m.alt > 584_000 + 5_000 for m in replies) and any(abs(m.rollspeed) > 0.01 for m in replies)


@pytest.mark.parametrize("chunk", [1, 7, 93, 4096])
def test_the_states_do_not_depend_on_how_the_bytes_arrive(chunk):
    frames = recorded_session()
    assert replay(frames, chunk=chunk)[0] == replay(frames)[0]


def _raise(*args, **kwargs):
    raise AssertionError("the physics path read the PC clock or drew a random number")


@contextlib.contextmanager
def no_pc_clock():
    """Every PC clock of the time module raises (frames are parsed, stepped, answered meanwhile)."""
    clocks = ("time", "time_ns", "monotonic", "monotonic_ns", "perf_counter", "perf_counter_ns",
              "process_time", "process_time_ns", "thread_time", "thread_time_ns")
    with mock.patch.multiple(time, **{name: mock.DEFAULT for name in clocks if hasattr(time, name)}) as patched:
        for fake in patched.values():
            fake.side_effect = _raise
        yield


def test_the_physics_path_never_reads_the_pc_clock():
    frames = recorded_session()
    reference, _, _ = replay(frames)
    assert replay(frames, guard=no_pc_clock)[0] == reference


def test_the_physics_path_draws_no_random_numbers():
    frames = recorded_session()                 # the seeded jitter is drawn before the patches
    reference, _, _ = replay(frames)
    functions = ("random", "randint", "randrange", "uniform", "gauss", "normalvariate", "choice", "choices",
                 "shuffle", "sample", "getrandbits", "seed")
    patches = [mock.patch.object(random, name, _raise) for name in functions]
    patches += [mock.patch.object(cls, name, _raise) for cls in (random.Random, random.SystemRandom)
                for name in ("random", "getrandbits")]
    patches += [mock.patch.object(os, "urandom", _raise)]
    @contextlib.contextmanager
    def no_random():
        with contextlib.ExitStack() as stack:
            for patch in patches:
                stack.enter_context(patch)
            yield
    assert replay(frames, guard=no_random)[0] == reference


@pytest.mark.parametrize("module", PHYSICS_PATH_MODULES)
def test_physics_path_modules_import_no_clock_and_no_randomness(module):
    """A static guard next to the runtime ones: the modules that compute the states import no PC
    clock and no random source (the PC clock stays in link, transport and the command line)."""
    path = Path(transport_module.__file__).with_name(f"{module}.py")
    imported = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
        elif isinstance(node, ast.Attribute) and node.attr == "urandom":
            imported.add("os.urandom")
    assert not imported & (PC_CLOCK_MODULES | RANDOM_MODULES | {"os.urandom"}), (module, imported)


# --- retransmission, conflicts, reboots, lockstep through the frame loop ---------------------
def test_a_reply_lost_with_a_reconnect_gives_the_uninterrupted_run():
    """lockstep SITL resends the frame whose reply was lost with the connection: the flying aircraft
    carries on (no reset), and the replies are the uninterrupted run's"""
    frames = [actuator_frame(T0_US + k * LOOP_US, armed=True, pwm=(1500, 1500, 2000), flags=LOCKSTEP_FLAG)
              for k in range(400)]
    lost = 300
    reference, _, _, _, _ = run(list(frames), IntegratingPlant())
    reads = frames[:lost + 1] + [OSError("connection reset")] + frames[lost:]
    transport = FakeTransport(reads=reads)
    link = make_link(transport, [])
    assert link.connect(lambda: False)
    stepper = BoardClockStepper(IntegratingPlant(), Catapult(0.0, 0.0, 0.0))
    said: list[str] = []
    stream = io.StringIO()
    ticks = itertools.count()
    real_send = link.send_state
    sends = itertools.count()

    def send_losing_one(time_usec, fields):           # the reply to frame `lost` never arrives
        if next(sends) == lost:
            return False
        return real_send(time_usec, fields)
    link.send_state = send_losing_one
    run_bridge(link, Bridge(stepper), LinkLog(stream), lambda: next(ticks) * 0.001,
               should_stop=lambda: not transport.reads, say=said.append)
    got = [m.to_dict() for m in parse(bytes(transport.written))]
    assert got == [m.to_dict() for m in parse(reference)]     # the lost one answered when resent
    assert (stepper.retransmits, stepper.reinits, stepper.armed_reinits) == (1, 0, 0)
    assert any("again" in line for line in said) and not any("ALERT" in line for line in said)
    assert (link.stats.connects, link.stats.disconnects) == (2, 1)


def test_a_conflicting_frame_is_an_error_answered_with_the_state_at_that_time():
    frames = [actuator_frame(T0_US, armed=True), actuator_frame(T0_US + LOOP_US, armed=True, pwm=(1500, 1500, 2000)),
              actuator_frame(T0_US + LOOP_US, armed=True)]
    written, rows, said, stepper, _ = run(frames, IntegratingPlant())
    replies = parse(written)
    assert [m.time_usec for m in replies] == [T0_US, T0_US + LOOP_US, T0_US + LOOP_US]
    assert replies[1].to_dict() == replies[2].to_dict()
    assert [r["event"] for r in rows] == ["first", STEP, CONFLICT] and stepper.conflicts == 1
    assert any("ERROR" in line and "contract" in line for line in said)


def test_a_reboot_while_armed_is_an_alert_and_a_disarmed_one_is_not():
    frames = [actuator_frame(T0_US, armed=True), actuator_frame(T0_US + LOOP_US, armed=True),
              actuator_frame(300_000), actuator_frame(305_000), actuator_frame(100_000)]
    _, rows, said, stepper, _ = run(frames, IntegratingPlant())
    assert [r["event"] for r in rows] == ["first", STEP, REINIT, STEP, REINIT]
    alerts = [line for line in said if "ALERT" in line]
    assert len(alerts) == 1 and "ARMED" in alerts[0] and "went back" in alerts[0] and "invalid" in alerts[0]
    assert sum("board reboot" in line for line in said) == 2 and stepper.armed_reinits == 1


def test_a_board_reboot_through_a_reopened_port_restarts_the_aircraft():
    """A rebooting board drops its USB port; the link reopens it and the first frame of the new
    boot (board time back near 0) puts the aircraft back at the origin."""
    before = [actuator_frame(40_000_000 + k * LOOP_US, armed=False, pwm=(1500, 1500, 2000)) for k in range(50)]
    after = [actuator_frame(1_500_000 + k * LOOP_US) for k in range(3)]
    plant = IntegratingPlant()
    _, rows, said, stepper, link = run(before + [OSError("device disconnected")] + after, plant)
    assert [r["event"] for r in rows[-3:]] == [REINIT, STEP, STEP]
    assert stepper.reinits == 1 and plant.speed == 0.0
    assert (link.stats.connects, link.stats.disconnects) == (2, 1)


def test_the_lockstep_flag_is_reported_once_and_a_gap_is_reported():
    frames = [actuator_frame(T0_US + k * LOOP_US, flags=LOCKSTEP_FLAG) for k in range(3)]
    frames.append(actuator_frame(T0_US + 7 * LOOP_US, flags=LOCKSTEP_FLAG))
    _, rows, said, _, _ = run(frames, IntegratingPlant())
    assert sum("lockstep" in line for line in said) == 1
    assert rows[-1]["event"] == GAP and rows[-1]["lost_frames"] == "4"   # 25 ms after the last
    assert any("GAP" in line and "about 4 frames lost" in line for line in said)


def test_the_retransmitted_frame_is_answered_with_the_same_state():
    frames = [actuator_frame(T0_US + k * LOOP_US, pwm=(1500, 1500, 2000)) for k in range(50)]
    written, rows, *_ = run(frames + [frames[-1]], IntegratingPlant())
    replies = parse(written)
    assert rows[-1]["event"] == RETRANSMIT and rows[-1]["substeps"] == "0"
    assert replies[-1].to_dict() == replies[-2].to_dict()


# --- link and transport -------------------------------------------------------------------------
def test_mavlink1_and_loopback_frames_are_ignored_and_counted():
    own = mavlink2.MAVLink(None, srcSystem=BRIDGE_SYSID, srcComponent=mavlink2.MAV_COMP_ID_AUTOPILOT1)
    link = make_link(FakeTransport(reads=[actuator_frame(5000, mavlink1=True) + actuator_frame(5000, sender=own)
                                          + actuator_frame(10000)]), [])
    assert [m.time_usec for m in link.receive(lambda: False)] == [10000]
    assert (link.stats.rx_mavlink1, link.stats.rx_loopback, link.stats.rx_actuators) == (1, 1, 1)


def test_the_serial_port_is_flushed_when_it_is_opened(monkeypatch):
    opened = []

    class FakeSerial:
        def __init__(self, port, baud, timeout, write_timeout):
            self.calls = ["open"]
            opened.append(self)

        def reset_input_buffer(self):
            self.calls.append("flush")

        def close(self):
            self.calls.append("close")

    monkeypatch.setattr(transport_module.serial, "Serial", FakeSerial)
    port = transport_module.SerialTransport("COM11")
    port.open()
    port.close()
    port.open()                                 # a reopen after a board reboot flushes again
    assert [s.calls for s in opened] == [["open", "flush", "close"], ["open", "flush"]]

