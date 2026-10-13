"""JSBSim program plant of the HITL bridge (tools/tuning/hitl_bridge/jsbsim_process.py), without JSBSim.

FakeProgram answers on the plant's side of the sockets the way /usr/games/JSBSim
(FlightGear 2020.3.18) did in the probes: the real program's <LABELS> line first (copied
verbatim), "set successful" and "Iterations performed" replies with prompts, one stale packet
at the old frame before the N packets of an "iterate N", "Unknown property" ending a batch.
"""
from collections import deque
from pathlib import Path

import pytest

from tests.tools.tuning.test_hitl_bridge_files import make_model
from tests.tools.tuning.test_hitl_bridge_physics import commands_as_given
from tests.tools.tuning.test_hitl_bridge_protocol import REAL_LABELS_LINE, jsbsim_line
from tools.tuning.hitl_bridge import jsbsim_files as jf
from tools.tuning.hitl_bridge import jsbsim_process as jp
from tools.tuning.hitl_bridge.jsbsim_plant import N_PER_LBF, THROTTLE_CMD
from tools.tuning.hitl_bridge.jsbsim_program import ProgramConfig
from tools.tuning.hitl_bridge.environment import CALM, Wind
from tools.tuning.hitl_bridge.catapult import Catapult
from tools.tuning.hitl_bridge.board_clock import BoardClockStepper
from tools.tuning.hitl_bridge.physics import Origin, SurfaceCommand
from tools.tuning.hitl_bridge.plant_options import PlantOptions

ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.16523, ground_alt_msl_m=584.0, heading_deg=353.0)
DT_1200 = 1.0 / 1200.0
KNOWN_INPUTS = {jf.FORCE_OUTPUT, *jf.DIRECT_COMMAND_PROPERTIES, *jf.LAGGED_COMMAND_PROPERTIES}
NO_LAG = PlantOptions(servo_speed_s=0.0)


class FakeProgram:
    def __init__(self, dt_s=DT_1200, start_frame=1, labels=REAL_LABELS_LINE):
        self.dt_s = dt_s
        self.frame = start_frame
        self.frame_shift = 0            # >0: report frames ahead of the real count
        self.time_shift_s = 0.0
        self.swallow_iterate = False    # no packets after an iterate (a hung program)
        self.inputs = {}
        self.sent = []
        self.replies = bytearray(b"Connected to JSBSim server\nJSBSim> ")
        # as observed: the labels, then frames 0 and 1 from before the program held
        self.datagrams = deque([labels, self.packet(start_frame - 1), self.packet(start_frame)])
        self.stop_calls = 0

    def packet(self, frame: int) -> bytes:
        t = frame * self.dt_s
        values = {jf.SIM_TIME_S: t + self.time_shift_s, jf.FRAME: frame,
                  jf.FRAME_MOD: (frame + self.frame_shift) % jf.FRAME_MODULO, jf.DLAT_UDEG: frame * 0.01,
                  jf.DLON_UDEG: -frame * 0.02, "velocities/v-north-fps": frame * 0.1,
                  "velocities/vc-fps": frame * 0.1, "forces/fbz-total-lbs": -12.804,
                  "inertia/mass-slugs": 0.4}
        return jsbsim_line([t] + [values.get(name, 0.0) for name in jf.STATE_PROPERTIES])

    def _reply(self, text: str) -> None:
        self.replies += (text + "\nJSBSim> ").encode()

    # ProgramIO
    def send(self, data: bytes) -> None:
        self.sent.append(data)
        for line in data.decode().splitlines():
            command, *args = line.split()
            if command == "set":
                name, value = args
                if name not in KNOWN_INPUTS:
                    self._reply("Unknown property")
                    break                                    # JSBSim drops the rest of the batch
                if name == jf.FORCE_OUTPUT:
                    self.datagrams.append(self.packet(self.frame))
                else:
                    self.inputs[name] = float(value)
                self._reply("set successful")
            elif command == "iterate":
                self._reply("Iterations performed")
                if self.swallow_iterate:
                    continue
                self.datagrams.append(self.packet(self.frame))   # stale: old frame, before the steps
                for _ in range(int(args[0])):
                    self.frame += 1
                    self.datagrams.append(self.packet(self.frame))

    def read_replies(self, timeout_s):
        data = bytes(self.replies)
        self.replies.clear()
        return data

    def read_datagram(self, timeout_s):
        return self.datagrams.popleft() if self.datagrams else None

    def running(self):
        return self.stop_calls == 0

    def console_tail(self):
        return "(fake console)"

    def stop(self):
        self.stop_calls += 1


class Programs:
    """Launcher for the plant that hands out FakePrograms and remembers them."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.started = []
        self.programs = []

    def __call__(self, config, run_dir, rate_hz, start_index, check_other_programs):
        self.started.append((run_dir, rate_hz, start_index, check_other_programs))
        self.programs.append(FakeProgram(**self.kwargs))
        return self.programs[-1]


def make_plant(tmp_path: Path, dt_s=DT_1200, launcher=None, clock=None, options=NO_LAG, log=None):
    launcher = launcher or Programs(dt_s=dt_s)
    config = ProgramConfig(control_port=0, state_port=0, run_root=tmp_path / "runs")
    extra = {"clock": clock} if clock else {}
    plant = jp.JSBSimProcessPlant(make_model(tmp_path), ORIGIN, dt_s, config, options, launcher=launcher,
                                  log=log or (lambda _: None), **extra)
    return plant, launcher


def iterate_counts(program: FakeProgram) -> list[int]:
    return [int(line.split()[1]) for data in program.sent for line in data.decode().splitlines()
            if line.startswith("iterate")]


def test_start_reads_the_labels_and_the_forced_output_for_the_initial_state(tmp_path):
    plant, programs = make_plant(tmp_path)
    program = programs.programs[0]
    assert program.sent == [b"set simulation/force-output 0\n"]
    assert plant.state().lat_deg == pytest.approx(ORIGIN.lat_deg + 0.01e-6)       # frame 1
    assert programs.started[0][1:] == (pytest.approx(1200.0), 1, True)
    assert (plant.run_dir / "start.xml").is_file()


def test_iterate_counts_follow_the_board_clock_with_the_remainder_carried(tmp_path):
    plant, programs = make_plant(tmp_path, dt_s=1.0 / 1000.0)
    stepper = BoardClockStepper(plant, Catapult(0.0, 0.0, 0.0), mixer=commands_as_given)
    for k, t in enumerate(range(0, 5 * 3500, 3500)):              # 3.5 steps per frame at 1000 Hz
        stepper.on_actuators(t, SurfaceCommand(0.0, 0.0, 0.2), armed=False)
        state = plant.state()
    program = programs.programs[0]
    assert iterate_counts(program) == [3, 4, 3, 4]
    assert program.frame == 1 + 14
    assert state.lat_deg == pytest.approx(ORIGIN.lat_deg + 15 * 0.01e-6)         # the packet of frame 15
    # every packet before the target is skipped: the stale one and the N-1 between, N per iterate
    assert plant.stats.skipped_packets == 14 == plant.stats.steps
    assert program.inputs[THROTTLE_CMD] == 0.2


def test_200_hz_frames_at_1200_hz_are_six_steps_and_the_catapult_edges_split_a_frame(tmp_path):
    plant, programs = make_plant(tmp_path)
    # armed at 5 ms: force from 5 ms + 2.5 ms to 7.5 ms + 5 ms = steps 9..14 (step start times)
    stepper = BoardClockStepper(plant, Catapult(350.0, 0.005, 0.0025), mixer=commands_as_given)
    for k in range(5):
        stepper.on_actuators(k * 5000, SurfaceCommand(0.0, 0.0, 1.0), armed=k >= 1)
        plant.state()
    program = programs.programs[0]
    batches = [data.decode().splitlines() for data in program.sent[1:]]
    forces = [float(b[3].split()[2]) for b in batches]
    assert [int(b[6].split()[1]) for b in batches] == [6, 3, 3, 3, 3, 6]
    assert forces == pytest.approx([0, 0, 350 / N_PER_LBF, 350 / N_PER_LBF, 0, 0])
    assert program.frame == 1 + 24


def test_with_servo_lag_the_batch_sets_the_raw_inputs_of_the_generated_lag(tmp_path):
    plant, programs = make_plant(tmp_path, options=PlantOptions(servo_speed_s=0.14))
    plant.step(SurfaceCommand(0.25, -0.5, 1.0), 0.0, CALM)
    plant.state()
    lines = programs.programs[0].sent[1].decode().splitlines()
    assert lines == ["set hitl/aileron-cmd-raw 0.25", "set hitl/elevator-cmd-raw -0.5",
                     "set hitl/throttle-cmd-raw 1", "set external_reactions/catapult/magnitude 0",
                     "set atmosphere/wind-north-fps 0", "set atmosphere/wind-east-fps 0", "iterate 1"]
    assert "hitl/servo-gate" in (plant.run_dir / "aircraft" / "Peron15" / "navpy_hitl_output.xml").read_text()


def test_the_wind_reaches_the_program_with_the_steps_it_belongs_to(tmp_path):
    plant, programs = make_plant(tmp_path)
    stepper = BoardClockStepper(plant, Catapult(0.0, 0.0, 0.0), Wind(speed_m_s=3.048, from_deg=0.0),
                                mixer=commands_as_given)
    for t_us in (4_995_000, 5_000_000, 5_005_000):     # the wind starts at 5 s of board time
        stepper.on_actuators(t_us, SurfaceCommand(0.0, 0.0, 0.0), armed=False)
        plant.state()
    program = programs.programs[0]
    batches = [data.decode().splitlines() for data in program.sent[1:]]
    assert [(b[4], b[6]) for b in batches] == [("set atmosphere/wind-north-fps 0", "iterate 6"),
                                               ("set atmosphere/wind-north-fps -10", "iterate 6")]
    assert program.inputs["atmosphere/wind-north-fps"] == -10.0


def test_a_long_board_gap_is_sent_in_bounded_iterates(tmp_path):
    plant, programs = make_plant(tmp_path)
    stepper = BoardClockStepper(plant, Catapult(0.0, 0.0, 0.0), mixer=commands_as_given)
    stepper.on_actuators(0, SurfaceCommand(0.0, 0.0, 0.0), armed=False)
    stepper.on_actuators(300_000, SurfaceCommand(0.0, 0.0, 0.0), armed=False)     # 0.3 s = 360 steps
    plant.state()
    assert iterate_counts(programs.programs[0]) == [jp.MAX_STEPS_PER_ITERATE] * 3


def test_a_refused_command_stops_with_the_reply(tmp_path):
    plant, programs = make_plant(tmp_path)
    plant.command_properties = ("fcs/no-such-input",) + plant.command_properties[1:]
    plant.step(SurfaceCommand(0.0, 0.0, 0.0), 0.0, CALM)
    with pytest.raises(jf.JSBSimError, match="Unknown property"):
        plant.state()


def test_a_packet_past_the_target_or_with_the_wrong_time_is_an_error(tmp_path):
    plant, programs = make_plant(tmp_path)
    # Reported 2 frames ahead, the stale packet of "iterate 1" (frame 1) reads as frame 3, past
    # the target 2. (With a shift of 1 it would read as the target and fail the time check.)
    programs.programs[0].frame_shift = 2
    plant.step(SurfaceCommand(0.0, 0.0, 0.0), 0.0, CALM)
    with pytest.raises(jf.JSBSimError, match="past the target"):
        plant.state()
    plant, programs = make_plant(tmp_path / "b")
    programs.programs[0].time_shift_s = 0.001                     # a program running another step size
    plant.step(SurfaceCommand(0.0, 0.0, 0.0), 0.0, CALM)
    with pytest.raises(jf.JSBSimError, match="step is not"):
        plant.state()


def test_no_packet_for_the_target_frame_times_out(tmp_path):
    ticks = iter(range(1000))
    plant, programs = make_plant(tmp_path, clock=lambda: next(ticks) * 0.5)
    programs.programs[0].swallow_iterate = True
    plant.step(SurfaceCommand(0.0, 0.0, 0.0), 0.0, CALM)
    with pytest.raises(jf.JSBSimError, match="no state for frame 2"):
        plant.state()


def test_wrong_output_columns_are_refused_and_the_program_stopped(tmp_path):
    launcher = Programs(labels=REAL_LABELS_LINE.replace(b",mass-slugs", b""))
    with pytest.raises(jf.JSBSimError, match="columns"):
        make_plant(tmp_path, launcher=launcher)
    assert launcher.programs[0].stop_calls == 1


def test_a_failing_start_log_line_still_stops_the_program(tmp_path):
    def broken_log(text):
        raise BrokenPipeError("the runner's pipe is gone")
    launcher = Programs()
    with pytest.raises(BrokenPipeError):
        make_plant(tmp_path, launcher=launcher, log=broken_log)
    assert launcher.programs[0].stop_calls == 1


def test_reset_restarts_the_program_only_after_it_stepped_and_close_stops_it(tmp_path):
    plant, programs = make_plant(tmp_path)
    plant.reset()                                                 # first frame: still at the start
    assert len(programs.programs) == 1
    plant.step(SurfaceCommand(0.0, 0.0, 0.0), 0.0, CALM)
    plant.state()
    plant.reset()                                                 # board reboot: a fresh program
    assert len(programs.programs) == 2 and programs.programs[0].stop_calls == 1
    # only the first start looks for other JSBSim programs; a restart checks only its ports
    assert [(s[2], s[3]) for s in programs.started] == [(1, True), (2, False)]
    assert plant.state().lat_deg == pytest.approx(ORIGIN.lat_deg + 0.01e-6)
    plant.close()
    plant.close()
    assert programs.programs[1].stop_calls == 1
    with pytest.raises(jf.JSBSimError, match="not running"):
        plant.state()


def test_a_missing_model_is_refused_before_anything_starts(tmp_path):
    launcher = Programs()
    with pytest.raises(FileNotFoundError):
        jp.JSBSimProcessPlant(tmp_path / "Nope", ORIGIN, DT_1200, ProgramConfig(run_root=tmp_path),
                              launcher=launcher)
    assert launcher.started == []
