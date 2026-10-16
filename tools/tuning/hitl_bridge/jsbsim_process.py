"""JSBSim program plant for the HITL bridge: /usr/games/JSBSim driven over its sockets (--plant program).

The owner's choice is the existing WSL JSBSim program (compiled from FlightGear 2020.3.18),
not the Python package. It is driven the way ArduPilot's SITL JSBSim backend drives it
(fork worktree libraries/SITL/SIM_JSBSim.cpp): "set <property> <value>" lines ending in
"iterate N" on the input socket (send_servos, :339-390), the state read back from an output
socket (recv_fdm, :416-452). The run folder is made by jsbsim_files, the lines are read and
written by jsbsim_protocol, the process is jsbsim_program. Differences, each from a check of
this binary (probe scripts in the session scratchpad, hitl/proc_check and hitl/bridge_fix):

- N equal integration steps per "iterate N", N from the board time step and the simulation
  rate with the remainder carried (board_clock.BoardClockStepper counts whole steps since the
  first frame); SIM_JSBSim sends "iterate 1" per frame.
- State output: a SOCKET output over UDP with an explicit property list (specific force =
  total body force / mass, ECEF-relative p/q/r, calibrated airspeed, ...; jsbsim_plant), not
  ArduPilot's FLIGHTGEAR FGNetFDM: in this binary FGNetFDM.cur_time is not the simulation
  time (probe: 1011093945, 1011093084, ... per step), so a packet cannot be tied to an
  iterate, and FGNetFDM has the eye-point acceleration, Euler rates and no true airspeed.
- An "iterate" sent while the program holds first runs one frame at the old time (the input
  is read inside a frame whose time does not advance), then the N steps: N+1 packets, the
  first stale (probe: iterate 6 -> frames 1,2,...,7 after holding at 1). The packet used is
  the one whose frame number is the target (old frame + N) and whose sim-time-sec equals
  frame * dt; the N before it are skipped, never used. In the stale frame the engine model
  also integrates once more (probe: the propeller RPM rose at an unchanged time),
  so the motor spins up (N+1)/N as fast (7/6 at 200 Hz board frames and 1200 Hz); SIM_JSBSim's
  "iterate 1" doubles it. The servo lag is gated on the time and has no extra step.
- --nice=1e-8 as SIM_JSBSim (:211): without it the program sleeps 10 ms per hold cycle
  (probe: iterate 6 turnaround median 10.4 ms; with it 0.60 ms). Holding then takes about
  10% of one CPU core (review measurement).

Run folder (fresh per plant, under the run root): aircraft/<Name>/ (the model copy with
reset.xml and the generated system), start.xml, output.xml, and jsbsim_<n>.log (the
program's console for its n-th start: a reset of the aircraft restarts the program).
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional, Sequence

from tools.tuning.hitl_bridge.environment import Wind
from tools.tuning.hitl_bridge.jsbsim_files import (FORCE_OUTPUT, JSBSimError, command_properties, new_run_dir,
                                                   prepare_run_folder, rate_hz_from_dt)
from tools.tuning.hitl_bridge.jsbsim_program import Launcher, ProgramConfig, ProgramIO, launch_program
from tools.tuning.hitl_bridge.jsbsim_protocol import (AHEAD, BEHIND, LABELS_PREFIX, REPLY_SET_OK, ReplyReader,
                                                      StateSample, batch_replies, expect_replies, expected_labels,
                                                      frame_relation, iterate_command, parse_labels,
                                                      parse_state_line, printed_time_tolerance_s)
from tools.tuning.hitl_bridge.physics import Origin, PlantState, SurfaceCommand
from tools.tuning.hitl_bridge.plant_options import PlantOptions, servo_lag_alpha

REPLY_TIMEOUT_S = 2.0                   # replies come in the next hold cycle (microseconds)
STATE_TIMEOUT_S = 2.0                   # 6 steps took 0.6 ms in the probe; 2 s means the program hangs
STARTUP_STATE_TIMEOUT_S = 10.0          # the label line comes once the model is loaded (0.05 s in the probe)
# Longest single "iterate": every step sends one datagram, and a burst must fit the
# receive buffer even if the bridge is slow to read (Linux default 212992 bytes, about
# 1 kB of buffer accounting per small datagram). 120 steps = 0.1 s at 1200 Hz.
MAX_STEPS_PER_ITERATE = 120
FIRST_START = 1                         # only the first start refuses other JSBSim programs


@dataclass
class ProgramStats:
    starts: int = 0
    iterates: int = 0
    steps: int = 0
    skipped_packets: int = 0        # packets before the target frame (the stale one and N-1 more): N per iterate


class JSBSimProcessPlant:
    """physics.Plant on the JSBSim program: steps are collected and sent as 'iterate N'.

    step() only counts; state() sends, for each run of equal inputs, the four command
    values and "iterate N" in one write, checks the replies, and waits for the packet of
    the target frame. reset() restarts the program unless it has not stepped yet.
    """

    def __init__(self, model_dir: Path, origin: Origin, dt_s: float, config: Optional[ProgramConfig] = None,
                 options: PlantOptions = PlantOptions(), launcher: Launcher = launch_program,
                 clock: Callable[[], float] = time.monotonic, log: Callable[[str], None] = print) -> None:
        model_dir = Path(model_dir)
        if not (model_dir / f"{model_dir.name}.xml").is_file():
            raise FileNotFoundError(f"no JSBSim model {model_dir / (model_dir.name + '.xml')}")
        self.config = config or ProgramConfig()
        self.origin = origin
        self.dt_s = dt_s
        self.rate_hz = rate_hz_from_dt(dt_s)
        self._launcher = launcher
        self._clock = clock
        self._log = log
        lag_alpha = servo_lag_alpha(dt_s, options.servo_speed_s)
        self.command_properties = command_properties(lag_alpha)
        self.run_dir = new_run_dir(Path(self.config.run_root), datetime.now(timezone.utc))
        prepare_run_folder(model_dir, self.run_dir, origin, (self.config.control_port, self.config.state_port),
                           dt_s, lag_alpha, options.start)
        self.stats = ProgramStats()
        self._io: Optional[ProgramIO] = None
        self._replies = ReplyReader()
        self._pending: list[list] = []          # [(command, force, wind), steps]
        self._frame = 0
        self._state: Optional[PlantState] = None
        self._stepped = False
        self._start()

    # -- program lifecycle
    def _start(self) -> None:
        self.stats.starts += 1
        self._replies = ReplyReader()
        self._io = self._launcher(self.config, self.run_dir, self.rate_hz, self.stats.starts,
                                  self.stats.starts == FIRST_START)
        try:
            self._handshake()
            self._stepped = False
            self._log(f"jsbsim: started (#{self.stats.starts}) in {self.run_dir}, frame {self._frame}")
        except BaseException:
            self._stop_program()
            raise

    def _stop_program(self) -> None:
        if self._io is not None:
            self._io.stop()
            self._io = None

    def _handshake(self) -> None:
        assert self._io is not None
        labels = self._io.read_datagram(STARTUP_STATE_TIMEOUT_S)
        if labels is None or not labels.startswith(LABELS_PREFIX):
            raise JSBSimError(f"no <LABELS> line from the state output (got {labels!r:.120})")
        if parse_labels(labels) != expected_labels():
            raise JSBSimError(f"state output columns {parse_labels(labels)} differ from {expected_labels()}: "
                              "a property is missing in this model or build")
        # The forced output is sent before the reply to the set, so once the reply is here
        # every packet up to the current frame is queued (loopback delivers in the send call).
        self._io.send(f"set {FORCE_OUTPUT} 0\n".encode())
        self._expect_replies([REPLY_SET_OK])
        latest = None
        while (datagram := self._io.read_datagram(0.0)) is not None:
            if not datagram.startswith(LABELS_PREFIX):
                latest = parse_state_line(datagram, self.origin)
        if latest is None:
            raise JSBSimError("no state packet after the forced output")
        self._frame = latest.frame
        self._state = latest.state

    # -- replies and packets
    def _expect_replies(self, wanted: Sequence[str]) -> None:
        assert self._io is not None
        try:
            expect_replies(self._io.read_replies, self._replies, wanted, self._clock() + REPLY_TIMEOUT_S,
                           self._clock)
        except JSBSimError as exc:
            raise JSBSimError(f"{exc}:\n{self._io.console_tail()}") from exc

    def _await_frame(self, target_frame: int) -> StateSample:
        assert self._io is not None
        deadline = self._clock() + STATE_TIMEOUT_S
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0 or not self._io.running():
                raise JSBSimError(f"no state for frame {target_frame} within {STATE_TIMEOUT_S} s "
                                  f"(program running: {self._io.running()}):\n{self._io.console_tail()}")
            datagram = self._io.read_datagram(remaining)
            if datagram is None or datagram.startswith(LABELS_PREFIX):
                continue
            sample = parse_state_line(datagram, self.origin)
            relation = frame_relation(sample.frame_mod, target_frame)
            if relation == BEHIND:
                self.stats.skipped_packets += 1
                continue
            if relation == AHEAD:
                raise JSBSimError(f"JSBSim is at frame {sample.frame} (mod {sample.frame_mod}), past the target "
                                  f"{target_frame}: the step count and the program disagree")
            target_time = target_frame * self.dt_s
            if abs(sample.sim_time_s - target_time) > printed_time_tolerance_s(target_time):
                raise JSBSimError(f"frame {target_frame} came with sim time {sample.sim_time_s} s, expected "
                                  f"{target_time} s: the program's step is not {self.dt_s} s")
            return sample

    def _iterate(self, inputs: tuple[SurfaceCommand, float, Wind], steps: int) -> None:
        assert self._io is not None
        self._io.send(iterate_command(self.command_properties, *inputs, steps))
        self._expect_replies(batch_replies(self.command_properties))
        sample = self._await_frame(self._frame + steps)
        self._frame += steps
        self._state = sample.state
        self.stats.iterates += 1
        self.stats.steps += steps

    # -- physics.Plant
    def reset(self) -> None:
        if not self._stepped and not self._pending:
            return                      # still at the initial conditions
        self._pending.clear()
        self._stop_program()
        self._start()

    def step(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind) -> None:
        inputs = (command, catapult_force_n, wind)
        if self._pending and self._pending[-1][0] == inputs and self._pending[-1][1] < MAX_STEPS_PER_ITERATE:
            self._pending[-1][1] += 1
        else:
            self._pending.append([inputs, 1])
        self._stepped = True

    def state(self) -> PlantState:
        if self._io is None:
            raise JSBSimError("the JSBSim program is not running")
        for inputs, steps in self._pending:
            self._iterate(inputs, steps)
        self._pending.clear()
        assert self._state is not None
        return self._state

    def close(self) -> None:
        if self._io is not None:
            self._stop_program()
            self._log(f"jsbsim: stopped; {self.stats}")
