"""Physics time on the flight controller's clock (HIL link contract v2, "Timing and determinism").

BoardClockStepper advances a physics.Plant on the board time of each HIL_ACTUATOR_CONTROLS frame:
whole fixed integration steps since the first frame, the contract's causal zero-order hold, the
launch force and the wind (environment.py) by the board time of each step, and the contract's
reboot, GAP, retransmission and conflict rules, and the crash rule (crash_reason). Pure logic: it never reads the PC clock and draws
no random numbers, so the same frames give bit-identical states.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Callable, Optional

from tools.tuning.hitl_bridge.environment import CALM, US_PER_S, Wind, wind_at
from tools.tuning.hitl_bridge.physics import NEUTRAL_COMMAND, Launch, Plant, PlantState, SurfaceCommand, unmix_elevon
from tools.tuning.hitl_bridge.state_fields import at_rest

# --- Stepping on the board clock --------------------------------------------------
# The plant moves on the grid t_0 + n / rate (contract v2, "Timing and determinism"). The step
# is 1/N s for a whole N (the JSBSim program runs at --simulation-rate=N), so the grid is
# counted in integer arithmetic: (t - t_0) * N // 1e6 whole steps, exact for any board time.
# A float step is accepted when N * dt is 1 to within this (1/1200 s is not exact in binary).
STEP_RATE_TOLERANCE = 1e-9
# The board's SCHED_LOOP_RATE: the HIL link and the reference bridge are sized for 200 Hz
# (fork Tools/autotest/default_params/plane-hil.parm). Only the GAP report depends on it.
DEFAULT_LOOP_RATE_HZ = 200.0
# One lost frame doubles the interval between frames; 1.5 nominal periods leave half a period
# of loop jitter before an interval counts as a GAP (the reference bridge's GAP_FACTOR).
GAP_FACTOR = 1.5
# A running board never goes this long between frames (it declares its link LOST after 50 ms):
# a longer forward jump is a new boot, re-initialised at the origin rather than stepped across
# with outputs held for that long (the reference bridge's MAX_FORWARD_JUMP_US).
MAX_FORWARD_JUMP_US = 2_000_000

FIRST = "first"
REINIT = "reinit"
STEP = "step"
GAP = "gap"
# The same board time as the previous frame: the same frame again (a lockstep SITL resends a
# frame whose reply was lost with a connection) or other content at that time (a broken
# board: board time strictly increases within one boot). Neither is stepped.
RETRANSMIT = "retransmit"
CONFLICT = "conflict"
# The plant left the physical envelope (crash_reason): the aircraft hit the ground or the integration
# diverged (the bench run of 2026-10-13: after a ground strike JSBSim gave body rates no float32 holds,
# and packing them ended the bridge). From that frame on the plant is no longer stepped and every
# frame is answered with the crash state (state_fields.at_rest of the last state inside the envelope)
# until the board reboots (REINIT re-initialises the plant).
CRASH = "crash"
# Body rates beyond this are not an aircraft: the board rejects a state with them as corrupt
# (fork libraries/SITL/SIM_HIL_Link.h MAX_BODY_RATE_RADS, 16 revolutions per second).
MAX_BODY_RATE_RAD_S = 100.0
# The model's reference point this far under the ground means its contacts no longer carry it: at rest
# on its skids it is centimetres above the ground (the launch profile's ground_rest), and a hard landing
# compresses the contacts by centimetres. A metre under the ground is a crash, not a landing.
GROUND_PENETRATION_CRASH_M = 1.0

Mixer = Callable[[Any], SurfaceCommand]


def crash_reason(state: PlantState, ground_alt_msl_m: Optional[float]) -> str:
    """Why the state is outside the physical envelope ('' when it is inside): a value that is not a finite
    number, body rates over MAX_BODY_RATE_RAD_S, or (with the ground's elevation) the reference point more
    than GROUND_PENETRATION_CRASH_M under the ground."""
    bad = [f.name for f in fields(state) if not math.isfinite(getattr(state, f.name))]
    if bad:
        return f"not a finite number: {', '.join(bad)}"
    rate = max(abs(state.p_rad_s), abs(state.q_rad_s), abs(state.r_rad_s))
    if rate > MAX_BODY_RATE_RAD_S:
        return f"body rate {rate:.4g} rad/s, over {MAX_BODY_RATE_RAD_S:g}"
    if ground_alt_msl_m is not None and state.alt_msl_m < ground_alt_msl_m - GROUND_PENETRATION_CRASH_M:
        return f"{ground_alt_msl_m - state.alt_msl_m:.1f} m under the ground"
    return ""


def step_rate_hz(dt_s: float) -> int:
    """The whole number of integration steps per second for a step of dt_s; ValueError if 1/dt_s is not whole."""
    if not dt_s > 0:
        raise ValueError(f"integration step {dt_s} s must be positive")
    rate = round(1.0 / dt_s)
    if rate < 1 or abs(rate * dt_s - 1.0) > STEP_RATE_TOLERANCE:
        raise ValueError(f"integration step {dt_s} s is not 1/N s for a whole N")
    return rate


@dataclass(frozen=True)
class StepResult:
    event: str                  # FIRST, REINIT, STEP, GAP (a STEP across lost frames), RETRANSMIT or CONFLICT
    board_dt_us: int            # board time since the previous frame (0 unless STEP or GAP)
    substeps: int               # integration steps taken for this frame
    lost_frames: int            # estimated from the interval against the nominal loop, GAP only
    catapult_force_n: float     # force applied on the last substep
    on_launcher: bool = False   # the launcher still holds the aircraft: the plant was not stepped yet
    was_armed: bool = False     # the previous frame had the armed bit
    previous_time_us: int = -1  # board time of the previous frame (-1 on FIRST)
    crash: str = ""             # CRASH only: why the plant left the envelope (crash_reason)


class BoardClockStepper:
    """Advance the plant on the board clock, outputs held between frames (zero-order hold).

    Contract v2 (fork libraries/SITL/examples/HIL/readme.md), as the reference bridge
    (Tools/autotest/pysim/hil_bridge.py BoardClockStepper):
    - the first frame sets t_0 and latches u_0; the reply is the start state;
    - the plant time is a whole number of fixed steps since t_0, and the state that answers
      frame k is the state at the last grid point at or before t_k (less than one step early);
    - each step uses the outputs of the newest frame at or before its start: u_k takes effect
      at the first grid point at or after t_k (less than one step late), a step that straddles
      t_k still uses u_(k-1), and of several frames before one grid point only the newest acts;
    - board time going back, or jumping forward by more than max_jump_us, is a new boot: the
      plant is re-initialised at the origin (counted in armed_reinits when the previous frame
      was armed);
    - a forward jump longer than GAP_FACTOR nominal loop periods is stepped across with the
      held outputs and reported as a GAP;
    - the same board time again: RETRANSMIT when the frame (outputs, armed bit, flags) is the
      same, CONFLICT otherwise; neither steps, latches or re-initialises anything.
    Nothing here reads the PC clock or draws a random number.

    The frame's controls are kept as they came to identify the frame, and turned into the
    plant's surface commands by mixer (the elevon unmix by default) when latched.
    While a launcher holds the aircraft (Launch.holding) the board time passes without plant
    steps: the plant's own time starts when the launcher lets go, and until then the board
    reads the launcher's state (board_state).
    After a crash (crash_reason, checked after every frame's steps) the plant is not stepped again:
    board time and outputs are still taken, and the board reads the crash state, until a new boot.
    """

    def __init__(self, plant: Plant, catapult: Launch, wind: Wind = CALM, mixer: Mixer = unmix_elevon,
                 loop_rate_hz: float = DEFAULT_LOOP_RATE_HZ, max_jump_us: int = MAX_FORWARD_JUMP_US,
                 ground_alt_msl_m: Optional[float] = None) -> None:
        if not loop_rate_hz > 0:
            raise ValueError(f"loop rate {loop_rate_hz} Hz must be positive")
        self.plant = plant
        self.catapult = catapult
        self.wind = wind
        self.mixer = mixer
        self.rate_hz = step_rate_hz(plant.dt_s)
        self.nominal_loop_us = US_PER_S / float(loop_rate_hz)
        self.gap_threshold_us = GAP_FACTOR * self.nominal_loop_us
        self.max_jump_us = max_jump_us
        self.t0_us: Optional[int] = None
        self.last_us: Optional[int] = None
        self.last_frame: Optional[tuple[Any, bool, int]] = None
        self.held = NEUTRAL_COMMAND                                 # the command the next step uses
        self.pending: Optional[tuple[int, SurfaceCommand]] = None   # (grid index, command) not yet in effect
        self.armed = False
        self.steps_done = 0
        self.plant_stepped = False      # since the last (re)initialisation
        self.reinits = 0
        self.armed_reinits = 0
        self.gaps = 0
        self.retransmits = 0
        self.conflicts = 0
        self.ground_alt_msl_m = ground_alt_msl_m
        self.crashes = 0
        self.crashed: Optional[PlantState] = None   # the state the board reads after a crash
        self.last_sane: Optional[PlantState] = None

    def _grid_floor(self, time_us: int) -> int:
        """Index of the last grid point at or before time_us."""
        assert self.t0_us is not None
        return (time_us - self.t0_us) * self.rate_hz // US_PER_S

    def _grid_ceil(self, time_us: int) -> int:
        """Index of the first grid point at or after time_us."""
        assert self.t0_us is not None
        return -((self.t0_us - time_us) * self.rate_hz // US_PER_S)

    def grid_time_us(self, index: int) -> float:
        """Board time of grid point index: t_0 + index / rate."""
        assert self.t0_us is not None
        return self.t0_us + index * US_PER_S / self.rate_hz

    @property
    def plant_time_us(self) -> Optional[float]:
        """Board time of the plant state: t_0 plus whole steps (None before the first frame)."""
        return None if self.t0_us is None else self.grid_time_us(self.steps_done)

    def _initialise(self, time_us: int, command: SurfaceCommand) -> None:
        self.plant.reset()
        self.catapult.reset()
        self.t0_us = time_us
        self.steps_done = 0
        self.plant_stepped = False
        self.held = command             # t_0 is grid point 0: u_0 acts from the first step
        self.pending = None
        self.crashed = None
        self.last_sane = None

    def _apply_pending(self) -> None:
        """The latched command takes over once the plant is at its grid point."""
        if self.pending is not None and self.pending[0] <= self.steps_done:
            self.held = self.pending[1]
            self.pending = None

    def _latch(self, time_us: int, command: SurfaceCommand) -> None:
        """u_k acts from the first grid point at or after t_k; a newer frame before that point supersedes it."""
        self._apply_pending()
        self.pending = (self._grid_ceil(time_us), command)
        self._apply_pending()

    def _advance_to(self, time_us: int) -> tuple[int, float]:
        target = self._grid_floor(time_us)
        substeps = 0
        force = 0.0
        while self.steps_done < target:
            self._apply_pending()
            step_start_us = self.grid_time_us(self.steps_done)
            self.steps_done += 1
            if self.catapult.holding(step_start_us):
                continue                # on the launcher: no physics
            force = self.catapult.force_at(step_start_us)
            self.plant.step(self.held, force, wind_at(self.wind, step_start_us))
            self.plant_stepped = True
            substeps += 1
        return substeps, force

    @property
    def on_launcher(self) -> bool:
        """The plant still stands in its start pose because the launcher holds it."""
        return self.catapult.holds and not self.plant_stepped

    def board_state(self, time_us: int) -> PlantState:
        """The state the board reads at this board time: the plant's, or the launcher's while it holds."""
        if self.crashed is not None:
            return self.crashed
        state = self.plant.state()
        if not self.on_launcher:
            return state
        return self.catapult.held_state(state, time_us, wind_at(self.wind, time_us))

    def _is_new_boot(self, time_us: int) -> bool:
        assert self.last_us is not None
        return time_us < self.last_us or time_us - self.last_us > self.max_jump_us

    def on_actuators(self, time_us: int, controls: Any, armed: bool, flags: int = 0) -> StepResult:
        """Frame k (board time t_k, outputs u_k): advance to t_k with the outputs in effect, latch u_k.

        controls must compare by value (a tuple of PWM, or what the mixer takes)."""
        was_armed = self.armed
        previous = -1 if self.last_us is None else self.last_us
        frame = (controls, bool(armed), int(flags))
        if self.last_us is not None and time_us == self.last_us:
            if frame == self.last_frame:
                self.retransmits += 1
                event = RETRANSMIT
            else:
                self.conflicts += 1
                event = CONFLICT
            return StepResult(event, 0, 0, 0, 0.0, self.on_launcher, was_armed, previous)
        lost = 0
        if self.last_us is None or self._is_new_boot(time_us):
            if self.last_us is None:
                event = FIRST
            else:
                event = REINIT
                self.reinits += 1
                self.armed_reinits += int(was_armed)
            board_dt, substeps, force = 0, 0, 0.0
            self._initialise(time_us, self.mixer(controls))
        else:
            board_dt = time_us - self.last_us
            if board_dt > self.gap_threshold_us:
                event = GAP
                lost = max(1, int(round(board_dt / self.nominal_loop_us)) - 1)
                self.gaps += 1
            else:
                event = STEP
            substeps, force = (0, 0.0) if self.crashed is not None else self._advance_to(time_us)
            self._latch(time_us, self.mixer(controls))
        reason = self._check_crash()
        if reason:
            event = CRASH
        self.catapult.note(time_us, armed)
        self.armed = bool(armed)
        self.last_us = time_us
        self.last_frame = frame
        return StepResult(event, board_dt, substeps, lost, force, self.on_launcher, was_armed, previous, reason)

    def _check_crash(self) -> str:
        """After a frame's steps: the reason when the plant has just left the envelope, else ''."""
        if self.crashed is not None or not self.plant_stepped:
            return ""
        state = self.plant.state()
        reason = crash_reason(state, self.ground_alt_msl_m)
        if not reason:
            self.last_sane = state
            return ""
        self.crashes += 1
        self.crashed = at_rest(self.last_sane if self.last_sane is not None else state)
        return reason
