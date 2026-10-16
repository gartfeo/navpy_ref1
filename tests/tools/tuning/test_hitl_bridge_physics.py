"""Pure logic of the HITL JSBSim bridge (tools/tuning/hitl_bridge/physics.py, board_clock.py): no board, no JSBSim.

The board-clock stepping follows HIL link contract v2 (fork libraries/SITL/examples/HIL/readme.md);
the stepping vectors marked "reference" are those of the fork's reference bridge unit tests
(Tools/autotest/unittest/hil_bridge_unittest.py, StepperTest)."""
import io
import math
import random

import pytest
from pymavlink.dialects.v20 import common as mavlink2

from tools.tuning.hitl_bridge.catapult import Catapult
from tools.tuning.hitl_bridge.board_clock import (CONFLICT, CRASH, DEFAULT_LOOP_RATE_HZ, FIRST, GAP, GAP_FACTOR,
                                                  GROUND_PENETRATION_CRASH_M, MAX_BODY_RATE_RAD_S, MAX_FORWARD_JUMP_US,
                                                  REINIT, RETRANSMIT, STEP, BoardClockStepper, crash_reason,
                                                  step_rate_hz)
from tools.tuning.hitl_bridge.physics import (NEUTRAL_COMMAND, PlantState, SurfaceCommand, angle_from_pwm,
                                              range_from_pwm, unmix_elevon)
from tools.tuning.hitl_bridge.environment import CALM, WIND_START_BOARD_TIME_S, Wind, wind_at
from tools.tuning.hitl_bridge.plant_options import DEFAULT_SERVO_SPEED_S, PlantOptions, ServoLag, servo_lag_alpha
from tools.tuning.hitl_bridge.state_fields import euler_to_quaternion, hil_state_fields, on_launcher

DT_1200 = 1.0 / 1200.0
DT_1000 = 1.0 / 1000.0              # the reference bridge's PHYSICS_STEP_US 1000
LOOP_US = 5000                      # 200 Hz main loop
T0_US = 1_000_000                   # board time of a first frame
# seed of the jittered frame times below (no unseeded randomness in the tests either)
JITTER_SEED = 20261012


def commands_as_given(command: SurfaceCommand) -> SurfaceCommand:
    """Mixer for stepper tests that feed surface commands instead of PWM."""
    return command


class FakePlant:
    """Records what the stepper does; the state reports how many steps ran since reset."""

    def __init__(self, dt_s: float = DT_1200) -> None:
        self.dt_s = dt_s
        self.resets = 0
        self.steps: list[tuple[SurfaceCommand, float]] = []
        self.winds: list[Wind] = []
        self.steps_since_reset = 0
        self.closed = False

    def reset(self) -> None:
        self.resets += 1
        self.steps_since_reset = 0

    def step(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind = CALM) -> None:
        self.steps.append((command, catapult_force_n))
        self.winds.append(wind)
        self.steps_since_reset += 1

    def state(self) -> PlantState:
        return rest_state(lat_deg=self.steps_since_reset)

    def close(self) -> None:
        self.closed = True


def rest_state(**overrides) -> PlantState:
    values = dict(lat_deg=-35.363261, lon_deg=149.16523, alt_msl_m=584.0, roll_rad=0.0, pitch_rad=0.0,
                  yaw_rad=0.0, p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0, vn_m_s=0.0, ve_m_s=0.0, vd_m_s=0.0,
                  ias_m_s=0.0, tas_m_s=0.0, fx_m_s2=0.0, fy_m_s2=0.0, fz_m_s2=-9.80665)
    values.update(overrides)
    return PlantState(**values)


def cmd(tag: float) -> SurfaceCommand:
    return SurfaceCommand(aileron=tag, elevator=0.0, throttle=0.0)


def no_catapult() -> Catapult:
    return Catapult(force_n=0.0, duration_s=0.0, delay_s=0.0)


# --- unmix --------------------------------------------------------------------------
def test_pwm_normalisation_matches_servo_model():
    assert angle_from_pwm(1000) == -1.0
    assert angle_from_pwm(1500) == 0.0
    assert angle_from_pwm(2000) == 1.0
    assert range_from_pwm(1000) == 0.0
    assert range_from_pwm(1500) == 0.5
    assert range_from_pwm(2000) == 1.0


def test_unmix_is_the_sim_jsbsim_elevon_formula():
    # aileron = (ch2 - ch1)/2, elevator = -(ch2 + ch1)/2 on normalised ch1/ch2, throttle = ch3
    c = unmix_elevon([2000, 1000, 1250] + [0] * 13)
    assert c.aileron == pytest.approx(-1.0)
    assert c.elevator == pytest.approx(0.0)
    assert c.throttle == pytest.approx(0.25)
    c = unmix_elevon([2000, 2000, 2000] + [0] * 13)
    assert (c.aileron, c.elevator, c.throttle) == pytest.approx((0.0, -1.0, 1.0))
    c = unmix_elevon([1600, 1800, 1000] + [0] * 13)
    assert (c.aileron, c.elevator) == pytest.approx(((0.6 - 0.2) / 2, -(0.6 + 0.2) / 2))


def test_no_output_channels_are_held_neutral():
    assert unmix_elevon([0] * 16) == NEUTRAL_COMMAND
    assert unmix_elevon([0, 1500, 0] + [0] * 13) == NEUTRAL_COMMAND


def _arduplane_elevon_pwm(aileron: float, elevator: float, mixing_gain: float = 0.5) -> list[float]:
    """ArduPlane channel_function_mixer (ArduPlane/servos.cpp:188-189, called at :994) with
    SERVO1_FUNCTION 77 (left) and SERVO2_FUNCTION 78 (right), both unreversed, 1000/1500/2000:
    left = (elevator - aileron) * gain, right = (elevator + aileron) * gain, scaled +-4500."""
    def pwm(scaled: float) -> float:
        return 1500 + max(-4500.0, min(4500.0, scaled)) / 4500.0 * 500
    return [pwm((elevator - aileron) * mixing_gain), pwm((elevator + aileron) * mixing_gain), 1000] + [0] * 13


def test_right_roll_demand_gives_positive_jsbsim_aileron():
    # the model's roll moment on fcs/aileron-effective-rad has a positive coefficient, so positive = right roll
    c = unmix_elevon(_arduplane_elevon_pwm(aileron=4500, elevator=0))
    assert c.aileron == pytest.approx(0.5)
    assert c.elevator == pytest.approx(0.0)


def test_nose_up_demand_gives_negative_jsbsim_elevator():
    # the model's pitch moment on fcs/elevator-effective-rad has a negative coefficient, so negative = nose up
    c = unmix_elevon(_arduplane_elevon_pwm(aileron=0, elevator=4500))
    assert c.elevator == pytest.approx(-0.5)
    assert c.aileron == pytest.approx(0.0)


# --- conversions ----------------------------------------------------------------------
def _rotate(q, v):
    """Rotate vector v (body) by quaternion q = (w, x, y, z) body-to-NED."""
    w, x, y, z = q
    m = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def test_quaternion_is_body_to_ned_wxyz():
    assert euler_to_quaternion(0, 0, 0) == pytest.approx((1, 0, 0, 0))
    half = math.sqrt(0.5)
    assert euler_to_quaternion(0, 0, math.pi / 2) == pytest.approx((half, 0, 0, half))
    # heading 90 deg: the nose (body x) points east
    assert _rotate(euler_to_quaternion(0, 0, math.pi / 2), [1, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-12)
    # pitch up 30 deg: the nose points up, i.e. negative down
    nose = _rotate(euler_to_quaternion(0, math.radians(30), 0), [1, 0, 0])
    assert nose == pytest.approx([math.cos(math.radians(30)), 0, -0.5], abs=1e-12)
    # right roll 90 deg: the right wing (body y) points down
    assert _rotate(euler_to_quaternion(math.pi / 2, 0, 0), [0, 1, 0]) == pytest.approx([0, 0, 1], abs=1e-12)
    q = euler_to_quaternion(0.3, -0.2, 2.5)
    assert sum(c * c for c in q) == pytest.approx(1.0)


def test_level_at_rest_is_minus_1000_mg():
    f = hil_state_fields(rest_state())
    assert (f["xacc"], f["yacc"], f["zacc"]) == (0, 0, -1000)


def test_state_fields_units():
    s = rest_state(lat_deg=-35.363261, lon_deg=149.16523, alt_msl_m=584.1234, vn_m_s=12.344, ve_m_s=-3.0,
                   vd_m_s=0.512, ias_m_s=23.456, tas_m_s=25.0, p_rad_s=0.1, q_rad_s=-0.2, r_rad_s=0.3,
                   fx_m_s2=1.0, fy_m_s2=-2.0, fz_m_s2=-19.6133)
    f = hil_state_fields(s)
    assert f["lat"] == -353632610 and f["lon"] == 1491652300        # degE7
    assert f["alt"] == 584123                                       # mm MSL
    assert (f["vx"], f["vy"], f["vz"]) == (1234, -300, 51)            # cm/s NED
    assert (f["ind_airspeed"], f["true_airspeed"]) == (2346, 2500)   # cm/s
    assert (f["rollspeed"], f["pitchspeed"], f["yawspeed"]) == (0.1, -0.2, 0.3)
    assert (f["xacc"], f["yacc"], f["zacc"]) == (102, -204, -2000)   # mG
    assert all(isinstance(f[k], int) for k in ("lat", "lon", "alt", "vx", "vy", "vz", "ind_airspeed",
                                                 "true_airspeed", "xacc", "yacc", "zacc"))


def test_state_fields_clamp_to_mavlink_ranges():
    f = hil_state_fields(rest_state(fz_m_s2=-400.0, fx_m_s2=400.0, vn_m_s=500.0, ias_m_s=-1.0))
    assert (f["zacc"], f["xacc"], f["vx"], f["ind_airspeed"]) == (-32768, 32767, 32767, 0)


# --- catapult -------------------------------------------------------------------------
def test_catapult_fires_once_after_the_first_armed_frame():
    cat = Catapult(force_n=350.0, duration_s=0.5, delay_s=3.0)
    assert cat.force_at(20_000_000) == 0.0                     # never armed
    cat.note(10_000_000, armed=False)
    cat.note(11_000_000, armed=True)
    cat.note(12_000_000, armed=True)                           # later armed frames do not move it
    assert cat.force_at(13_999_999) == 0.0
    assert cat.force_at(14_000_000) == 350.0
    assert cat.force_at(14_499_999) == 350.0
    assert cat.force_at(14_500_000) == 0.0
    cat.note(15_000_000, armed=False)
    cat.note(16_000_000, armed=True)                           # re-arming does not fire again
    assert cat.force_at(19_200_000) == 0.0
    cat.reset()
    cat.note(30_000_000, armed=True)
    assert cat.force_at(33_100_000) == 350.0


# --- stepping on the board clock ---------------------------------------------------------
def test_first_message_only_initialises():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    r = stepper.on_actuators(7_000_000, cmd(1), armed=False)
    assert (r.event, r.substeps, r.board_dt_us) == (FIRST, 0, 0)
    assert plant.resets == 1 and plant.steps == []


def test_zero_order_hold_uses_the_previous_outputs():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(1_000_000, cmd(1), armed=False)
    r = stepper.on_actuators(1_000_000 + LOOP_US, cmd(2), armed=False)
    assert (r.event, r.substeps, r.board_dt_us) == (STEP, 6, LOOP_US)
    assert stepper.held == cmd(2)               # a frame on the grid acts from its own grid point
    assert [c.aileron for c, _ in plant.steps] == [1] * 6
    stepper.on_actuators(1_000_000 + 2 * LOOP_US, cmd(3), armed=False)
    assert [c.aileron for c, _ in plant.steps[6:]] == [2] * 6


def test_steps_follow_the_board_clock_without_drift():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    t = 123_456
    stepper.on_actuators(t, NEUTRAL_COMMAND, armed=False)
    for k in range(1, 201):
        r = stepper.on_actuators(t + k * LOOP_US, NEUTRAL_COMMAND, armed=False)
        assert r.substeps == 6
    assert len(plant.steps) == 1200                            # 1 s of board time at 1/1200 s


def test_jittered_frames_keep_whole_steps_up_to_board_time():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(0, NEUTRAL_COMMAND, armed=False)
    assert stepper.on_actuators(4_900, NEUTRAL_COMMAND, armed=False).substeps == 5    # 5.88 steps
    assert stepper.on_actuators(10_100, NEUTRAL_COMMAND, armed=False).substeps == 7   # 12.12 in total
    # Changed behaviour (contract v2): the same frame at the same time is a retransmission, answered
    # unstepped (v1 told frames apart by a counter in flags; v2 has none)
    again = stepper.on_actuators(10_100, NEUTRAL_COMMAND, armed=False)
    assert (again.event, again.substeps) == (RETRANSMIT, 0)


def test_backwards_time_reinitialises():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(9_000_000, cmd(1), armed=False)
    stepper.on_actuators(9_005_000, cmd(1), armed=False)
    r = stepper.on_actuators(2_000_000, cmd(5), armed=False)       # board rebooted
    assert (r.event, r.substeps) == (REINIT, 0)
    assert plant.resets == 2 and stepper.reinits == 1
    stepper.on_actuators(2_005_000, cmd(6), armed=False)
    assert plant.steps_since_reset == 6
    assert [c.aileron for c, _ in plant.steps[-6:]] == [5] * 6        # held from the re-init frame


def test_a_forward_gap_with_lost_frames_is_stepped_across_with_the_held_outputs():
    # The board keeps flying through a link pause, so the physics steps across it with the held
    # outputs and reports it. Changed behaviour (contract v2): only up to MAX_FORWARD_JUMP_US
    # (2 s); a longer jump is a new boot (next test), where v1 stepped across 30 s.
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(1_000_000, cmd(1), armed=True)
    r = stepper.on_actuators(2_500_000, cmd(2), armed=True)      # 1.5 s USB stall, 299 frames lost
    assert (r.event, r.lost_frames, r.board_dt_us, r.substeps) == (GAP, 299, 1_500_000, 1800)
    assert plant.resets == 1 and stepper.reinits == 0 and stepper.gaps == 1
    assert {c.aileron for c, _ in plant.steps} == {1}                 # held across the gap
    assert stepper.on_actuators(2_505_000, cmd(2), armed=True).event == STEP


def test_a_forward_jump_beyond_the_maximum_is_a_new_boot():
    # reference: test_forward_jump_beyond_max_reinitialises
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(T0_US, NEUTRAL_COMMAND, armed=False)
    assert stepper.on_actuators(T0_US + MAX_FORWARD_JUMP_US, NEUTRAL_COMMAND, armed=False).event == GAP
    beyond = stepper.on_actuators(T0_US + 2 * MAX_FORWARD_JUMP_US + 1, NEUTRAL_COMMAND, armed=False)
    assert (beyond.event, beyond.substeps) == (REINIT, 0)
    assert stepper.t0_us == T0_US + 2 * MAX_FORWARD_JUMP_US + 1 and plant.resets == 2


def test_a_board_stall_is_a_gap_from_board_time_alone():
    # Changed behaviour (contract v2): v1 told a stall of the board's own loop from lost frames
    # by its frame counter; v2 has no counter, so any interval over GAP_FACTOR loop periods is a GAP
    stepper = BoardClockStepper(FakePlant(), no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(1_000_000, NEUTRAL_COMMAND, armed=False)
    r = stepper.on_actuators(1_800_000, NEUTRAL_COMMAND, armed=False)    # the board's own loop took 0.8 s
    assert (r.event, r.lost_frames, r.substeps) == (GAP, 159, 960)


def test_flags_are_not_a_frame_counter():
    # Changed behaviour (contract v2): v1 re-initialised when the counter in flags went back; v2
    # flags carry only the lockstep bit, and a reboot shows in board time alone
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(3_000_000, NEUTRAL_COMMAND, armed=False, flags=500)
    assert stepper.on_actuators(3_005_000, NEUTRAL_COMMAND, armed=False, flags=1).event == STEP
    assert plant.resets == 1


def test_lost_frames_are_estimated_from_board_time_against_the_nominal_loop():
    # reference: test_gap_from_board_time_against_nominal_loop
    stepper = BoardClockStepper(FakePlant(dt_s=DT_1000), no_catapult(), mixer=commands_as_given,
                                loop_rate_hz=DEFAULT_LOOP_RATE_HZ)
    threshold = int(GAP_FACTOR * LOOP_US)
    stepper.on_actuators(T0_US, NEUTRAL_COMMAND, armed=False)
    t_us = T0_US + threshold
    assert stepper.on_actuators(t_us, NEUTRAL_COMMAND, armed=False).event == STEP
    t_us += threshold + 1
    gap = stepper.on_actuators(t_us, NEUTRAL_COMMAND, armed=False)
    assert (gap.event, gap.lost_frames) == (GAP, 1)
    t_us += 4 * LOOP_US
    assert stepper.on_actuators(t_us, NEUTRAL_COMMAND, armed=False).lost_frames == 3
    assert stepper.gaps == 2
    assert stepper.steps_done == (t_us - T0_US) // 1000          # stepped across like any interval


def test_the_loop_rate_sets_the_gap_threshold():
    stepper = BoardClockStepper(FakePlant(), no_catapult(), mixer=commands_as_given, loop_rate_hz=50)
    stepper.on_actuators(T0_US, NEUTRAL_COMMAND, armed=False)
    assert stepper.on_actuators(T0_US + 20_000, NEUTRAL_COMMAND, armed=False).event == STEP


# --- contract v2: causal grid rule, retransmission, conflicts, reboots ----------------------
def test_outputs_take_effect_at_the_next_grid_point():
    """reference: a step that straddles t_k still uses u_(k-1), and of two frames before the
    same grid point only the newer acts there"""
    plant = FakePlant(dt_s=DT_1000)
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    u = [cmd(k) for k in range(5)]
    stepper.on_actuators(T0_US, u[0], armed=False)
    stepper.on_actuators(T0_US + 1500, u[1], armed=False)     # acts from +2000
    stepper.on_actuators(T0_US + 3200, u[2], armed=False)     # would act from +4000 ...
    stepper.on_actuators(T0_US + 4000, u[3], armed=False)     # ... but u3 is newer there
    stepper.on_actuators(T0_US + 6000, u[4], armed=False)
    assert [c for c, _ in plant.steps] == [u[0], u[0], u[1], u[1], u[3], u[3]]
    assert stepper.held == u[4]


@pytest.mark.parametrize("dt_s", [DT_1000, DT_1200])
def test_each_step_uses_the_newest_frame_at_or_before_its_start(dt_s):
    """reference: the rule above over a long jittered session, frames faster and slower than the
    step, here also on the bridge's own 1/1200 s grid (counted exactly in integers)"""
    rng = random.Random(JITTER_SEED)
    plant = FakePlant(dt_s=dt_s)
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    frames, t_us = [], T0_US
    for k in range(400):
        frames.append((t_us, cmd(k)))
        t_us += rng.choice((1, 250, 833, 834, 999, 1000, 1001, 1500, 4321, 5000))
    for t, command in frames:
        stepper.on_actuators(t, command, armed=False)
    rate = step_rate_hz(dt_s)
    # grid point n is at T0 + n/rate s; a frame at t acts there when t * rate <= that, in integers
    want = [[c for t, c in frames if (t - T0_US) * rate <= n * 1_000_000][-1] for n in range(len(plant.steps))]
    assert [c for c, _ in plant.steps] == want
    assert stepper.steps_done == (frames[-1][0] - T0_US) * rate // 1_000_000


def test_the_integration_step_must_be_one_over_a_whole_number_of_seconds():
    assert (step_rate_hz(DT_1200), step_rate_hz(DT_1000), step_rate_hz(1 / 400)) == (1200, 1000, 400)
    for bad in (0.0, -0.001, 0.0015, 1 / 1200.5):
        with pytest.raises(ValueError):
            step_rate_hz(bad)
        with pytest.raises(ValueError):
            BoardClockStepper(FakePlant(dt_s=bad), no_catapult())


def test_the_same_frame_again_is_a_retransmission():
    """reference: a lockstep SITL resends a frame whose reply was lost with a connection: answered
    as before, nothing stepped, latched or reset, and the run goes on as if it had come once"""
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(T0_US, cmd(0), armed=True, flags=1)
    stepper.on_actuators(T0_US + LOOP_US, cmd(1), armed=True, flags=1)
    steps = list(plant.steps)
    r = stepper.on_actuators(T0_US + LOOP_US, cmd(1), armed=True, flags=1)
    assert (r.event, r.substeps, r.board_dt_us) == (RETRANSMIT, 0, 0)
    assert (stepper.retransmits, stepper.conflicts, stepper.reinits) == (1, 0, 0)
    assert plant.steps == steps and plant.resets == 1 and stepper.held == cmd(1)
    stepper.on_actuators(T0_US + 2 * LOOP_US, cmd(1), armed=True, flags=1)
    assert [c for c, _ in plant.steps[len(steps):]] == [cmd(1)] * 6


@pytest.mark.parametrize("other", [(cmd(9), False, 0), (cmd(0), True, 0), (cmd(0), False, 1)])
def test_the_same_time_with_other_content_is_a_conflict(other):
    """reference: board time strictly increases within one boot, so another frame at the same time
    breaks the contract: counted, nothing stepped, latched or reset"""
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(T0_US, cmd(0), armed=False)
    stepper.on_actuators(T0_US + LOOP_US, cmd(0), armed=False)
    command, armed, flags = other
    r = stepper.on_actuators(T0_US + LOOP_US, command, armed=armed, flags=flags)
    assert (r.event, r.substeps) == (CONFLICT, 0)
    assert (stepper.conflicts, stepper.retransmits, stepper.reinits) == (1, 0, 0)
    assert plant.resets == 1 and stepper.held == cmd(0) and not stepper.armed
    assert stepper.plant_time_us == T0_US + LOOP_US


def test_a_reboot_while_armed_is_counted():
    """reference: re-initialising while the previous frame had the armed bit is never silent"""
    stepper = BoardClockStepper(FakePlant(), no_catapult(), mixer=commands_as_given)
    stepper.on_actuators(T0_US, NEUTRAL_COMMAND, armed=True)
    r = stepper.on_actuators(T0_US // 2, NEUTRAL_COMMAND, armed=False)
    assert r.was_armed and r.previous_time_us == T0_US and stepper.armed_reinits == 1
    stepper.on_actuators(T0_US // 4, NEUTRAL_COMMAND, armed=False)
    assert (stepper.armed_reinits, stepper.reinits) == (1, 2)


def test_catapult_force_reaches_the_plant_by_substep_time():
    plant = FakePlant()
    # armed at t=0: force from 10.2 ms to 15.2 ms; substeps start every 833.3 us, so
    # the steps starting at 10.83 .. 15.00 ms (numbers 13..18) carry it
    stepper = BoardClockStepper(plant, Catapult(force_n=350.0, duration_s=0.005, delay_s=0.0102),
                                mixer=commands_as_given)
    stepper.on_actuators(0, NEUTRAL_COMMAND, armed=True)
    for k in range(1, 6):
        stepper.on_actuators(k * LOOP_US, NEUTRAL_COMMAND, armed=True)
    forces = [f for _, f in plant.steps]
    assert [i for i, f in enumerate(forces) if f] == list(range(13, 19))
    assert set(forces) == {0.0, 350.0}


def test_reinit_rearms_the_catapult():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, Catapult(force_n=100.0, duration_s=1.0, delay_s=0.0), mixer=commands_as_given)
    stepper.on_actuators(0, NEUTRAL_COMMAND, armed=True)
    stepper.on_actuators(LOOP_US, NEUTRAL_COMMAND, armed=True)
    assert plant.steps[-1][1] == 100.0
    stepper.on_actuators(0, NEUTRAL_COMMAND, armed=False)          # reboot, disarmed
    stepper.on_actuators(LOOP_US, NEUTRAL_COMMAND, armed=False)
    assert plant.steps[-1][1] == 0.0
    assert stepper.catapult.armed_at_us is None


# --- launcher (Catapult.holds) ---------------------------------------------------------
def launcher(delay_s: float = 0.0102) -> Catapult:
    return Catapult(force_n=350.0, duration_s=0.005, delay_s=delay_s, holds=True)


def test_a_launcher_holds_the_aircraft_without_physics_until_the_stroke():
    """On its skids the stroke ended below the lift-off speed and the aircraft ground-looped
    (HITL on SITL); a launcher holds it at rest, so the plant's first step is the stroke's."""
    plant = FakePlant()
    stepper = BoardClockStepper(plant, launcher(), mixer=commands_as_given)
    results = [stepper.on_actuators(0, NEUTRAL_COMMAND, armed=False)]
    results += [stepper.on_actuators(k * LOOP_US, NEUTRAL_COMMAND, armed=k >= 2) for k in range(1, 7)]
    # armed at 10 ms, stroke 20.2 .. 25.2 ms: the steps starting at 20.83 .. 25.0 ms (6 of them)
    # are the first the plant takes, all with the force, then the free flight
    assert [r.on_launcher for r in results] == [True] * 5 + [False] * 2
    assert [r.substeps for r in results] == [0, 0, 0, 0, 0, 5, 6]
    assert [f for _, f in plant.steps] == [350.0] * 6 + [0.0] * 5
    assert stepper.steps_done == 36                 # the board time passed on the launcher counts


def test_holding_ends_only_with_the_stroke_and_never_without_a_launcher():
    cat = launcher(delay_s=3.0)
    assert cat.holding(1e12)                        # never armed: still on the launcher
    cat.note(10_000_000, armed=True)
    assert cat.holding(12_999_999) and not cat.holding(13_000_000) and not cat.holding(1e12)
    cat.reset()
    assert cat.holding(13_000_000)
    assert not Catapult(350.0, 0.5, 3.0).holding(0)   # on the skids: the contacts carry it


def test_a_board_reboot_puts_the_aircraft_back_on_the_launcher():
    plant = FakePlant()
    stepper = BoardClockStepper(plant, launcher(delay_s=0.0), mixer=commands_as_given)
    stepper.on_actuators(0, NEUTRAL_COMMAND, armed=True)
    assert stepper.on_actuators(LOOP_US, NEUTRAL_COMMAND, armed=True).on_launcher is False
    flown = len(plant.steps)
    assert stepper.on_actuators(0, NEUTRAL_COMMAND, armed=False).on_launcher is True     # reboot
    assert stepper.on_actuators(LOOP_US, NEUTRAL_COMMAND, armed=False).on_launcher is True
    assert len(plant.steps) == flown and plant.resets == 2


def test_an_aircraft_on_the_launcher_reports_rest_and_the_launchers_support():
    pitch = math.radians(10.0)
    moving = rest_state(pitch_rad=pitch, vd_m_s=0.008, p_rad_s=1e-5, ias_m_s=0.01, fz_m_s2=0.0)   # free fall
    held = on_launcher(moving)
    g = 9.80665
    assert (held.fx_m_s2, held.fy_m_s2, held.fz_m_s2) == pytest.approx((g * math.sin(pitch), 0.0, -g * math.cos(pitch)))
    assert (held.vd_m_s, held.p_rad_s, held.ias_m_s, held.tas_m_s) == (0.0, 0.0, 0.0, 0.0)
    assert (held.pitch_rad, held.alt_msl_m, held.lat_deg) == (pitch, moving.alt_msl_m, moving.lat_deg)
    level = hil_state_fields(on_launcher(rest_state(fz_m_s2=0.0)))
    assert (level["xacc"], level["yacc"], level["zacc"]) == (0, 0, -1000)
    rolled = on_launcher(rest_state(roll_rad=math.radians(30.0)))
    assert rolled.fy_m_s2 == pytest.approx(-g * 0.5)


# --- servo lag and wind (plant_options) ---------------------------------------------------
def test_servo_lag_alpha_is_arduplanes_lowpass_alpha_for_sim_servo_speed():
    # ServoModel.cpp:59 cutoff = 1/(2 pi SIM_SERVO_SPEED); AP_Math.cpp calc_lowpass_alpha_dt:
    # rc = 1/(2 pi cutoff), alpha = dt/(dt + rc)
    cutoff_hz = 1.0 / (2 * math.pi * 0.14)
    rc = 1.0 / (2 * math.pi * cutoff_hz)
    assert servo_lag_alpha(DT_1200, 0.14) == pytest.approx(DT_1200 / (DT_1200 + rc), rel=1e-12)
    assert DEFAULT_SERVO_SPEED_S == 0.14 and PlantOptions().servo_speed_s == 0.14
    assert servo_lag_alpha(DT_1200, 0.0) == 1.0


def test_servo_lag_is_a_first_order_step_response_on_all_three_commands():
    lag = ServoLag(servo_lag_alpha(DT_1200, 0.14))
    target = SurfaceCommand(aileron=0.5, elevator=-1.0, throttle=1.0)
    first = lag.apply(target)
    a = servo_lag_alpha(DT_1200, 0.14)
    assert (first.aileron, first.elevator, first.throttle) == pytest.approx((0.5 * a, -a, a))
    for _ in range(167):                                    # 168 steps = 0.14 s, one time constant
        out = lag.apply(target)
    assert out.throttle == pytest.approx(1 - (1 - a) ** 168, rel=1e-12)
    assert out.throttle == pytest.approx(1 - math.exp(-1), abs=0.003)
    lag.reset()
    assert lag.output == NEUTRAL_COMMAND


def test_zero_servo_speed_passes_the_command_through_unchanged():
    lag = ServoLag(servo_lag_alpha(DT_1200, 0.0))
    command = SurfaceCommand(aileron=0.1, elevator=0.7, throttle=0.3)
    assert lag.apply(command) is command


def test_wind_is_given_by_the_direction_it_blows_from():
    north, east = Wind(speed_m_s=5.0, from_deg=270.0).air_velocity_ne_m_s()     # a westerly moves the air east
    assert (north, east) == pytest.approx((0.0, 5.0), abs=1e-12)
    assert Wind(4.0, 0.0).air_velocity_ne_m_s() == pytest.approx((-4.0, 0.0))  # from the north: towards south
    assert CALM.air_velocity_ne_m_s() == (0.0, 0.0) and str(CALM.air_velocity_ne_m_s()[0]) == "0.0"


def test_the_wind_starts_5_s_of_board_time_after_boot_as_in_sitl():
    # SITL_State.cpp:327-331 keeps the wind at 0 for 5 s so the airspeed sensor calibrates at 0;
    # board time is micros64 since boot, so this also holds after a reboot mid-run
    wind = Wind(6.0, 90.0)
    assert WIND_START_BOARD_TIME_S == 5.0
    assert wind_at(wind, 4_999_999) == CALM and wind_at(wind, 5_000_000) == wind
    plant = FakePlant()
    stepper = BoardClockStepper(plant, no_catapult(), wind, mixer=commands_as_given)
    stepper.on_actuators(4_990_000, NEUTRAL_COMMAND, armed=False)
    stepper.on_actuators(4_995_000, NEUTRAL_COMMAND, armed=False)
    stepper.on_actuators(5_000_000, NEUTRAL_COMMAND, armed=False)
    stepper.on_actuators(5_005_000, NEUTRAL_COMMAND, armed=False)
    assert plant.winds == [CALM] * 12 + [wind] * 6       # by the start time of each step
    stepper.on_actuators(300_000, NEUTRAL_COMMAND, armed=False)     # reboot: calm again
    stepper.on_actuators(305_000, NEUTRAL_COMMAND, armed=False)
    assert plant.winds[-6:] == [CALM] * 6


# --- crash: the plant leaves the envelope -------------------------------------------------
class CrashingPlant(FakePlant):
    """Flies level at 600 m MSL, then after crash_after steps gives the given (broken) state."""

    def __init__(self, crash_after: int, broken: dict) -> None:
        super().__init__()
        self.crash_after, self.broken = crash_after, broken

    def state(self) -> PlantState:
        if self.steps_since_reset > self.crash_after:
            return rest_state(**{"alt_msl_m": 600.0, **self.broken})
        return rest_state(alt_msl_m=600.0, lat_deg=-35.0, vn_m_s=25.0, ias_m_s=25.0, tas_m_s=25.0,
                          pitch_rad=0.1)


@pytest.mark.parametrize("broken, why", [
    ({"q_rad_s": 1e40}, "body rate"),                       # bench 2026-10-13: no float32 holds it
    ({"q_rad_s": float("nan")}, "not a finite number: q_rad_s"),
    ({"alt_msl_m": 575.0}, "9.0 m under the ground"),
])
def test_a_crash_freezes_the_aircraft_at_rest_where_it_last_was_and_the_bridge_keeps_answering(broken, why):
    plant = CrashingPlant(crash_after=12, broken=broken)
    stepper = BoardClockStepper(plant, no_catapult(), mixer=commands_as_given, ground_alt_msl_m=584.0)
    events = [stepper.on_actuators(T0_US + k * LOOP_US, cmd(1), armed=True) for k in range(6)]
    assert [r.event for r in events] == [FIRST, STEP, STEP, CRASH, STEP, STEP]
    assert why in events[3].crash and stepper.crashes == 1
    assert len(plant.steps) == 18                            # not stepped after the crash
    frozen = stepper.board_state(T0_US + 5 * LOOP_US)
    assert (frozen.lat_deg, frozen.alt_msl_m, frozen.pitch_rad) == (-35.0, 600.0, 0.1)   # the last sane place
    assert (frozen.vn_m_s, frozen.ias_m_s, frozen.q_rad_s) == (0.0, 0.0, 0.0)
    fields = hil_state_fields(frozen)
    mavlink2.MAVLink(io.BytesIO()).hil_state_quaternion_encode(T0_US, **fields).pack(mavlink2.MAVLink(io.BytesIO()))
    reboot = stepper.on_actuators(T0_US, cmd(1), armed=False)
    assert reboot.event == REINIT and stepper.crashed is None
    assert stepper.board_state(T0_US).alt_msl_m == 600.0


def test_the_envelope_check_passes_a_normal_state_and_needs_the_ground_for_the_height_rule():
    assert crash_reason(rest_state(alt_msl_m=584.08), 584.0) == ""
    assert crash_reason(rest_state(alt_msl_m=584.0 - GROUND_PENETRATION_CRASH_M + 0.01), 584.0) == ""
    assert crash_reason(rest_state(alt_msl_m=500.0), None) == ""
    assert crash_reason(rest_state(r_rad_s=-MAX_BODY_RATE_RAD_S - 1), None).startswith("body rate")
