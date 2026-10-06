import math

import pytest

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.law import (
    FixedTerminalLawConfigProvider,
    PITCH_CEILING_DEG,
    ROLL_LIMIT_CAP_DEG,
    TerminalLawConfig,
    VERTICAL_PN_NAVIGATION_CONSTANT,
    VERTICAL_RATE_FILTER_TAU_S,
    VisionNavLaw,
)
from navpy.modules.navigation.nav.nav_law_factory import VehicleTerminalLawConfigProvider


def _frame(
    *,
    ts=1.0,
    body=(1.0, 0.0, 0.0),
    control=(1.0, 0.0, 0.0),
    roll=0.0,
    airspeed=25.0,
    yaw_rate=0.0,
    pitch=0.0,
):
    return TerminalVisionFrame(
        "cam", 0, 7, 9, ts, *body, *control, roll, airspeed, yaw_rate, pitch
    )


def _unit(angle_y_deg=0.0, angle_z_deg=0.0):
    y = math.tan(math.radians(angle_y_deg))
    z = math.tan(math.radians(angle_z_deg))
    norm = math.sqrt(1.0 + y * y + z * z)
    return 1.0 / norm, y / norm, z / norm


def _law(tau=0.5, throttle=None):
    return VisionNavLaw(FixedTerminalLawConfigProvider(
        TerminalLawConfig(-55.0, 25.0, 45.0, tau, throttle)
    ))


def test_steady_nonzero_bearing_without_los_rate_commands_no_roll():
    # INTENDED BEHAVIOUR CHANGE.  This test previously asserted the opposite --
    # that a 10 deg bearing produced sin(10 deg) of roll -- because the law
    # carried a proportional bearing term.  In wind the collision course is
    # crabbed, so a steady non-zero bearing IS the converged geometry: rolling
    # to null it turns the aircraft off the collision course and can only reach
    # equilibrium with a standing LOS drift.  Zero LOS rate must mean zero roll.
    law = _law(tau=None)
    first = _frame(ts=1.0, control=_unit(angle_y_deg=10.0))
    law.seed(first)
    law.commit(law.plan(first))

    plan = law.plan(_frame(ts=1.1, control=_unit(angle_y_deg=10.0)))

    assert plan.lateral_rate.rate_rad_s == pytest.approx(0.0)
    assert plan.raw_roll_deg == pytest.approx(0.0)
    assert plan.command.cmd_roll_deg == pytest.approx(0.0)


def test_bootstrap_anchor_holds_the_clamped_command_not_measured_attitude():
    # Conditional integration has to hold on the bootstrap path too.  An
    # aircraft at -80 deg pitch against a -55 deg limit issues -55; if the
    # anchor kept the raw -80, the next cycle would integrate from a command
    # that was never issued.  Previously this produced -54.715 on the next
    # frame instead of holding -55.
    law = _law(tau=0.2875, throttle=0.4)
    first = law.plan(_frame(ts=1.0, pitch=-80.0))
    law.commit(first)

    assert first.command.cmd_pitch_deg == -55.0
    assert first.next_anchor.cmd_pitch_deg == -55.0

    second = law.plan(_frame(ts=1.1, pitch=-80.0))

    assert second.command.cmd_pitch_deg == pytest.approx(-55.0)


def test_preview_before_seed_yields_a_flyable_plan_and_does_not_advance():
    # confirmation.py previews a frame to decide whether it is flyable BEFORE
    # seed has ever run.  Preview must therefore produce a command with no
    # prior command state, and must not move the anchor -- otherwise a
    # rejected confirmation would silently integrate.
    law = _law(tau=None)
    frame = _frame(ts=1.0, control=_unit(angle_z_deg=6.0), pitch=-5.0)

    plan = law.preview(frame)

    assert plan is not None
    assert plan.within_limits
    assert plan.command.cmd_pitch_deg == pytest.approx(-5.0)
    assert law.preview(frame).command.cmd_pitch_deg == pytest.approx(-5.0)


def test_seed_without_issuing_a_command_still_anchors_a_clamped_value():
    # record_terminal_confirmed_detection seeds the law WITHOUT issuing a
    # command, so the seeded anchor is what the first real command integrates
    # from. Seeding at -80 deg against a -55 deg limit previously left the raw
    # -80 in the anchor, and the first issued command came out -44.601249.
    law = _law(tau=0.2875, throttle=0.4)
    preview = law.preview(_frame(ts=1.0, pitch=-80.0))
    law.seed(_frame(ts=1.0, pitch=-80.0))

    first = law.plan(_frame(ts=1.1, pitch=-80.0))

    assert preview.command.cmd_pitch_deg == -55.0
    assert first.raw_pitch_deg == pytest.approx(-55.0)
    assert first.command.cmd_pitch_deg == pytest.approx(-55.0)
    assert first.next_anchor.cmd_pitch_deg == pytest.approx(-55.0)


def test_fresh_frame_after_reset_rebootstraps_instead_of_failing():
    # command_reset clears held output AND law state, so the next fresh frame
    # reaches plan() with neither a previous command nor a held command.
    law = _law(tau=None)
    seeded = _frame(ts=1.0, control=_unit(angle_z_deg=6.0), pitch=-5.0)
    law.seed(seeded)
    law.commit(law.plan(seeded))
    law.reset()

    plan = law.plan(_frame(ts=2.0, control=_unit(angle_z_deg=9.0), pitch=-12.0))

    assert plan is not None
    assert plan.bootstrapped
    assert plan.command.cmd_pitch_deg == pytest.approx(-12.0)


def test_lateral_pn_cancels_visual_rate_caused_by_aircraft_turn():
    law = _law(tau=None)
    airspeed = 25.0
    roll = 30.0
    turn_rate = 9.80665 * math.tan(math.radians(roll)) / airspeed
    first = _frame(
        ts=1.0, control=_unit(angle_y_deg=10.0), roll=roll,
        yaw_rate=turn_rate,
    )
    law.seed(first)
    second_bearing = 10.0 - math.degrees(turn_rate) * 0.1

    plan = law.plan(_frame(
        ts=1.1,
        control=_unit(angle_y_deg=second_bearing),
        roll=roll,
        airspeed=airspeed,
        yaw_rate=turn_rate,
    ))

    assert plan.lateral_rate.visual_rate_rad_s == pytest.approx(-turn_rate)
    assert plan.lateral_rate.raw_inertial_rate_rad_s == pytest.approx(0.0)
    assert plan.lateral_rate.rate_rad_s == pytest.approx(0.0)


def test_vertical_command_integrates_from_the_previous_command():
    # INTENDED BEHAVIOUR CHANGE.  The vertical command used to be measured
    # pitch plus an offset, which made the airframe's own response time part of
    # the loop gain and made the effective gain scale with frame rate.  It is
    # now an integrator whose state is the previous COMMAND, advanced by
    # N * los_rate * dt, so its fixed point is zero LOS rate -- the collision
    # course -- and the measured attitude no longer feeds the loop.
    law = _law(tau=0.5)
    first_frame = _frame(ts=10.0, control=_unit(angle_z_deg=10.0), pitch=-3.0)
    law.seed(first_frame)
    law.commit(law.plan(first_frame))
    raw_rate = math.radians(2.0) / 0.2
    # Integrate the filter over the interval, not its endpoint sample.
    filtered = (1.0 + VERTICAL_RATE_FILTER_TAU_S / 0.2
                * math.expm1(-0.2 / VERTICAL_RATE_FILTER_TAU_S)) * raw_rate
    # Anchor is the seeded command (measured pitch of the seed frame), NOT the
    # measured pitch of this frame -- so a wrong pitch here changes nothing.
    expected = -3.0 - math.degrees(
        VERTICAL_PN_NAVIGATION_CONSTANT * filtered * 0.2
    )

    second = law.plan(_frame(
        ts=10.2,
        control=_unit(angle_z_deg=12.0),
        pitch=-7.0,
    ))

    assert second.raw_pitch_deg == pytest.approx(expected)


def test_vertical_command_ignores_measured_pitch_once_anchored():
    # The whole point of command-state anchoring: two frames identical except
    # for the aircraft's measured pitch must produce the same command.  Under
    # the old structure they differed by exactly the pitch difference.
    def _command_for(measured_pitch):
        law = _law(tau=0.5)
        first_frame = _frame(ts=10.0, control=_unit(angle_z_deg=10.0), pitch=-3.0)
        law.seed(first_frame)
        law.commit(law.plan(first_frame))
        return law.plan(_frame(
            ts=10.2,
            control=_unit(angle_z_deg=12.0),
            pitch=measured_pitch,
        )).raw_pitch_deg

    assert _command_for(-7.0) == pytest.approx(_command_for(-30.0))


def test_config_pitch_time_constant_no_longer_reaches_the_command():
    # PTCH2SRV_TCONST used to do two unrelated jobs at once: the rate-filter
    # constant and the rate-to-angle conversion.  It must no longer affect the
    # command at all; it survives only as a diagnostics field.
    def _command_for(tau):
        law = _law(tau=tau)
        first_frame = _frame(ts=10.0, control=_unit(angle_z_deg=10.0), pitch=-3.0)
        law.seed(first_frame)
        law.commit(law.plan(first_frame))
        return law.plan(_frame(
            ts=10.2,
            control=_unit(angle_z_deg=12.0),
            pitch=-7.0,
        )).raw_pitch_deg

    assert _command_for(0.2875) == pytest.approx(_command_for(0.9))
    assert _command_for(0.2875) == pytest.approx(_command_for(None))


def _balloon_geometry_plan():
    """The recorded uav-123 balloon geometry, 0.14 s frame interval."""
    law = _law(tau=0.2875)
    first_frame = _frame(ts=0.0, control=_unit(angle_z_deg=4.6), airspeed=23.0)
    law.seed(first_frame)
    law.commit(law.plan(first_frame))
    plan = law.plan(_frame(
        ts=0.14,
        control=_unit(angle_z_deg=3.5),
        airspeed=23.0,
        pitch=-7.16,
    ))
    return plan, first_frame.aircraft_pitch_deg


def test_decaying_elevation_commands_a_small_pull_up_not_a_balloon():
    # Same recorded uav-123 geometry as before, but the bound is RE-DERIVED for
    # the command-state integrator: the old bound was calibrated to the old
    # formula's tau * N product and is meaningless against an increment.
    #
    # The flat pitch ceiling still applies; the next test separately checks
    # the increment using the exact integral of the rate filter.
    plan, _ = _balloon_geometry_plan()

    assert plan.rate.rate_rad_s < 0.0
    assert plan.command.cmd_pitch_deg <= PITCH_CEILING_DEG


def test_first_slow_frame_increment_is_smaller_than_the_previous_law():
    # Former strict xfail: endpoint*dt overstated the filtered-rate integral.
    # Exact integration satisfies this FIRST-frame bound without changing tau/N.
    # The remaining filter tail still contributes on later frames; this does
    # not establish a settled-response bound or solve every balloon exposure.
    plan, anchor_pitch_deg = _balloon_geometry_plan()

    increment_deg = plan.raw_pitch_deg - anchor_pitch_deg
    assert increment_deg == pytest.approx(
        -math.degrees(
            VERTICAL_PN_NAVIGATION_CONSTANT * plan.rate.rate_rad_s * 0.14
        )
    )
    assert 0.0 < increment_deg < 2.612


def test_pitch_command_can_never_balloon_above_the_flat_ceiling():
    # Structural anti-balloon guard.  Whatever the loop asks for, the command
    # may not climb past PITCH_CEILING_DEG.  A ceiling referenced to the LOS
    # elevation was considered and rejected: it degrades crosswind and can
    # itself command a large nose-up through trajectory feedback.
    law = _law(tau=None)
    first_frame = _frame(ts=0.0, control=_unit(angle_z_deg=14.0), pitch=0.0)
    law.seed(first_frame)
    law.commit(law.plan(first_frame))

    plan = law.plan(_frame(ts=0.05, control=_unit(angle_z_deg=1.0)))

    assert plan.raw_pitch_deg > PITCH_CEILING_DEG
    assert plan.command.cmd_pitch_deg == PITCH_CEILING_DEG
    assert not plan.within_limits


def test_zero_vertical_los_rate_preserves_current_aircraft_pitch():
    law = _law(tau=0.5)
    first = _frame(ts=1.0, control=_unit(angle_z_deg=9.0), pitch=-6.0)
    law.seed(first)
    law.commit(law.plan(first))

    plan = law.plan(_frame(
        ts=1.1,
        control=_unit(angle_z_deg=9.0),
        pitch=-6.0,
    ))

    assert plan.raw_pitch_deg == pytest.approx(-6.0)


@pytest.mark.parametrize("tau", [None, 0.0, -1.0])
def test_absent_or_invalid_config_tau_does_not_disable_the_vertical_channel(tau):
    # INTENDED BEHAVIOUR CHANGE.  Previously a missing or non-positive
    # PTCH2SRV_TCONST silently zeroed the vertical response, because the same
    # value was both the filter constant and the gain.  The filter now has its
    # own constant, so an absent autopilot parameter cannot disable navigation.
    law = _law(tau=tau)
    first_frame = _frame(ts=1.0, control=_unit(angle_z_deg=2.0), pitch=-4.0)
    law.seed(first_frame)
    law.commit(law.plan(first_frame))
    plan = law.plan(_frame(
        ts=1.1,
        control=_unit(angle_z_deg=15.0),
        pitch=-4.0,
    ))
    assert plan.rate.rate_rad_s > 0.0
    assert plan.raw_pitch_deg < -4.0


def test_the_tighter_of_autopilot_limit_and_law_cap_clips_the_command():
    # INTENDED BEHAVIOUR CHANGE.  The autopilot limits used to be the only
    # clipping authority.  The law now also caps roll at ROLL_LIMIT_CAP_DEG and
    # pitch at PITCH_FLOOR_DEG / PITCH_CEILING_DEG, so the tighter of the two
    # binds.  Here the law's roll cap (35) is tighter than the autopilot's 45,
    # and the autopilot's pitch floor (-55) is tighter than the law's -70.
    law = _law(tau=None, throttle=0.0)
    seed = _frame(ts=0.9, control=_unit(angle_z_deg=10.0), pitch=-40.0)
    law.seed(seed)
    law.commit(law.plan(seed))

    plan = law.plan(
        _frame(
            ts=1.0,
            control=_unit(angle_y_deg=25.0, angle_z_deg=24.0),
            pitch=-40.0,
        )
    )

    assert plan.raw_roll_deg > ROLL_LIMIT_CAP_DEG
    assert plan.command.cmd_roll_deg == ROLL_LIMIT_CAP_DEG
    assert plan.raw_pitch_deg < -55.0
    assert plan.command.cmd_pitch_deg == -55.0
    assert plan.command.cmd_thr == 0.0
    assert not plan.within_limits


@pytest.mark.parametrize(
    ("configured_throttle", "trim_throttle"),
    [(150.0, 40.0), (None, 150.0)],
)
def test_terminal_throttle_is_clamped_to_the_mavlink_fraction_range(
    configured_throttle,
    trim_throttle,
):
    class Vehicle:
        params = {
            "PTCH_LIM_MIN_DEG": -55.0,
            "PTCH_LIM_MAX_DEG": 25.0,
            "ROLL_LIMIT_DEG": 45.0,
            "PTCH2SRV_TCONST": 0.5,
            "TRIM_THROTTLE": trim_throttle,
        }

        def get_parameter(self, name, quiet=False):
            assert quiet is True
            return self.params.get(name)

    args = type("Args", (), {"delivery_throttle": configured_throttle})()
    law = VisionNavLaw(VehicleTerminalLawConfigProvider(Vehicle(), args))

    assert law.preview(_frame()).command.cmd_thr == 1.0


def test_preview_and_uncommitted_plan_do_not_mutate_rate_history():
    law = _law()
    law.seed(_frame(ts=1.0, control=_unit(angle_z_deg=5.0)))
    law.preview(_frame(ts=1.0, control=_unit(angle_z_deg=5.0)))
    plan = law.plan(_frame(ts=2.0, control=_unit(angle_z_deg=10.0)))

    expected = (1.0 + VERTICAL_RATE_FILTER_TAU_S
                * math.expm1(-1.0 / VERTICAL_RATE_FILTER_TAU_S)) * math.radians(5.0)
    assert plan.rate.rate_rad_s == pytest.approx(expected)


def test_regressed_direct_plan_preserves_committed_rate_anchor():
    law = _law()
    accepted = _frame(ts=10.0, control=_unit(angle_z_deg=5.0))
    law.seed(accepted)
    law.commit(law.plan(accepted))
    regressed = law.plan(_frame(ts=9.0, control=_unit(angle_z_deg=40.0)))
    law.commit(regressed)
    next_plan = law.plan(_frame(ts=11.0, control=_unit(angle_z_deg=7.0)))
    raw_rate = math.radians(2.0)
    expected = (1.0 + VERTICAL_RATE_FILTER_TAU_S
                * math.expm1(-1.0 / VERTICAL_RATE_FILTER_TAU_S)) * raw_rate

    assert regressed.rate.rate_rad_s == 0.0
    assert next_plan.rate.rate_rad_s == pytest.approx(expected)


def test_actual_parameter_absence_never_defaults_tau_or_throttle():
    class Vehicle:
        params = {
            "PTCH_LIM_MIN_DEG": -60.0,
            "PTCH_LIM_MAX_DEG": 30.0,
            "ROLL_LIMIT_DEG": 50.0,
        }

        @property
        def min_pitch(self):
            raise AssertionError("fallback min_pitch was read")

        @property
        def max_pitch(self):
            raise AssertionError("fallback max_pitch was read")

        @property
        def lim_roll(self):
            raise AssertionError("fallback lim_roll was read")

        def get_parameter(self, name, quiet=False):
            assert quiet is True
            return self.params.get(name)

    args = type("Args", (), {"delivery_throttle": None})()
    config = VehicleTerminalLawConfigProvider(Vehicle(), args).read()
    assert config is not None
    assert config.pitch_time_constant_s is None
    assert config.throttle is None


@pytest.mark.parametrize(
    "updates",
    [
        {"PTCH_LIM_MIN_DEG": None},
        {"PTCH_LIM_MAX_DEG": math.nan},
        {"ROLL_LIMIT_DEG": math.inf},
        {"PTCH_LIM_MIN_DEG": 20.0, "PTCH_LIM_MAX_DEG": 20.0},
        {"PTCH_LIM_MIN_DEG": 30.0, "PTCH_LIM_MAX_DEG": 20.0},
        {"ROLL_LIMIT_DEG": 0.0},
        {"ROLL_LIMIT_DEG": -1.0},
    ],
)
def test_invalid_actual_limits_make_terminal_config_unavailable(updates):
    params = {
        "PTCH_LIM_MIN_DEG": -35.0,
        "PTCH_LIM_MAX_DEG": 18.0,
        "ROLL_LIMIT_DEG": 27.0,
        **updates,
    }
    vehicle = type(
        "Vehicle",
        (),
        {"get_parameter": lambda self, name, quiet=False: params.get(name)},
    )()
    args = type("Args", (), {"delivery_throttle": None})()

    assert VehicleTerminalLawConfigProvider(vehicle, args).read() is None


def test_raw_autopilot_limits_are_the_exact_clipping_authority():
    class Vehicle:
        params = {
            "PTCH_LIM_MIN_DEG": -35.0,
            "PTCH_LIM_MAX_DEG": 18.0,
            "ROLL_LIMIT_DEG": 27.0,
            "PTCH2SRV_TCONST": None,
            "TRIM_THROTTLE": 40.0,
        }

        def __getattr__(self, name):
            if name in {"min_pitch", "max_pitch", "lim_roll"}:
                raise AssertionError(f"fallback property {name} was read")
            raise AttributeError(name)

        def get_parameter(self, name, quiet=False):
            assert quiet is True
            return self.params.get(name)

    args = type("Args", (), {"delivery_throttle": None})()
    law = VisionNavLaw(VehicleTerminalLawConfigProvider(Vehicle(), args))
    seed = _frame(ts=0.9, control=_unit(angle_z_deg=10.0), pitch=-30.0)
    law.seed(seed)
    law.commit(law.plan(seed))

    plan = law.plan(
        _frame(
                ts=1.0,
                control=_unit(angle_y_deg=25.0, angle_z_deg=24.0),
                pitch=-30.0,
        )
    )

    assert plan is not None
    # Both autopilot limits are tighter than the law's own caps here, so they
    # remain the exact clipping authority.
    assert plan.command.cmd_roll_deg == 27.0
    assert plan.command.cmd_pitch_deg == -35.0
    assert plan.command.cmd_thr == 0.4


def test_config_provider_refreshes_actual_parameters_on_law_reset():
    class Vehicle:
        params = {
            "PTCH_LIM_MIN_DEG": -60.0,
            "PTCH_LIM_MAX_DEG": 30.0,
            "ROLL_LIMIT_DEG": 50.0,
            "PTCH2SRV_TCONST": 0.4,
            "TRIM_THROTTLE": 25.0,
        }

        def get_parameter(self, name, quiet=False):
            assert quiet is True
            return self.params.get(name)

    vehicle = Vehicle()
    args = type("Args", (), {"delivery_throttle": None})()
    law = VisionNavLaw(VehicleTerminalLawConfigProvider(vehicle, args))
    assert law.preview(_frame()).command.cmd_thr == 0.25
    vehicle.params["PTCH_LIM_MIN_DEG"] = None
    law.reset()
    assert not law.available
    assert law.plan(_frame()) is None
    vehicle.params["PTCH_LIM_MIN_DEG"] = -45.0
    vehicle.params["TRIM_THROTTLE"] = 0.0
    law.reset()
    assert law.available
    assert law.preview(_frame()).command.cmd_thr == 0.0


def _drive(law, frames):
    """Run a frame sequence through the law, returning every issued command."""
    issued = []
    law.seed(frames[0])
    for frame in frames:
        plan = law.plan(frame)
        if plan is None:
            issued.append(None)
            continue
        law.commit(plan)
        issued.append((plan.command.cmd_roll_deg, plan.command.cmd_pitch_deg))
    return issued


def _wobbling_frames():
    # Irregular dt on purpose: the law integrates rate*dt and low-passes over
    # dt, so a uniform cadence would not exercise the interval-dependent paths.
    steps = (0.026, 0.052, 0.026, 0.104, 0.026, 0.031, 0.078, 0.026)
    frames, ts, bearing, elevation = [], 1.0, 0.0, -2.0
    for index, step in enumerate(steps * 4):
        ts += step
        bearing += 0.35 if index % 3 else -0.5
        elevation -= 0.22
        frames.append(_frame(
            ts=ts,
            control=_unit(angle_y_deg=bearing, angle_z_deg=elevation),
            yaw_rate=0.01 * (index % 5),
            airspeed=25.0 + 0.4 * index,
        ))
    return frames


def test_identical_frame_sequence_produces_bit_identical_commands():
    # Determinism floor.  Run-to-run CPA scatter can come from the environment
    # (frame drops, sensor noise, scheduling) or from the law itself; this
    # separates the two by pinning the law half.  Bit-identical, not approx:
    # the law is pure arithmetic over its inputs, so any difference here is a
    # defect (hidden global state, dict/set ordering, retained state across
    # reset) and would make every live A/B uninterpretable.
    frames = _wobbling_frames()

    first = _drive(_law(tau=None), frames)
    second = _drive(_law(tau=None), frames)

    assert first == second
    assert len(first) == len(frames)
    assert all(command is not None for command in first)


def test_reset_fully_clears_state_so_a_replay_matches_a_fresh_law():
    # The vertical channel anchors each command on the PREVIOUS command, so
    # state outliving reset() would silently make a run depend on whatever
    # flew before it -- exactly the kind of cross-run coupling that would show
    # up as unexplained scatter in a repeat sweep.
    frames = _wobbling_frames()
    law = _law(tau=None)

    _drive(law, frames)
    law.reset()
    after_reset = _drive(law, frames)

    assert after_reset == _drive(_law(tau=None), frames)


def test_an_outlier_hold_stays_labelled_after_its_anchor_is_advanced():
    """The outlier path rebuilds its plan, and the label has to survive that.

    An outlier holds the command and then advances the differentiator reference
    past the discontinuity, which replaces the plan's anchor. Rebuilding the
    plan without carrying the origin relabelled the hold as a normal
    integration, so a log reader -- and the command-causality audit that reads
    those logs -- would try to explain a reissued anchor as a fresh PN command.
    """
    law = _law(tau=None)
    first = _frame(ts=1.0)
    law.seed(first)
    law.commit(law.plan(first))

    # 40 deg of bearing in one frame, past the 30 deg outlier gate.
    plan = law.plan(_frame(ts=1.1, control=_unit(angle_y_deg=40.0)))

    assert plan.reseed is True
    assert plan.origin.reason == "outlier"
    # The held anchor is what the command must be checked against, so it has to
    # come through the rebuild too.
    assert plan.origin.anchor_cmd_roll_deg == pytest.approx(plan.raw_roll_deg)
    assert plan.origin.anchor_cmd_pitch_deg == pytest.approx(plan.raw_pitch_deg)
